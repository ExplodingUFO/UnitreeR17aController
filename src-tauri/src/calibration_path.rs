//! Offline geometry for the complete calibration programme. Samples are actual
//! joint configurations and FK TCP positions, not a Cartesian bounding box.
use crate::kinematics::{
    Frame, Kinematics, LocalTarget, MotionLimits, Pose, StepInput, offset_pose, pose_error,
};
use serde::{Deserialize, Serialize};

const END_TOLERANCE: f64 = 1e-6;
const PATH_TOLERANCE: f64 = 0.00002;
const ORIENTATION_TOLERANCE: f64 = 0.0002;
const MIN_DIRECTION: f64 = 0.0001;
const JOINT_SAMPLE_STEP: f64 = 0.01;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PreviewRequest {
    pub amplitude: f64,
    pub speed: f64,
    pub axes: Vec<usize>,
}

impl Default for PreviewRequest {
    fn default() -> Self {
        Self {
            amplitude: 0.05,
            speed: 0.01,
            axes: vec![0, 1, 2],
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CartesianSegment {
    pub axis: usize,
    /// Absolute signed displacement from the original TCP in base metres.
    pub offset: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CartesianProgram {
    pub side: String,
    pub origin: [f64; 7],
    /// Flange-to-TCP transform used for both planning and execution.
    pub tcp: Pose,
    pub amplitude: f64,
    pub speed: f64,
    /// Full joint excursion in radians; each joint visits +/- half of this.
    #[serde(default = "default_joint_excursion")]
    pub joint_excursion: f64,
    pub segments: Vec<CartesianSegment>,
}

fn default_joint_excursion() -> f64 {
    16.0_f64.to_radians()
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PreviewPoint {
    pub joints: [f64; 7],
    pub tcp: [f64; 3],
    /// Cartesian segment index, then three joint legs per axis after that;
    /// the Cartesian phase's joint return uses segments.len() + 21.
    pub segment: usize,
    pub space: String,
    /// Base XYZ index for Cartesian samples, joint index for joint samples;
    /// ignored (zero) for space="joint_return", which moves all joints.
    pub axis: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CalibrationPreview {
    pub program: CartesianProgram,
    pub points: Vec<PreviewPoint>,
}

#[derive(Clone, Copy)]
struct Sample {
    joints: [f64; 7],
    tcp: [f64; 3],
}

/// Plans geometry only: no feedback, device I/O, control state or timing claims.
/// Each direction is independent, so a blocked outward leg cannot hide another
/// useful axis or the seven joint trials that are also part of calibration.
pub fn preview(
    model: &Kinematics,
    side: &str,
    origin: [f64; 7],
    tcp: Pose,
    request: PreviewRequest,
) -> Result<CalibrationPreview, String> {
    if !matches!(side, "right" | "left") {
        return Err("side must be right or left".into());
    }
    if !request.amplitude.is_finite() || !(0.02..=0.10).contains(&request.amplitude) {
        return Err("末端单向幅度为 20–100 mm".into());
    }
    if !request.speed.is_finite() || !(0.005..=0.0135).contains(&request.speed) {
        return Err("末端测试速度为 5–13.5 mm/s".into());
    }
    if request.axes.iter().any(|axis| *axis >= 3) {
        return Err("末端测试方向为基座 X、Y、Z".into());
    }
    let origin_pose = model.fk(&origin, &tcp)?;
    let start = Sample {
        joints: origin,
        tcp: origin_pose.translation,
    };
    let mut program = CartesianProgram {
        side: side.into(),
        origin,
        tcp,
        amplitude: request.amplitude,
        speed: request.speed,
        joint_excursion: default_joint_excursion(),
        segments: Vec::new(),
    };
    let mut points = Vec::new();
    let mut current = start;
    for axis in 0..3 {
        if !request.axes.contains(&axis) {
            continue;
        }
        let axis_start = current;
        let segment_start = program.segments.len();
        let point_start = points.len();
        for target in [program.amplitude, -program.amplitude] {
            let samples = trace(model, &program, origin_pose, current, axis, target);
            let endpoint = *samples.last().unwrap();
            let offset = endpoint.tcp[axis] - start.tcp[axis];
            if target.signum() * offset >= MIN_DIRECTION {
                append_cartesian(&mut program, &mut points, axis, offset, &samples);
                current = endpoint;
            }
        }
        if program.segments.len() == segment_start {
            continue;
        }
        let returning = trace(model, &program, origin_pose, current, axis, 0.0);
        let endpoint = *returning.last().unwrap();
        if (endpoint.tcp[axis] - start.tcp[axis]).abs() <= END_TOLERANCE {
            append_cartesian(&mut program, &mut points, axis, 0.0, &returning);
            current = endpoint;
        } else {
            // An axis that cannot complete its return is omitted as a whole.
            // Other axes and joint trials still get their own usable preview.
            program.segments.truncate(segment_start);
            points.truncate(point_start);
            current = axis_start;
        }
    }
    append_joint_return(model, &program, current, &mut points)?;
    append_joint_trials(model, &program, start, &mut points)?;
    Ok(CalibrationPreview { program, points })
}

fn trace(
    model: &Kinematics,
    program: &CartesianProgram,
    origin_pose: Pose,
    start: Sample,
    axis: usize,
    target_offset: f64,
) -> Vec<Sample> {
    let mut samples = vec![start];
    // These bounds keep the offline IK local. Reset derivatives and measured
    // joints on each step: the preview is geometry, not simulated servo motion.
    let limits = MotionLimits {
        joint_velocity: [0.2; 7],
        joint_acceleration: [2.0; 7],
        max_reference_error: 0.1,
        max_joint_step: JOINT_SAMPLE_STEP,
        linear_speed: program.speed,
        angular_speed: 0.15,
    };
    // A finite budget also handles directions whose projection vanishes at a
    // reach boundary. Only accepted actual FK samples survive a clipped leg.
    for _ in 0..2000 {
        let previous = *samples.last().unwrap();
        let distance = previous.tcp[axis] - origin_pose.translation[axis];
        let remaining = target_offset - distance;
        if remaining.abs() <= END_TOLERANCE {
            break;
        }
        let mut delta = [0.0; 6];
        delta[axis] = distance + remaining.clamp(-0.002, 0.002);
        let Ok(target) = offset_pose(&origin_pose, delta, Frame::Base) else {
            break;
        };
        let Ok(candidate) = model.step(StepInput {
            reference: previous.joints,
            measured: previous.joints,
            previous_velocity: [0.0; 7],
            target: LocalTarget::Pose(target),
            dt: 0.1,
            tcp: program.tcp,
            limits,
            smoothing: None,
        }) else {
            break;
        };
        let Ok(pose) = model.fk(&candidate.q, &program.tcp) else {
            break;
        };
        let error = pose_error(&origin_pose, &pose);
        let projection = error[axis];
        let transverse = (0..3)
            .filter(|component| *component != axis)
            .map(|component| error[component].powi(2))
            .sum::<f64>()
            .sqrt();
        let rotation = error[3..].iter().map(|v| v * v).sum::<f64>().sqrt();
        if !projection.is_finite()
            || remaining.signum() * (projection - distance) < 1e-8
            || projection.abs() > program.amplitude
            || transverse > PATH_TOLERANCE
            || rotation > ORIENTATION_TOLERANCE
        {
            break;
        }
        samples.push(Sample {
            joints: candidate.q,
            tcp: pose.translation,
        });
    }
    samples
}

fn append_cartesian(
    program: &mut CartesianProgram,
    points: &mut Vec<PreviewPoint>,
    axis: usize,
    offset: f64,
    samples: &[Sample],
) {
    let segment = program.segments.len();
    for sample in samples {
        points.push(PreviewPoint {
            joints: sample.joints,
            tcp: sample.tcp,
            segment,
            space: "cartesian".into(),
            axis,
        });
    }
    program.segments.push(CartesianSegment { axis, offset });
}

fn append_joint_return(
    model: &Kinematics,
    program: &CartesianProgram,
    start: Sample,
    points: &mut Vec<PreviewPoint>,
) -> Result<(), String> {
    let largest = (0..7)
        .map(|axis| (program.origin[axis] - start.joints[axis]).abs())
        .fold(0.0_f64, f64::max);
    let steps = (largest / JOINT_SAMPLE_STEP).ceil().max(1.0) as usize;
    for step in 0..=steps {
        let joints = if step == steps {
            program.origin
        } else {
            std::array::from_fn(|axis| {
                start.joints[axis]
                    + (program.origin[axis] - start.joints[axis]) * step as f64 / steps as f64
            })
        };
        points.push(PreviewPoint {
            joints,
            tcp: model.fk(&joints, &program.tcp)?.translation,
            segment: program.segments.len() + 21,
            space: "joint_return".into(),
            axis: 0,
        });
    }
    Ok(())
}

fn append_joint_trials(
    model: &Kinematics,
    program: &CartesianProgram,
    origin: Sample,
    points: &mut Vec<PreviewPoint>,
) -> Result<(), String> {
    let half = program.joint_excursion / 2.0;
    for axis in 0..7 {
        let mut previous = 0.0;
        for (leg, target) in [half, -half, 0.0].into_iter().enumerate() {
            let steps = ((target - previous).abs() / JOINT_SAMPLE_STEP).ceil() as usize;
            for step in 0..=steps {
                let offset = previous + (target - previous) * step as f64 / steps as f64;
                let mut joints = origin.joints;
                joints[axis] += offset;
                points.push(PreviewPoint {
                    joints,
                    tcp: model.fk(&joints, &program.tcp)?.translation,
                    segment: program.segments.len() + axis * 3 + leg,
                    space: "joint".into(),
                    axis,
                });
            }
            previous = target;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn origin() -> [f64; 7] {
        [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3]
    }

    fn tool() -> Pose {
        offset_pose(
            &Pose::identity(),
            [0.025, -0.015, 0.10, 0.2, -0.3, 0.4],
            Frame::Tool,
        )
        .unwrap()
    }

    fn check_samples(model: &Kinematics, preview: &CalibrationPreview) {
        let program = &preview.program;
        let original = model.fk(&program.origin, &program.tcp).unwrap();
        assert!(!preview.points.is_empty());
        for point in &preview.points {
            let pose = model.fk(&point.joints, &program.tcp).unwrap();
            assert_eq!(point.tcp, pose.translation);
            assert!(
                point
                    .joints
                    .iter()
                    .chain(point.tcp.iter())
                    .all(|v| v.is_finite())
            );
            match point.space.as_str() {
                "cartesian" => {
                    assert!(point.segment < program.segments.len());
                    assert_eq!(point.axis, program.segments[point.segment].axis);
                    let error = pose_error(&original, &pose);
                    assert!(error[point.axis].abs() <= program.amplitude + 1e-12);
                    for (axis, value) in error.iter().enumerate().take(3) {
                        if axis != point.axis {
                            assert!(value.abs() <= PATH_TOLERANCE);
                        }
                    }
                    assert!(
                        error[3..].iter().map(|v| v * v).sum::<f64>().sqrt()
                            <= ORIENTATION_TOLERANCE
                    );
                }
                "joint" => {
                    assert_eq!((point.segment - program.segments.len()) / 3, point.axis);
                    for axis in 0..7 {
                        let delta = point.joints[axis] - program.origin[axis];
                        if axis == point.axis {
                            assert!(delta.abs() <= program.joint_excursion / 2.0 + 1e-12);
                        } else {
                            assert_eq!(delta, 0.0);
                        }
                    }
                }
                "joint_return" => {
                    assert_eq!(point.segment, program.segments.len() + 21);
                    assert_eq!(point.axis, 0);
                }
                other => panic!("unexpected sample space {other}"),
            }
        }
        for pair in preview.points.windows(2) {
            for axis in 0..7 {
                assert!(
                    (pair[1].joints[axis] - pair[0].joints[axis]).abs()
                        <= JOINT_SAMPLE_STEP + 1e-12
                );
            }
        }
        for (segment, target) in program.segments.iter().enumerate() {
            let last = preview
                .points
                .iter()
                .rfind(|p| p.space == "cartesian" && p.segment == segment)
                .unwrap();
            assert!(
                (last.tcp[target.axis] - original.translation[target.axis] - target.offset).abs()
                    <= END_TOLERANCE
            );
            if target.offset == 0.0 {
                let pose = model.fk(&last.joints, &program.tcp).unwrap();
                let error = pose_error(&original, &pose);
                assert!(error[..3].iter().all(|value| value.abs() <= PATH_TOLERANCE));
            }
        }
        assert_eq!(preview.points.last().unwrap().joints, program.origin);
        assert_eq!(preview.points.last().unwrap().tcp, original.translation);
    }

    fn assert_close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.iter().zip(expected) {
            assert!(
                (actual - expected).abs() <= 1e-14 * (1.0 + expected.abs()),
                "{actual} != {expected}"
            );
        }
    }

    fn assert_same_program(actual: &CartesianProgram, expected: &CartesianProgram) {
        assert_eq!(actual.side, expected.side);
        assert_close(&actual.origin, &expected.origin);
        assert_close(&actual.tcp.translation, &expected.tcp.translation);
        assert_close(&actual.tcp.rotation, &expected.tcp.rotation);
        assert_close(
            &[actual.amplitude, actual.speed, actual.joint_excursion],
            &[expected.amplitude, expected.speed, expected.joint_excursion],
        );
        assert_eq!(actual.segments.len(), expected.segments.len());
        for (actual, expected) in actual.segments.iter().zip(&expected.segments) {
            assert_eq!(actual.axis, expected.axis);
            assert_close(&[actual.offset], &[expected.offset]);
        }
    }

    #[test]
    fn default_request_and_program_envelope_round_trip() {
        let request: PreviewRequest = serde_json::from_str("{}").unwrap();
        assert_eq!(request, PreviewRequest::default());
        let partial: PreviewRequest = serde_json::from_str(r#"{"speed":0.005}"#).unwrap();
        assert_eq!(partial.amplitude, 0.05);
        assert_eq!(partial.axes, vec![0, 1, 2]);
        let model = Kinematics::load("right").unwrap();
        let result = preview(&model, "right", origin(), tool(), request).unwrap();
        assert_eq!(result.program.joint_excursion, 16.0_f64.to_radians());
        let mut value = serde_json::to_value(&result.program).unwrap();
        value.as_object_mut().unwrap().remove("joint_excursion");
        let restored: CartesianProgram = serde_json::from_value(value).unwrap();
        assert_same_program(&restored, &result.program);
        let encoded = serde_json::to_string(&result).unwrap();
        let decoded: CalibrationPreview = serde_json::from_str(&encoded).unwrap();
        assert_same_program(&decoded.program, &result.program);
        assert_eq!(decoded.points.len(), result.points.len());
        for (actual, expected) in decoded.points.iter().zip(&result.points) {
            assert_eq!(actual.space, expected.space);
            assert_eq!(actual.axis, expected.axis);
            assert_eq!(actual.segment, expected.segment);
            assert_close(&actual.joints, &expected.joints);
            assert_close(&actual.tcp, &expected.tcp);
        }
    }

    #[test]
    fn redundant_ik_drift_and_the_full_joint_return_are_in_the_preview() {
        let model = Kinematics::load("right").unwrap();
        let result = preview(&model, "right", origin(), tool(), PreviewRequest::default()).unwrap();
        check_samples(&model, &result);
        let last_cartesian = result
            .points
            .iter()
            .rfind(|point| point.space == "cartesian")
            .unwrap();
        let returning: Vec<_> = result
            .points
            .iter()
            .filter(|point| point.space == "joint_return")
            .collect();
        assert!(
            last_cartesian
                .joints
                .iter()
                .zip(origin())
                .any(|(a, b)| (a - b).abs() > 1e-7)
        );
        assert!(returning.len() >= 2);
        assert_eq!(returning[0].joints, last_cartesian.joints);
        assert_eq!(returning.last().unwrap().joints, origin());
        for (index, point) in returning.iter().enumerate() {
            let fraction = index as f64 / (returning.len() - 1) as f64;
            let expected: [f64; 7] = std::array::from_fn(|axis| {
                last_cartesian.joints[axis]
                    + (origin()[axis] - last_cartesian.joints[axis]) * fraction
            });
            assert_close(&point.joints, &expected);
        }
    }

    #[test]
    fn both_arms_preview_base_xyz_at_the_selected_tcp_without_orientation_drift() {
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            for tcp in [Pose::identity(), tool()] {
                let result =
                    preview(&model, side, origin(), tcp, PreviewRequest::default()).unwrap();
                check_samples(&model, &result);
                assert_eq!(result.program.segments.len(), 9, "{side}, {tcp:?}");
                for axis in 0..3 {
                    let segments = &result.program.segments[axis * 3..axis * 3 + 3];
                    assert!(segments.iter().all(|segment| segment.axis == axis));
                    assert!(
                        (segments[0].offset - 0.05).abs() <= END_TOLERANCE,
                        "{side} +{axis}: {}",
                        segments[0].offset
                    );
                    assert!(
                        (segments[1].offset + 0.05).abs() <= END_TOLERANCE,
                        "{side} -{axis}: {}",
                        segments[1].offset
                    );
                    assert_eq!(segments[2].offset, 0.0);
                }
            }
        }
    }

    #[test]
    fn every_joint_sweep_has_both_eight_degree_extremes_and_returns_to_origin() {
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            let result = preview(
                &model,
                side,
                origin(),
                tool(),
                PreviewRequest {
                    axes: vec![],
                    ..PreviewRequest::default()
                },
            )
            .unwrap();
            assert!(result.program.segments.is_empty());
            check_samples(&model, &result);
            for axis in 0..7 {
                let samples: Vec<_> = result
                    .points
                    .iter()
                    .filter(|point| point.space == "joint" && point.axis == axis)
                    .collect();
                let minimum = samples
                    .iter()
                    .map(|point| point.joints[axis] - origin()[axis])
                    .fold(f64::INFINITY, f64::min);
                let maximum = samples
                    .iter()
                    .map(|point| point.joints[axis] - origin()[axis])
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!((minimum + 8.0_f64.to_radians()).abs() < 1e-12);
                assert!((maximum - 8.0_f64.to_radians()).abs() < 1e-12);
                assert_eq!(samples.last().unwrap().joints, origin());
                for leg in 0..3 {
                    assert!(samples.iter().any(|point| point.segment == axis * 3 + leg));
                }
            }
            let point = result
                .points
                .iter()
                .find(|point| point.joints != origin())
                .unwrap();
            assert_ne!(
                point.tcp,
                model
                    .fk(&point.joints, &Pose::identity())
                    .unwrap()
                    .translation
            );
        }
    }

    #[test]
    fn clipping_and_singular_starts_keep_other_directions_and_joint_trials() {
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            let result = preview(
                &model,
                side,
                [0.0; 7],
                Pose::identity(),
                PreviewRequest {
                    amplitude: 0.10,
                    ..PreviewRequest::default()
                },
            )
            .unwrap();
            check_samples(&model, &result);
            assert!(
                !result.program.segments.is_empty(),
                "{side}: all Cartesian directions lost"
            );
            assert!(
                result.program.segments.len() < 9
                    || result
                        .program
                        .segments
                        .iter()
                        .any(|segment| segment.offset != 0.0
                            && segment.offset.abs() < 0.10 - END_TOLERANCE),
                "{side}: expected an outward reach boundary"
            );
            assert!((0..7).all(|axis| {
                result
                    .points
                    .iter()
                    .any(|point| point.space == "joint" && point.axis == axis)
            }));
        }
    }

    #[test]
    fn selected_axes_are_deduplicated_and_ordered_in_base_coordinates() {
        let model = Kinematics::load("right").unwrap();
        let result = preview(
            &model,
            "right",
            origin(),
            tool(),
            PreviewRequest {
                amplitude: 0.02,
                speed: 0.0135,
                axes: vec![2, 0, 2],
            },
        )
        .unwrap();
        check_samples(&model, &result);
        assert_eq!(
            result
                .program
                .segments
                .iter()
                .map(|s| s.axis)
                .collect::<Vec<_>>(),
            vec![0, 0, 0, 2, 2, 2]
        );
        assert!(
            !result
                .points
                .iter()
                .any(|point| point.space == "cartesian" && point.axis == 1)
        );
    }

    #[test]
    fn invalid_requests_fail_before_geometry() {
        let model = Kinematics::load("right").unwrap();
        for value in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            0.0,
            0.01999,
            0.10001,
        ] {
            assert!(
                preview(
                    &model,
                    "right",
                    origin(),
                    tool(),
                    PreviewRequest {
                        amplitude: value,
                        ..PreviewRequest::default()
                    }
                )
                .is_err()
            );
        }
        for value in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            0.0,
            0.00499,
            0.01351,
        ] {
            assert!(
                preview(
                    &model,
                    "right",
                    origin(),
                    tool(),
                    PreviewRequest {
                        speed: value,
                        ..PreviewRequest::default()
                    }
                )
                .is_err()
            );
        }
        assert!(
            preview(
                &model,
                "right",
                origin(),
                tool(),
                PreviewRequest {
                    axes: vec![3],
                    ..PreviewRequest::default()
                }
            )
            .is_err()
        );
        assert!(preview(&model, "other", origin(), tool(), PreviewRequest::default()).is_err());
        let mut invalid_origin = origin();
        invalid_origin[2] = f64::NAN;
        assert!(
            preview(
                &model,
                "right",
                invalid_origin,
                tool(),
                PreviewRequest::default()
            )
            .is_err()
        );
        assert!(
            preview(
                &model,
                "right",
                origin(),
                Pose {
                    rotation: [0.0; 4],
                    ..tool()
                },
                PreviewRequest::default()
            )
            .is_err()
        );
    }
}
