//! Offline geometry for the complete calibration programme. Samples are actual
//! joint configurations and FK TCP positions, not a Cartesian bounding box.
use crate::calibration_spline::TimedLeg;
use crate::kinematics::{
    ContinuationBounds, Frame, JointGuide, Kinematics, Pose, offset_pose, pose_error,
};
use crate::trajectory::MotionTuning;
use serde::{Deserialize, Serialize};
use std::{sync::Arc, time::Instant};

#[cfg(test)]
const END_TOLERANCE: f64 = 1e-6;
const PATH_TOLERANCE: f64 = 0.00002;
const ORIENTATION_TOLERANCE: f64 = 0.0002;
const JOINT_SAMPLE_STEP: f64 = 0.01;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PreviewRequest {
    /// None selects the complete locally reachable range in each direction.
    pub amplitude: Option<f64>,
    pub speed: f64,
    pub axes: Vec<usize>,
    pub joint_speeds: Vec<f64>,
    pub motion: MotionTuning,
}

impl Default for PreviewRequest {
    fn default() -> Self {
        Self {
            amplitude: None,
            speed: 0.01,
            axes: vec![0, 1, 2],
            joint_speeds: Vec::new(),
            motion: MotionTuning::default(),
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preview_id: Option<String>,
    #[serde(skip)]
    pub timed: Vec<Arc<TimedLeg>>,
    #[serde(skip)]
    pub joint_legs: [Vec<Arc<TimedLeg>>; 7],
    #[serde(default)]
    pub joint_speeds: Vec<f64>,
    pub side: String,
    pub origin: [f64; 7],
    /// Flange-to-TCP transform used for both planning and execution.
    pub tcp: Pose,
    pub amplitude: f64,
    pub speed: f64,
    /// Full joint excursion in radians; each joint visits +/- half of this.
    #[serde(default = "default_joint_excursion")]
    pub joint_excursion: f64,
    /// Absolute joint targets after clipping the requested excursion to model travel.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint_targets: Option<[[f64; 2]; 7]>,
    #[serde(default)]
    pub motion: MotionTuning,
    #[serde(default)]
    pub auto_range: bool,
    pub segments: Vec<CartesianSegment>,
    #[serde(default)]
    pub guide: Vec<BranchPoint>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct BranchPoint {
    pub axis: usize,
    pub offset: f64,
    pub joints: [f64; 7],
}

/// A short blend from the last transmitted hold joins the already fitted leg.
/// A joint trial consists solely of this same rest-to-rest move.
#[derive(Clone, Debug)]
pub struct Playback {
    pub approach: crate::teaching::Approach,
    pub leg: Option<Arc<TimedLeg>>,
}
impl Playback {
    pub fn duration(&self) -> f64 {
        self.approach.duration + self.leg.as_ref().map_or(0.0, |leg| leg.duration())
    }

    pub fn sample(&self, t: f64) -> crate::teaching::Sample {
        match &self.leg {
            Some(leg) if t >= self.duration() => leg.sample(leg.duration()),
            Some(leg) if t >= self.approach.duration => leg.sample(t - self.approach.duration),
            _ => self.approach.sample(t),
        }
    }
}

/// A held phase doubles as the controller's acknowledgement of that particular
/// pause. Only the writer knows when the first resumed batch actually starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlaybackPhase {
    Original(Instant),
    Held {
        since: Instant,
        offset: f64,
    },
    Resuming {
        started: Instant,
        offset: f64,
        k: f64,
    },
}

impl PlaybackPhase {
    pub fn for_send(self, published: Self, at: Instant, limits: MotionTuning) -> Self {
        if let Self::Held { offset, .. } = self
            && self == published
        {
            // f' <= (1-f)/k, |f''| <= 6(1-f)/k². The two jerk terms
            // 3q''ff' and q'f'' each get half of the unused jerk allowance.
            let k = (limits.joint_speed / limits.acceleration)
                .max(6.0 * (limits.acceleration / limits.jerk))
                .max(12.0_f64.sqrt() * (limits.joint_speed.sqrt() / limits.jerk.sqrt()));
            return Self::Resuming {
                started: at,
                offset,
                k,
            };
        }
        self
    }

    /// Spline time and its first two wall-clock derivatives. For a restart,
    /// the integral t*x² has derivative smoothstep(x), x=t/(t+k).
    fn clock(self, at: Instant) -> (f64, f64, f64) {
        match self {
            Self::Original(started) => (
                at.saturating_duration_since(started).as_secs_f64(),
                1.0,
                0.0,
            ),
            Self::Held { offset, .. } => (offset, 0.0, 0.0),
            Self::Resuming { started, offset, k } => {
                let t = at.saturating_duration_since(started).as_secs_f64();
                let x = t / (t + k);
                (
                    offset + t * x * x,
                    x * x * (3.0 - 2.0 * x),
                    6.0 * x * (1.0 - x).powi(3) / k,
                )
            }
        }
    }

    pub fn progress(self, at: Instant) -> f64 {
        self.clock(at).0
    }

    pub fn sample(self, plan: &Playback, at: Instant) -> crate::teaching::Sample {
        let (time, rate, acceleration) = self.clock(at);
        let mut sample = plan.sample(time);
        for i in 0..7 {
            sample.ddq[i] = sample.ddq[i] * rate * rate + sample.dq[i] * acceleration;
            sample.dq[i] *= rate;
        }
        sample
    }
}

impl CartesianProgram {
    /// Bind confirmation to the geometry and tuning even when another client
    /// has replaced the one-entry cache. JSON clients normalize signed zero.
    pub fn content_id(&self) -> String {
        use sha2::{Digest, Sha256};
        let mut hash = Sha256::new();
        hash.update(b"r17a-calibration-spline-v1");
        hash.update(self.side.as_bytes());
        hash.update([self.auto_range as u8]);
        let mut number = |v: f64| hash.update(if v == 0.0 { 0.0 } else { v }.to_le_bytes());
        for value in self
            .origin
            .into_iter()
            .chain(self.tcp.translation)
            .chain(self.tcp.rotation)
            .chain([
                self.amplitude,
                self.speed,
                self.joint_excursion,
                self.motion.joint_speed,
                self.motion.acceleration,
                self.motion.jerk,
            ])
            .chain(self.joint_targets.into_iter().flatten().flatten())
        {
            number(value);
        }
        number(self.joint_speeds.len() as f64);
        self.joint_speeds.iter().copied().for_each(&mut number);
        number(self.segments.len() as f64);
        for s in &self.segments {
            number(s.axis as f64);
            number(s.offset);
        }
        for p in &self.guide {
            number(p.axis as f64);
            number(p.offset);
            p.joints.into_iter().for_each(&mut number);
        }
        format!("{:x}", hash.finalize())
    }

    fn fit_cartesian(
        &self,
        model: &Kinematics,
        axis: usize,
        knots: &[(f64, [f64; 7])],
    ) -> Result<TimedLeg, String> {
        let fit = || {
            let mut leg = TimedLeg::fit(knots, self.speed, self.motion, model.speed_limits())?;
            leg.limit_tcp_speed(model, &self.tcp, self.speed)?;
            Ok::<_, String>(leg)
        };
        fit().map_err(|e| {
            format!(
                "{} 方向的时间规划失败，调整速度或行程后重新预览。{e}",
                ["X", "Y", "Z"][axis]
            )
        })
    }

    fn plan_joint_trials(
        &mut self,
        model: &Kinematics,
        bounds: ContinuationBounds,
    ) -> Result<(), String> {
        self.set_joint_targets(bounds);
        if self.joint_speeds.is_empty() {
            self.joint_speeds =
                crate::calibration::speeds(&crate::calibration::Settings::default());
        }
        let mut legs: [Vec<Arc<TimedLeg>>; 7] = Default::default();
        for (axis, (trials, [low, high])) in
            legs.iter_mut().zip(self.joint_targets.unwrap()).enumerate()
        {
            let mut start = self.origin;
            for &speed in &self.joint_speeds {
                for q in [high, low, self.origin[axis]] {
                    let mut end = self.origin;
                    end[axis] = q;
                    let leg = TimedLeg::fit(
                        &[(start[axis], start), (q, end)],
                        speed,
                        self.motion,
                        model.speed_limits(),
                    )
                    .map_err(|_| {
                        format!(
                            "J{} 试验速度或时长无法表示，请调整关节速度后重新预览",
                            axis + 1
                        )
                    })?;
                    trials.push(Arc::new(leg));
                    start = end;
                }
            }
        }
        self.joint_legs = legs;
        Ok(())
    }

    fn set_joint_targets(&mut self, bounds: ContinuationBounds) {
        let half = self.joint_excursion * 0.5;
        self.joint_targets = Some(std::array::from_fn(|i| {
            [
                (self.origin[i] - half).max(bounds.lower[i]),
                (self.origin[i] + half).min(bounds.upper[i]),
            ]
        }));
    }

    /// Legacy or non-cached requests are prepared off the control thread. Follow
    /// each requested turnaround, including the original branch on its way back.
    pub fn prepare(&mut self, model: &Kinematics) -> Result<(), String> {
        self.motion.validate()?;
        let origin = model.fk(&self.origin, &self.tcp)?;
        let start = Sample {
            joints: self.origin,
            tcp: origin.translation,
        };
        let bounds = model.continuation_bounds(&self.origin)?;
        self.plan_joint_trials(model, bounds)?;
        // Old API clients may supply just target offsets. Resolve their knots
        // once before dispatch, so they get the same playback as a new preview.
        if self.preview_id.is_none()
            && self.segments.iter().any(|s| {
                !self
                    .guide
                    .iter()
                    .any(|k| k.axis == s.axis && k.offset == s.offset)
            })
        {
            let mut guide = Vec::new();
            for segment in &mut self.segments {
                let axis = segment.axis;
                if axis >= 3 || !segment.offset.is_finite() {
                    return Err("末端试验方向或位移无效".into());
                }
                let samples = trace(model, self.tcp, origin, start, axis, segment.offset, bounds);
                let end = samples.last().unwrap().tcp[axis] - start.tcp[axis];
                if (end - segment.offset).abs() > PATH_TOLERANCE {
                    return Err(format!(
                        "{} 方向可达 {:.1} mm，请重新预览行程",
                        ["X", "Y", "Z"][axis],
                        end * 1000.0
                    ));
                }
                segment.offset = end;
                guide.extend(samples.into_iter().map(|sample| BranchPoint {
                    axis,
                    offset: sample.tcp[axis] - start.tcp[axis],
                    joints: sample.joints,
                }));
            }
            guide.sort_by(|a, b| a.axis.cmp(&b.axis).then(a.offset.total_cmp(&b.offset)));
            guide.dedup_by(|a, b| a.axis == b.axis && a.offset == b.offset);
            self.guide = guide;
        }

        for point in &self.guide {
            if point.axis >= 3 {
                return Err("预览方向无效".into());
            }
            let mut delta = [0.0; 6];
            delta[point.axis] = point.offset;
            let goal = offset_pose(&origin, delta, Frame::Base)?;
            let error = pose_error(&goal, &model.fk(&point.joints, &self.tcp)?);
            if (0..7)
                .any(|i| point.joints[i] < bounds.lower[i] || point.joints[i] > bounds.upper[i])
                || error[..3].iter().any(|v| v.abs() > PATH_TOLERANCE)
                || error[3..].iter().any(|v| v.abs() > ORIENTATION_TOLERANCE)
            {
                return Err("已保存的路径与当前模型不符，请重新生成预览".into());
            }
        }
        let mut previous = 0.0;
        let mut last_joints = self.origin;
        let mut timed = Vec::with_capacity(self.segments.len());
        for segment in &self.segments {
            let range = segment.offset.min(previous)..=segment.offset.max(previous);
            let mut knots: Vec<_> = self
                .guide
                .iter()
                .filter(|p| p.axis == segment.axis && range.contains(&p.offset))
                .map(|p| (p.offset, p.joints))
                .collect();
            if previous > segment.offset {
                knots.reverse();
            }
            if knots.first() != Some(&(previous, last_joints))
                || knots.last().map(|k| k.0) != Some(segment.offset)
            {
                return Err("预览节点与试验起止位置不一致，请重新生成预览".into());
            }
            if knots.len() == 1 {
                knots.push(knots[0]);
            }
            timed.push(Arc::new(self.fit_cartesian(model, segment.axis, &knots)?));
            previous = segment.offset;
            last_joints = knots.last().unwrap().1;
        }
        self.timed = timed;
        Ok(())
    }

    /// Interpolate the preview branch at actual signed TCP progress using the
    /// accepted geometric samples. No IK runs in this lookup.
    pub fn guide_at(&self, axis: usize, offset: f64) -> Option<JointGuide> {
        let first = self.guide.partition_point(|point| point.axis < axis);
        let end = self.guide.partition_point(|point| point.axis <= axis);
        let points = &self.guide[first..end];
        if points.len() < 2 {
            return None;
        }
        let next = points
            .partition_point(|point| point.offset <= offset)
            .clamp(1, points.len() - 1);
        let a = &points[next - 1];
        let b = &points[next];
        let distance = b.offset - a.offset;
        if distance <= 0.0 {
            return None;
        }
        let fraction = ((offset - a.offset) / distance).clamp(0.0, 1.0);
        Some(JointGuide {
            axis,
            position: std::array::from_fn(|i| a.joints[i] + (b.joints[i] - a.joints[i]) * fraction),
            tangent: std::array::from_fn(|i| (b.joints[i] - a.joints[i]) / distance),
        })
    }
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_s: Option<f64>,
    pub space: String,
    /// Base XYZ index for Cartesian samples, joint index for joint samples;
    /// ignored (zero) for space="joint_return", which moves all joints.
    pub axis: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CalibrationPreview {
    pub program: CartesianProgram,
    pub points: Vec<PreviewPoint>,
    #[serde(default)]
    pub ranges: Vec<AxisRange>,
    #[serde(default)]
    pub cartesian_duration_s: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AxisRange {
    pub axis: usize,
    /// Signed displacements from the original TCP in base metres.
    pub negative: f64,
    pub positive: f64,
}

#[derive(Clone, Copy)]
struct Sample {
    joints: [f64; 7],
    tcp: [f64; 3],
}

/// Generate geometry and fit its executable, time-parameterized joint path offline.
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
    if request
        .amplitude
        .is_some_and(|value| !value.is_finite() || value <= 0.0)
    {
        return Err("末端单向幅度必须为有限正数".into());
    }
    if !request.speed.is_finite() || request.speed <= 0.0 {
        return Err("末端测试速度必须为有限正数".into());
    }
    request.motion.validate()?;
    if request.axes.iter().any(|axis| *axis >= 3) {
        return Err("末端测试方向为基座 X、Y、Z".into());
    }
    let origin_pose = model.fk(&origin, &tcp)?;
    let bounds = model.continuation_bounds(&origin)?;
    let start = Sample {
        joints: origin,
        tcp: origin_pose.translation,
    };
    let mut program = CartesianProgram {
        preview_id: None,
        timed: Vec::new(),
        joint_legs: Default::default(),
        joint_speeds: request.joint_speeds,
        side: side.into(),
        origin,
        tcp,
        amplitude: request.amplitude.unwrap_or(0.0),
        speed: request.speed,
        joint_excursion: default_joint_excursion(),
        joint_targets: None,
        motion: request.motion,
        auto_range: request.amplitude.is_none(),
        segments: Vec::new(),
        guide: Vec::new(),
    };
    program.plan_joint_trials(model, bounds)?;
    let mut points = Vec::new();
    let mut ranges = Vec::with_capacity(3);
    // The sphere only bounds the search. Every accepted point is solved from
    // its neighbour with the selected TCP orientation held fixed.
    let search_distance = 2.0 * model.reach_radius(&tcp);
    if !search_distance.is_finite() || search_distance <= 0.0 {
        return Err("invalid model geometry extent".into());
    }
    for axis in 0..3 {
        let positive = trace(
            model,
            tcp,
            origin_pose,
            start,
            axis,
            search_distance,
            bounds,
        );
        let negative = trace(
            model,
            tcp,
            origin_pose,
            start,
            axis,
            -search_distance,
            bounds,
        );
        let range = AxisRange {
            axis,
            negative: negative.last().unwrap().tcp[axis] - start.tcp[axis],
            positive: positive.last().unwrap().tcp[axis] - start.tcp[axis],
        };
        if program.auto_range {
            program.amplitude = program.amplitude.max(range.positive).max(-range.negative);
        }
        ranges.push(range);
        if !request.axes.contains(&axis) {
            continue;
        }
        let positive = clip_trace(
            model,
            tcp,
            origin_pose,
            axis,
            positive,
            request.amplitude,
            bounds,
        );
        let negative = clip_trace(
            model,
            tcp,
            origin_pose,
            axis,
            negative,
            request.amplitude,
            bounds,
        );
        let mut knots = Vec::new();
        for samples in [&negative, &positive] {
            for sample in samples {
                knots.push(BranchPoint {
                    axis,
                    offset: sample.tcp[axis] - start.tcp[axis],
                    joints: sample.joints,
                });
            }
        }
        knots.sort_by(|a, b| a.offset.total_cmp(&b.offset));
        knots.dedup_by(|a, b| a.offset == b.offset);
        program.guide.extend(knots);
        if positive.len() == 1 && negative.len() == 1 {
            continue;
        }
        if positive.len() > 1 {
            let offset = positive.last().unwrap().tcp[axis] - start.tcp[axis];
            append_cartesian(
                &mut program,
                &mut points,
                axis,
                offset,
                &positive,
                model,
                start.tcp[axis],
            )?;
        }
        if negative.len() > 1 {
            // Reverse the accepted positive branch through the exact starting
            // configuration, then continue out along the negative branch.
            let mut crossing: Vec<_> = positive.iter().rev().copied().collect();
            crossing.extend_from_slice(&negative[1..]);
            let offset = negative.last().unwrap().tcp[axis] - start.tcp[axis];
            append_cartesian(
                &mut program,
                &mut points,
                axis,
                offset,
                &crossing,
                model,
                start.tcp[axis],
            )?;
        }
        let outward = if negative.len() > 1 {
            &negative
        } else {
            &positive
        };
        let returning: Vec<_> = outward.iter().rev().copied().collect();
        append_cartesian(
            &mut program,
            &mut points,
            axis,
            0.0,
            &returning,
            model,
            start.tcp[axis],
        )?;
    }
    append_joint_return(model, &program, start, &mut points)?;
    append_joint_trials(model, &program, start, &mut points)?;
    program.preview_id = Some(program.content_id());
    Ok(CalibrationPreview {
        cartesian_duration_s: program.timed.iter().map(|leg| leg.duration()).sum(),
        program,
        points,
        ranges,
    })
}

fn trace(
    model: &Kinematics,
    tcp: Pose,
    origin_pose: Pose,
    start: Sample,
    axis: usize,
    target_offset: f64,
    bounds: ContinuationBounds,
) -> Vec<Sample> {
    let mut samples = vec![start];
    let length = model.reach_radius(&tcp);
    let nominal_step = length / 128.0;
    let resolution = length * 1e-8;
    // Monotone displacement bounded by model extent, with adaptive subdivision
    // at a continuation boundary. No operating speed or fixed travel budget.
    loop {
        let previous = *samples.last().unwrap();
        let distance = previous.tcp[axis] - origin_pose.translation[axis];
        let remaining = target_offset - distance;
        let tolerance = resolution.min(target_offset.abs() * 1e-6);
        if remaining.abs() <= tolerance {
            break;
        }
        let mut step = nominal_step.min(remaining.abs());
        let mut accepted = None;
        // Permit even very small requested amplitudes; resolution only governs
        // subdivision after a failed solve, not validation of the request.
        loop {
            let mut delta = [0.0; 6];
            delta[axis] = distance + remaining.signum() * step;
            let target = offset_pose(&origin_pose, delta, Frame::Base).unwrap();
            if let Ok(joints) =
                model.continue_pose(&previous.joints, &tcp, &target, JOINT_SAMPLE_STEP, bounds)
                && let Ok(pose) = model.fk(&joints, &tcp)
            {
                let error = pose_error(&origin_pose, &pose);
                let progress = remaining.signum() * (error[axis] - distance);
                let transverse = (0..3)
                    .filter(|component| *component != axis)
                    .map(|component| error[component].powi(2))
                    .sum::<f64>()
                    .sqrt();
                let rotation = error[3..].iter().map(|v| v * v).sum::<f64>().sqrt();
                if progress > 0.0
                    && progress >= resolution.min(remaining.abs()) * (1.0 - 1e-5)
                    && progress <= step + tolerance
                    && progress <= remaining.abs()
                    && transverse <= PATH_TOLERANCE
                    && rotation <= ORIENTATION_TOLERANCE
                {
                    accepted = Some(Sample {
                        joints,
                        tcp: pose.translation,
                    });
                    break;
                }
            }
            step *= 0.5;
            if step < resolution {
                break;
            }
        }
        match accepted {
            Some(sample) => samples.push(sample),
            None => break,
        }
    }
    samples
}

fn clip_trace(
    model: &Kinematics,
    tcp: Pose,
    origin: Pose,
    axis: usize,
    mut samples: Vec<Sample>,
    amplitude: Option<f64>,
    bounds: ContinuationBounds,
) -> Vec<Sample> {
    let Some(amplitude) = amplitude else {
        return samples;
    };
    let endpoint = samples.last().unwrap().tcp[axis] - origin.translation[axis];
    if endpoint.abs() <= amplitude {
        return samples;
    }
    let keep = samples
        .partition_point(|sample| (sample.tcp[axis] - origin.translation[axis]).abs() < amplitude);
    samples.truncate(keep.max(1));
    let finishing = trace(
        model,
        tcp,
        origin,
        *samples.last().unwrap(),
        axis,
        endpoint.signum() * amplitude,
        bounds,
    );
    samples.extend_from_slice(&finishing[1..]);
    samples
}

fn append_cartesian(
    program: &mut CartesianProgram,
    points: &mut Vec<PreviewPoint>,
    axis: usize,
    offset: f64,
    samples: &[Sample],
    model: &Kinematics,
    axis_origin: f64,
) -> Result<(), String> {
    let knots: Vec<_> = samples
        .iter()
        .map(|p| (p.tcp[axis] - axis_origin, p.joints))
        .collect();
    let leg = program.fit_cartesian(model, axis, &knots)?;
    let segment = program.segments.len();
    for (index, sample) in samples.iter().enumerate() {
        points.push(PreviewPoint {
            joints: sample.joints,
            tcp: sample.tcp,
            segment,
            t_s: Some(leg.knot_time(index)),
            space: "cartesian".into(),
            axis,
        });
    }
    program.segments.push(CartesianSegment { axis, offset });
    program.timed.push(Arc::new(leg));
    Ok(())
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
            t_s: None,
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
    let targets = program.joint_targets.unwrap();
    for axis in 0..7 {
        let mut previous = 0.0;
        for (leg, target) in [
            targets[axis][1] - origin.joints[axis],
            targets[axis][0] - origin.joints[axis],
            0.0,
        ]
        .into_iter()
        .enumerate()
        {
            let steps = ((target - previous).abs() / JOINT_SAMPLE_STEP)
                .ceil()
                .max(1.0) as usize;
            for step in 0..=steps {
                let offset = previous + (target - previous) * step as f64 / steps as f64;
                let mut joints = origin.joints;
                joints[axis] += offset;
                points.push(PreviewPoint {
                    joints,
                    tcp: model.fk(&joints, &program.tcp)?.translation,
                    t_s: None,
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
        let bounds = model.continuation_bounds(&program.origin).unwrap();
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
                    for axis in 0..7 {
                        assert!(point.joints[axis] >= bounds.lower[axis] - 1e-12);
                        assert!(point.joints[axis] <= bounds.upper[axis] + 1e-12);
                    }
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
        assert_eq!(actual.guide.len(), expected.guide.len());
        for (actual, expected) in actual.guide.iter().zip(&expected.guide) {
            assert_eq!(actual.axis, expected.axis);
            assert_close(&[actual.offset], &[expected.offset]);
            assert_close(&actual.joints, &expected.joints);
        }
    }

    #[test]
    fn auto_ranges_extend_beyond_100mm_and_manual_travel_clips_only_to_geometry() {
        let model = Kinematics::load("right").unwrap();
        let automatic =
            preview(&model, "right", origin(), tool(), PreviewRequest::default()).unwrap();
        check_samples(&model, &automatic);
        assert!(automatic.program.auto_range);
        assert_eq!(automatic.ranges.len(), 3);
        assert!(
            automatic
                .ranges
                .iter()
                .any(|range| range.positive > 0.1 || range.negative < -0.1)
        );
        let maximum = automatic
            .ranges
            .iter()
            .map(|range| range.positive.max(-range.negative))
            .fold(0.0, f64::max);
        assert_eq!(automatic.program.amplitude, maximum);
        for amplitude in [1e-6, 0.001, 0.15, f64::MAX] {
            let manual = preview(
                &model,
                "right",
                origin(),
                tool(),
                PreviewRequest {
                    amplitude: Some(amplitude),
                    ..PreviewRequest::default()
                },
            )
            .unwrap();
            check_samples(&model, &manual);
            assert!(!manual.program.auto_range);
            assert_eq!(manual.program.amplitude, amplitude);
            assert_eq!(manual.ranges, automatic.ranges);
            for segment in manual
                .program
                .segments
                .iter()
                .filter(|segment| segment.offset != 0.0)
            {
                let range = &manual.ranges[segment.axis];
                let available = if segment.offset > 0.0 {
                    range.positive
                } else {
                    -range.negative
                };
                assert!((segment.offset.abs() - amplitude.min(available)).abs() < END_TOLERANCE);
            }
            if amplitude <= 0.15 {
                assert_eq!(manual.program.segments.len(), 9);
            } else {
                assert_eq!(manual.points, automatic.points);
                assert_eq!(manual.program.segments, automatic.program.segments);
            }
        }
    }

    #[test]
    fn unrepresentable_joint_trial_is_reported_while_generating_the_preview() {
        let model = Kinematics::load("right").unwrap();
        let result = preview(
            &model,
            "right",
            origin(),
            tool(),
            PreviewRequest {
                joint_speeds: vec![1e-310],
                ..Default::default()
            },
        );
        assert!(result.unwrap_err().contains("关节速度"));
    }

    #[test]
    fn operating_speed_and_motion_preferences_do_not_change_reach_or_samples() {
        let model = Kinematics::load("right").unwrap();
        let baseline =
            preview(&model, "right", origin(), tool(), PreviewRequest::default()).unwrap();
        for speed in [1e-12, 0.001, 0.2, 1000.0, f64::MAX] {
            let motion = MotionTuning {
                joint_speed: speed,
                acceleration: speed,
                jerk: speed,
            };
            let result = preview(
                &model,
                "right",
                origin(),
                tool(),
                PreviewRequest {
                    speed,
                    motion,
                    ..PreviewRequest::default()
                },
            )
            .unwrap();
            assert_eq!(result.ranges, baseline.ranges);
            assert_eq!(result.program.segments, baseline.program.segments);
            assert_eq!(result.points.len(), baseline.points.len());
            for (a, b) in result.points.iter().zip(&baseline.points) {
                assert_eq!((a.joints, a.tcp), (b.joints, b.tcp));
            }
            assert!(result.cartesian_duration_s.is_finite() && result.cartesian_duration_s > 0.0);
            assert_eq!(result.program.speed, speed);
            assert_eq!(result.program.motion, motion);
        }
    }

    #[test]
    fn two_origins_and_tcps_use_actual_fk_and_fixed_orientation_continuation() {
        let origins = [
            origin(),
            [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133],
        ];
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            let mut ranges = Vec::new();
            for origin in origins {
                let flange = preview(
                    &model,
                    side,
                    origin,
                    Pose::identity(),
                    PreviewRequest::default(),
                )
                .unwrap();
                let tool =
                    preview(&model, side, origin, tool(), PreviewRequest::default()).unwrap();
                check_samples(&model, &flange);
                check_samples(&model, &tool);
                assert_ne!(flange.points[0].tcp, tool.points[0].tcp);
                // Holding orientation fixes the TCP-to-flange displacement in
                // base axes: straight translation travel is therefore invariant
                // under a rigid TCP offset, up to continuation resolution.
                for (a, b) in flange.ranges.iter().zip(&tool.ranges) {
                    assert!((a.positive - b.positive).abs() < END_TOLERANCE);
                    assert!((a.negative - b.negative).abs() < END_TOLERANCE);
                }
                ranges.push(flange.ranges);
            }
            assert!(
                ranges[0]
                    .iter()
                    .zip(&ranges[1])
                    .any(|(a, b)| (a.positive - b.positive).abs() > 0.01
                        || (a.negative - b.negative).abs() > 0.01)
            );
        }
    }

    #[test]
    fn captured_origin_ranges_respect_all_model_position_limits_and_are_directional() {
        let model = Kinematics::load("right").unwrap();
        // center_reference from calibration-4a3ca783-4d2d-49e2-867b-7e2b6d549d94.
        let origin = [
            1.0485126779344875,
            0.33696024505486105,
            -0.024104458715885565,
            0.5221739002599245,
            1.6839417312898655,
            0.07363282576166262,
            0.3497202195213932,
        ];
        let result = preview(
            &model,
            "right",
            origin,
            Pose::identity(),
            PreviewRequest::default(),
        )
        .unwrap();
        check_samples(&model, &result);
        let (lower, upper) = model.position_limits();
        for point in &result.points {
            assert!((0..7).all(
                |i| point.joints[i] >= lower[i] - 1e-12 && point.joints[i] <= upper[i] + 1e-12
            ));
        }
        assert_eq!(result.ranges.len(), 3);
        assert!(result.program.amplitude > 0.1);
        assert!(
            result
                .ranges
                .iter()
                .any(|range| (range.positive + range.negative).abs() > 0.01)
        );
        assert!(
            result
                .ranges
                .windows(2)
                .all(|pair| (pair[0].positive - pair[1].positive).abs() > 1e-4
                    || (pair[0].negative - pair[1].negative).abs() > 1e-4)
        );
        for axis in 0..3 {
            assert!(
                result
                    .program
                    .segments
                    .iter()
                    .any(|segment| segment.axis == axis && segment.offset != 0.0)
            );
        }
    }

    #[test]
    fn nominally_outside_origins_allow_return_toward_the_model_without_extending_outward() {
        let model = Kinematics::load("right").unwrap();
        let (lower, upper) = model.position_limits();
        let mut outside = origin();
        outside[5] = lower[5] - 0.2;
        outside[6] = upper[6] + 0.3;
        let result = preview(&model, "right", outside, tool(), PreviewRequest::default()).unwrap();
        check_samples(&model, &result);
        assert!(
            result
                .program
                .segments
                .iter()
                .any(|segment| segment.offset != 0.0)
        );
        for point in result
            .points
            .iter()
            .filter(|point| point.space == "cartesian")
        {
            assert!(point.joints[5] >= outside[5] - 1e-12);
            assert!(point.joints[6] <= outside[6] + 1e-12);
        }
    }

    #[test]
    fn default_request_and_program_envelope_round_trip() {
        let request: PreviewRequest = serde_json::from_str("{}").unwrap();
        assert_eq!(request, PreviewRequest::default());
        let partial: PreviewRequest = serde_json::from_str(r#"{"speed":0.005}"#).unwrap();
        assert_eq!(partial.amplitude, None);
        assert_eq!(partial.axes, vec![0, 1, 2]);
        let model = Kinematics::load("right").unwrap();
        let result = preview(&model, "right", origin(), tool(), request).unwrap();
        assert_eq!(result.program.joint_excursion, 16.0_f64.to_radians());
        let mut value = serde_json::to_value(&result.program).unwrap();
        value.as_object_mut().unwrap().remove("joint_excursion");
        value.as_object_mut().unwrap().remove("motion");
        value.as_object_mut().unwrap().remove("auto_range");
        let restored: CartesianProgram = serde_json::from_value(value).unwrap();
        assert_same_program(&restored, &result.program);
        assert_eq!(restored.motion, MotionTuning::default());
        assert!(!restored.auto_range);
        let encoded = serde_json::to_string(&result).unwrap();
        let decoded: CalibrationPreview = serde_json::from_str(&encoded).unwrap();
        assert_same_program(&decoded.program, &result.program);
        assert_eq!(decoded.points.len(), result.points.len());
        assert_eq!(decoded.program.motion, result.program.motion);
        assert_eq!(decoded.program.auto_range, result.program.auto_range);
        let mut legacy = serde_json::to_value(&result).unwrap();
        legacy.as_object_mut().unwrap().remove("ranges");
        let legacy: CalibrationPreview = serde_json::from_value(legacy).unwrap();
        assert!(legacy.ranges.is_empty());
        for (actual, expected) in decoded.points.iter().zip(&result.points) {
            assert_eq!(actual.space, expected.space);
            assert_eq!(actual.axis, expected.axis);
            assert_eq!(actual.segment, expected.segment);
            assert_close(&actual.joints, &expected.joints);
            assert_close(&actual.tcp, &expected.tcp);
        }
    }

    #[test]
    fn branch_guides_fit_the_command_body_and_interpolate_the_preview_knots() {
        let model = Kinematics::load("right").unwrap();
        let captured = [
            1.0485035052359992,
            0.3369648851818667,
            -0.024104830439799833,
            0.5221767742481414,
            1.683944156955803,
            0.07362801043445026,
            0.34971860713891384,
        ];
        for (origin, tcp) in [(captured, Pose::identity()), (origin(), tool())] {
            let started = std::time::Instant::now();
            let result = preview(&model, "right", origin, tcp, PreviewRequest::default()).unwrap();
            let elapsed = started.elapsed();
            let original = model.fk(&origin, &tcp).unwrap();
            let mut expected: Vec<_> = result
                .points
                .iter()
                .filter(|point| point.space == "cartesian")
                .map(|point| BranchPoint {
                    axis: point.axis,
                    offset: point.tcp[point.axis] - original.translation[point.axis],
                    joints: point.joints,
                })
                .collect();
            expected.sort_by(|a, b| a.axis.cmp(&b.axis).then(a.offset.total_cmp(&b.offset)));
            expected.dedup_by(|a, b| a.axis == b.axis && a.offset == b.offset);
            assert_eq!(result.program.guide, expected);
            assert!(result.program.guide.len() > 51);
            for knot in &result.program.guide {
                let guide = result.program.guide_at(knot.axis, knot.offset).unwrap();
                assert_eq!(guide.axis, knot.axis);
                assert_close(&guide.position, &knot.joints);
                assert!(guide.tangent.iter().all(|value| value.is_finite()));
            }
            let request = serde_json::json!({
                "request_id": "12345678-1234-1234-1234-123456789012",
                "client_id": "12345678-1234-1234-1234-123456789012",
                "lease_id": "12345678-1234-1234-1234-123456789012",
                "generation": 123456789, "kind": "calibrate_start",
                "settings": { "axis": 1, "excursion": 0.2, "speed": 0.02,
                    "max_friction": 0.1, "cartesian": result.program }
            });
            let size = serde_json::to_vec(&request).unwrap().len();
            assert!(size < 2 * 1024 * 1024, "command body contains {size} bytes");
            eprintln!(
                "preview timing tcp={:?}: {elapsed:?}, guide knots={}, request bytes={size}",
                tcp.translation,
                request["settings"]["cartesian"]["guide"]
                    .as_array()
                    .unwrap()
                    .len()
            );
        }
    }

    #[test]
    #[ignore = "reads the local calibration report in R17A_CALIBRATION_REPORT"]
    fn replay_saved_program_at_recorded_send_times() {
        let data = std::fs::read(std::env::var("R17A_CALIBRATION_REPORT").unwrap()).unwrap();
        let report: crate::calibration::Report = serde_json::from_slice(&data).unwrap();
        let model = Kinematics::load(&report.side).unwrap();
        let mut program = (*report.settings.cartesian.unwrap()).clone();
        let bounds = model.continuation_bounds(&program.origin).unwrap();
        program.prepare(&model).unwrap();
        let times: Vec<_> = program
            .timed
            .iter()
            .scan(0.0, |elapsed, leg| {
                *elapsed += leg.duration();
                Some(*elapsed)
            })
            .collect();
        let mut samples = 0;
        for p in report.curves.iter() {
            let Some(ms) = p.command_at_ms else { continue };
            let elapsed = ms as f64 / 1000.0;
            let index = times
                .partition_point(|end| *end <= elapsed)
                .min(times.len() - 1);
            let start = if index == 0 { 0.0 } else { times[index - 1] };
            let sample = program.timed[index].sample(elapsed - start);
            for axis in 0..7 {
                assert!((bounds.lower[axis]..=bounds.upper[axis]).contains(&sample.q[axis]));
                assert!(
                    sample.dq[axis].abs()
                        <= program.motion.speed_limits(model.speed_limits())[axis]
                );
                assert!(sample.ddq[axis].abs() <= program.motion.acceleration);
            }
            samples += 1;
        }
        assert!(samples > 0);
        eprintln!(
            "{} recorded send times; {} legs; first leg {:.3}s; XYZ loop {:.3}s",
            samples,
            program.timed.len(),
            program.timed[0].duration(),
            program.timed.iter().map(|l| l.duration()).sum::<f64>()
        );
    }

    #[test]
    fn stored_fifty_one_knot_programs_still_deserialize_and_interpolate() {
        let mut program = CartesianProgram {
            preview_id: None,
            timed: vec![],
            joint_legs: Default::default(),
            joint_speeds: vec![],
            joint_targets: None,
            side: "right".into(),
            origin: origin(),
            tcp: tool(),
            amplitude: 0.008,
            speed: 0.01,
            joint_excursion: default_joint_excursion(),
            motion: MotionTuning::default(),
            auto_range: false,
            segments: vec![],
            guide: vec![],
        };
        for axis in 0..3 {
            for point in -8..=8 {
                let offset = point as f64 * 0.001;
                program.guide.push(BranchPoint {
                    axis,
                    offset,
                    joints: std::array::from_fn(|i| {
                        program.origin[i] + offset * (i + axis + 1) as f64
                    }),
                });
            }
        }
        let restored: CartesianProgram =
            serde_json::from_str(&serde_json::to_string(&program).unwrap()).unwrap();
        assert_eq!(restored.guide.len(), 51);
        for axis in 0..3 {
            for offset in [-0.008, -0.0035, 0.0, 0.0025, 0.008] {
                let guide = restored.guide_at(axis, offset).unwrap();
                let expected: [f64; 7] =
                    std::array::from_fn(|i| program.origin[i] + offset * (i + axis + 1) as f64);
                assert_close(&guide.position, &expected);
                for i in 0..7 {
                    assert!((guide.tangent[i] - (i + axis + 1) as f64).abs() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn reversible_continuation_returns_to_the_exact_starting_configuration() {
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
        assert_eq!(last_cartesian.joints, origin());
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
                let result = preview(
                    &model,
                    side,
                    origin(),
                    tcp,
                    PreviewRequest {
                        amplitude: Some(0.05),
                        ..PreviewRequest::default()
                    },
                )
                .unwrap();
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
                    amplitude: Some(0.10),
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
                amplitude: Some(0.02),
                speed: 0.0135,
                axes: vec![2, 0, 2],
                ..PreviewRequest::default()
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
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -0.001] {
            assert!(
                preview(
                    &model,
                    "right",
                    origin(),
                    tool(),
                    PreviewRequest {
                        amplitude: Some(value),
                        ..PreviewRequest::default()
                    }
                )
                .is_err()
            );
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -0.001] {
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
