//! Offline command replay. CI contains only synthetic poses and an anonymous clock.
//! The ignored capture test samples commands; it does not replay physical motors.
use crate::{
    kinematics::{
        CartesianPath, Frame, Kinematics, LocalTarget, MotionLimits, Pose, SmoothingState,
        StepInput, pose_error,
    },
    protocol::test_wire_targets,
    reference::{CartesianGuard, Previous, Reference},
    trace::{TraceData, TraceRecording},
    trajectory::{JOINT_ACCELERATION, JOINT_JERK, JOINT_SPEED},
};
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};

const LINEAR_CAP: f64 = 0.015;
const ANGULAR_CAP: f64 = 0.15;
const LOCAL_TICKS: usize = 180;
const BRAKE_TICK: usize = 90;
const MAX_PAIR_US: u64 = 15_000;

#[derive(Deserialize)]
struct Clock {
    control_us: Vec<u64>,
    sends: Vec<Send>,
}

#[derive(Deserialize)]
struct Send {
    at_us: u64,
    source: usize,
}

fn clock() -> Clock {
    serde_json::from_str(include_str!("../tests/fixtures/send-clock.json")).unwrap()
}

fn at(origin: Instant, us: u64) -> Instant {
    origin + Duration::from_micros(us)
}

fn norm(values: &[f64]) -> f64 {
    values.iter().map(|v| v * v).sum::<f64>().sqrt()
}

// Public SDK position/velocity grid, converted through the public calibration.
// Float32 narrowing is additional to integer quantization. All CI positions
// remain within two radians, so this roundoff allowance is conservative.
fn wire_error_bounds(side: &str, joint: usize) -> (f64, f64) {
    let calibration = match side {
        "right" => include_str!("../../resources/calibration/joint_calib_right.txt"),
        "left" => include_str!("../../resources/calibration/joint_calib_left.txt"),
        _ => unreachable!(),
    };
    let gain = calibration
        .lines()
        .find_map(|line| {
            let fields: Vec<_> = line.split('#').next().unwrap().split_whitespace().collect();
            if fields.len() == 4 && fields[0].parse::<usize>().ok() == Some(joint) {
                Some((fields[1].parse::<f64>().unwrap() * fields[2].parse::<f64>().unwrap()).abs())
            } else {
                None
            }
        })
        .unwrap();
    let reduction = match joint {
        0..=1 => 32.0,
        2..=4 => 38.0 / 3.0,
        _ => 16.0,
    };
    let position = gain * std::f64::consts::TAU / (32768.0 * reduction);
    let velocity = gain * std::f64::consts::TAU / (64.0 * reduction);
    let roundoff = 16.0 * f32::EPSILON as f64;
    (position * 0.5 + roundoff, velocity * 0.5 + roundoff)
}

#[test]
fn constant_joint_jogs_match_send_time_on_every_axis_and_wire_grid() {
    let clock = clock();
    let origin = Instant::now();
    for side in ["right", "left"] {
        for axis in 0..7 {
            for speed in [0.001, 0.05, JOINT_SPEED] {
                for direction in [-1.0, 1.0] {
                    let speed = speed * direction;
                    let (position_error, velocity_error) = wire_error_bounds(side, axis);
                    let mut previous: Option<(u64, crate::reference::Sample, [f64; 7])> = None;
                    let (mut count, mut error2, mut bound2) = (0, 0.0, 0.0);
                    for send in clock.sends.iter().filter(|s| s.source < LOCAL_TICKS) {
                        let source_us = clock.control_us[send.source];
                        let mut q = [0.0; 7];
                        let mut dq = [0.0; 7];
                        q[axis] = speed * source_us as f64 * 1e-6;
                        dq[axis] = speed;
                        let source = Reference::joint(at(origin, source_us), q, dq, [0.0; 7], 1, 0);
                        let prior = previous.as_ref().map(|(us, s, _)| Previous {
                            at: at(origin, *us),
                            q: s.q,
                            dq: s.dq,
                        });
                        let sample = source.sample(at(origin, send.at_us), prior).unwrap();
                        let (wire_q, wire_dq) = test_wire_targets(side, sample.q, sample.dq);
                        assert!((wire_q[axis] - sample.q[axis]).abs() <= position_error);
                        assert!((wire_dq[axis] - speed).abs() <= velocity_error);
                        for joint in 0..7 {
                            if joint != axis {
                                assert_eq!(sample.q[joint], 0.0);
                                assert_eq!(sample.dq[joint], 0.0);
                            }
                        }
                        if let Some((previous_us, previous_sample, previous_q)) = previous {
                            let elapsed_us = send.at_us - previous_us;
                            if elapsed_us <= MAX_PAIR_US {
                                let dt = elapsed_us as f64 * 1e-6;
                                let raw_error =
                                    ((sample.q[axis] - previous_sample.q[axis]) / dt - speed).abs();
                                assert!(
                                    raw_error / speed.abs() < 1e-8,
                                    "{side} axis={axis} speed={speed}: raw relative error"
                                );
                                let error = (wire_q[axis] - previous_q[axis]) / dt - wire_dq[axis];
                                let bound = 2.0 * position_error / dt + velocity_error;
                                assert!(
                                    error.abs() <= bound,
                                    "{side} axis={axis} speed={speed}: wire error exceeds SDK grid bound"
                                );
                                error2 += (error / speed.abs()).powi(2);
                                bound2 += (bound / speed.abs()).powi(2);
                                count += 1;
                            }
                        }
                        previous = Some((send.at_us, sample, wire_q));
                    }
                    assert!(count > 140);
                    // At 0.001 rad/s the velocity grid can exceed the requested
                    // speed. A universal percentage tolerance would be invalid.
                    assert!((error2 / count as f64).sqrt() <= (bound2 / count as f64).sqrt());
                }
            }
        }
    }
}

fn limits() -> MotionLimits {
    MotionLimits {
        joint_velocity: [JOINT_SPEED; 7],
        joint_acceleration: [JOINT_ACCELERATION; 7],
        max_reference_error: 0.60,
        max_joint_step: 0.01,
        linear_speed: LINEAR_CAP,
        angular_speed: ANGULAR_CAP,
    }
}

fn guard(
    model: &Arc<Kinematics>,
    tcp: Pose,
    path: CartesianPath,
    measured: [f64; 7],
) -> CartesianGuard {
    CartesianGuard {
        model: model.clone(),
        tcp,
        path,
        goal: None,
        linear_speed: LINEAR_CAP,
        angular_speed: ANGULAR_CAP,
        measured,
        max_reference_error: 0.60,
    }
}

#[test]
fn cartesian_send_samples_preserve_all_axes_frames_and_offset_tcp_through_braking() {
    let clock = clock();
    let origin = Instant::now();
    let tcp = Pose {
        translation: [0.0, 0.0, 0.1],
        ..Pose::identity()
    };
    for side in ["left", "right"] {
        let model = Arc::new(Kinematics::load(side).unwrap());
        for frame in [Frame::Base, Frame::Tool] {
            for axis in 0..6 {
                for (linear_speed, angular_speed) in
                    [(0.0005, 0.005), (0.014, 0.14), (LINEAR_CAP, ANGULAR_CAP)]
                {
                    for direction in [-1.0, 1.0] {
                        // Synthetic configuration, independent of any private capture.
                        let mut q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
                        let (mut dq, mut ddq) = ([0.0; 7], [0.0; 7]);
                        let path = CartesianPath {
                            origin: model.fk(&q, &tcp).unwrap(),
                            axis,
                            frame,
                        };
                        let mut references = Vec::with_capacity(LOCAL_TICKS);
                        let mut prior_us = 0;
                        for (tick, &us) in clock.control_us.iter().take(LOCAL_TICKS).enumerate() {
                            let dt = (us - prior_us) as f64 * 1e-6;
                            let stopping = tick >= BRAKE_TICK;
                            let mut twist = [0.0; 6];
                            if !stopping {
                                twist[axis] = direction
                                    * if axis < 3 {
                                        linear_speed
                                    } else {
                                        angular_speed
                                    };
                            }
                            let input = StepInput {
                                reference: q,
                                measured: q,
                                previous_velocity: dq,
                                target: LocalTarget::Velocity { twist, frame },
                                dt,
                                tcp,
                                limits: limits(),
                                smoothing: Some(SmoothingState {
                                    acceleration: ddq,
                                    jerk: [JOINT_JERK; 7],
                                    path: Some(path),
                                    stopping,
                                }),
                            };
                            let candidate = model.step(input).unwrap_or_else(|error| {
                                panic!(
                                    "{side} source {frame:?} axis={axis} direction={direction} \
                                    speeds=({linear_speed},{angular_speed}) tick={tick}: {error}"
                                )
                            });
                            for (joint, previous_acceleration) in ddq.iter().enumerate() {
                                assert!(candidate.dq[joint].abs() <= JOINT_SPEED + 1e-10);
                                assert!(candidate.ddq[joint].abs() <= JOINT_ACCELERATION + 1e-9);
                                assert!(
                                    (candidate.ddq[joint] - previous_acceleration).abs()
                                        <= JOINT_JERK * dt + 1e-8
                                );
                            }
                            (q, dq, ddq) = (candidate.q, candidate.dq, candidate.ddq);
                            let mut reference = Reference::joint(at(origin, us), q, dq, ddq, 1, 0);
                            reference.stopping = stopping;
                            reference.cartesian = Some(guard(&model, tcp, path, q));
                            references.push(reference);
                            prior_us = us;
                        }
                        let mut previous: Option<(u64, crate::reference::Sample, Pose, Pose)> =
                            None;
                        let (mut start_count, mut brake_count) = (0, 0);
                        for send in clock.sends.iter().filter(|s| s.source < LOCAL_TICKS) {
                            let source = &references[send.source];
                            let prior = previous.as_ref().map(|(us, s, _, _)| Previous {
                                at: at(origin, *us),
                                q: s.q,
                                dq: s.dq,
                            });
                            let sample = source.sample(at(origin, send.at_us), prior)
                                .unwrap_or_else(|status| {
                                    panic!("{side} sample {frame:?} axis={axis} direction={direction} \
                                        speeds=({linear_speed},{angular_speed}) source={} stopping={}: {status:?}",
                                        send.source, source.stopping)
                                });
                            let pose = model.fk(&sample.q, &tcp).unwrap();
                            path.check(&pose).unwrap_or_else(|error| {
                                panic!("{side} sample left the immutable Cartesian path: {error}")
                            });
                            let (wire_q, wire_dq) = test_wire_targets(side, sample.q, sample.dq);
                            let wire_pose = model.fk(&wire_q, &tcp).unwrap();
                            path.check(&wire_pose).unwrap_or_else(|error| {
                                panic!(
                                    "{side} wire target left the immutable Cartesian path: {error}"
                                )
                            });
                            for joint in 0..7 {
                                let (position_error, velocity_error) =
                                    wire_error_bounds(side, joint);
                                assert!((wire_q[joint] - sample.q[joint]).abs() <= position_error);
                                assert!(
                                    (wire_dq[joint] - sample.dq[joint]).abs() <= velocity_error
                                );
                                assert!(sample.dq[joint].abs() <= JOINT_SPEED + 1e-10);
                                assert!(wire_dq[joint].abs() <= JOINT_SPEED + velocity_error);
                            }
                            if let Some((previous_us, _, previous_pose, previous_wire_pose)) =
                                previous
                            {
                                let dt = (send.at_us - previous_us) as f64 * 1e-6;
                                let delta = pose_error(&previous_pose, &pose);
                                assert!(
                                    norm(&delta[..3]) <= LINEAR_CAP * dt + 1e-10,
                                    "{side} sample linear TCP cap"
                                );
                                assert!(
                                    norm(&delta[3..]) <= ANGULAR_CAP * dt + 1e-10,
                                    "{side} sample angular TCP cap"
                                );
                                let wire_delta = pose_error(&previous_wire_pose, &wire_pose);
                                let before_rounding =
                                    pose_error(&previous_pose, &previous_wire_pose);
                                let after_rounding = pose_error(&pose, &wire_pose);
                                // Only the known endpoint rounding can extend a wire
                                // increment beyond the strict unquantized TCP cap.
                                assert!(
                                    norm(&wire_delta[..3])
                                        <= LINEAR_CAP * dt
                                            + norm(&before_rounding[..3])
                                            + norm(&after_rounding[..3])
                                            + 1e-10
                                );
                                assert!(
                                    norm(&wire_delta[3..])
                                        <= ANGULAR_CAP * dt
                                            + norm(&before_rounding[3..])
                                            + norm(&after_rounding[3..])
                                            + 1e-10
                                );
                            }
                            if source.stopping {
                                brake_count += 1;
                            } else {
                                start_count += 1;
                            }
                            previous = Some((send.at_us, sample, pose, wire_pose));
                        }
                        assert!(start_count > 60 && brake_count > 60);
                        let end = references.last().unwrap();
                        assert!(
                            norm(&pose_error(
                                &path.origin,
                                &model.fk(&references[BRAKE_TICK - 1].q, &tcp).unwrap()
                            )) > 1e-5,
                            "{side} synthetic motion must make progress"
                        );
                        assert!(
                            norm(&end.dq) < 1e-5,
                            "{side} synthetic brake must reach rest"
                        );
                    }
                }
            }
        }
    }
}

#[derive(Clone)]
struct CapturedSource {
    at_us: u64,
    generation: u64,
    enabled: bool,
    stopping: bool,
    q: [f64; 7],
    dq: [f64; 7],
    ddq: [f64; 7],
    measured: [f64; 7],
    path: Option<CartesianPath>,
}

#[derive(Default)]
struct Consistency {
    count: usize,
    error2: f64,
    speed2: f64,
}

impl Consistency {
    fn add(&mut self, position_rate: f64, velocity: f64) {
        self.count += 1;
        self.error2 += (position_rate - velocity).powi(2);
        self.speed2 += velocity.powi(2);
    }
    fn relative_rms(&self) -> Option<f64> {
        (self.count > 0 && self.speed2 > 1e-9).then(|| (self.error2 / self.speed2).sqrt())
    }
}

#[test]
#[ignore = "private local capture; set UNITREE_REPLAY_CAPTURE, no hardware or server"]
fn local_capture_replay() {
    // Never embed a private filename, identifier, port, pose, or feedback in CI.
    let path = std::env::var_os("UNITREE_REPLAY_CAPTURE").expect("set UNITREE_REPLAY_CAPTURE");
    let bytes = std::fs::read(path).expect("could not read local capture");
    // Production deserialization accepts old captures without new timing fields.
    let recording: TraceRecording = serde_json::from_slice(&bytes).expect("invalid capture schema");
    let model = Arc::new(Kinematics::load(&recording.trace.side).expect("capture side"));
    // Confirmed by the capture owner; this old schema did not record these.
    let tcp = Pose::identity();
    let frame = Frame::Base;
    let origin = Instant::now();
    let mut controls: Vec<_> = recording
        .samples
        .iter()
        .filter(|sample| matches!(sample.data, TraceData::Control { .. }))
        .collect();
    controls.sort_by_key(|sample| sample.at_us);
    let mut sources = BTreeMap::new();
    let mut active_path = None;
    let mut active_key = None;
    let mut approximate_origins = 0;
    for sample in controls {
        let TraceData::Control {
            seq,
            generation,
            enabled,
            stopping,
            motion,
            axis,
            q_ref,
            dq_ref,
            ddq_ref,
            q: measured,
            ..
        } = sample.data
        else {
            unreachable!()
        };
        let measured = measured.map(|value| {
            value
                .filter(|q| q.is_finite())
                .expect("capture control requires seven finite measured joint positions")
        });
        if stopping {
            // Stop changes generation/motion but inherits the old selected path.
        } else if matches!(motion, 3 | 4) {
            let key = (generation, motion, axis);
            if active_key != Some(key) {
                let axis = axis.expect("Cartesian capture lacks axis") as usize;
                assert!(axis < 6, "Cartesian capture axis outside range");
                // begin() anchors from measured joints between control samples.
                // That exact anchor is absent here. First recorded source FK is
                // an approximation, never an invented exact original origin.
                active_path = Some(CartesianPath {
                    origin: model.fk(&q_ref, &tcp).unwrap(),
                    axis,
                    frame,
                });
                active_key = Some(key);
                approximate_origins += 1;
            }
        } else {
            active_key = None;
            active_path = None;
        }
        sources.insert(
            seq,
            CapturedSource {
                at_us: sample.at_us,
                generation,
                enabled,
                stopping,
                q: q_ref,
                dq: dq_ref,
                ddq: ddq_ref,
                measured,
                path: active_path,
            },
        );
    }
    let mut sends: Vec<_> = recording
        .samples
        .iter()
        .filter(|sample| matches!(sample.data, TraceData::Transmit { .. }))
        .collect();
    sends.sort_by_key(|sample| sample.at_us);
    let transmit_gap_pairs = sends
        .windows(2)
        .filter(|pair| pair[1].at_us - pair[0].at_us > MAX_PAIR_US)
        .count();
    let mut previous: Option<Previous> = None;
    let mut previous_new: Option<(u64, [f64; 7], [f64; 7])> = None;
    let mut previous_old: Option<(u64, [f64; 7], [f64; 7])> = None;
    let (mut old, mut new) = (Consistency::default(), Consistency::default());
    let (mut matched, mut missing_source, mut rejected, mut skipped_gaps) = (0, 0, 0, 0);
    let (mut old_path_flags, mut new_path_flags, mut source_path_flags) = (0, 0, 0);
    let (mut raw_linear_cap_flags, mut raw_angular_cap_flags) = (0, 0);
    let mut statuses = BTreeMap::<String, usize>::new();
    for tx in sends {
        let TraceData::Transmit {
            seq,
            mode,
            q_wire,
            dq_wire,
            ..
        } = tx.data
        else {
            unreachable!()
        };
        if !mode.iter().all(|mode| *mode == 1) {
            previous = None;
            previous_new = None;
            previous_old = None;
            continue;
        }
        let Some(source) = sources.get(&seq) else {
            missing_source += 1;
            previous = None;
            previous_new = None;
            previous_old = None;
            continue;
        };
        if !source.enabled {
            continue;
        }
        matched += 1;
        if let Some(path) = source.path {
            source_path_flags +=
                usize::from(path.check(&model.fk(&source.q, &tcp).unwrap()).is_err());
            old_path_flags += usize::from(path.check(&model.fk(&q_wire, &tcp).unwrap()).is_err());
        }
        let mut reference = Reference::joint(
            at(origin, source.at_us),
            source.q,
            source.dq,
            source.ddq,
            source.generation,
            0,
        );
        reference.stopping = source.stopping;
        reference.cartesian = source
            .path
            .map(|path| guard(&model, tcp, path, source.measured));
        let sampled = reference.sample(at(origin, tx.at_us), previous);
        if let Some((last_us, last_q, last_dq)) = previous_old
            && tx.at_us >= 2_500_000
            && tx.at_us <= 6_500_000
            && last_us >= 2_500_000
        {
            let elapsed = tx.at_us - last_us;
            if elapsed <= MAX_PAIR_US {
                old.add(
                    (q_wire[3] - last_q[3]) / (elapsed as f64 * 1e-6),
                    (dq_wire[3] + last_dq[3]) * 0.5,
                );
            } else {
                skipped_gaps += 1;
            }
        }
        previous_old = Some((tx.at_us, q_wire, dq_wire));
        let sample = match sampled {
            Ok(sample) => sample,
            Err(status) => {
                rejected += 1;
                *statuses.entry(format!("{status:?}")).or_default() += 1;
                previous_new = None;
                // Keep the last actually sampled command as the production
                // sender would; do not fabricate a replacement on rejection.
                continue;
            }
        };
        let (new_q, new_dq) = test_wire_targets(&recording.trace.side, sample.q, sample.dq);
        if let Some(path) = source.path {
            new_path_flags += usize::from(path.check(&model.fk(&new_q, &tcp).unwrap()).is_err());
            if let Some(prior) = previous {
                let dt = at(origin, tx.at_us).duration_since(prior.at).as_secs_f64();
                if dt <= MAX_PAIR_US as f64 * 1e-6 {
                    let delta = pose_error(
                        &model.fk(&prior.q, &tcp).unwrap(),
                        &model.fk(&sample.q, &tcp).unwrap(),
                    );
                    raw_linear_cap_flags +=
                        usize::from(norm(&delta[..3]) > LINEAR_CAP * dt + 1e-10);
                    raw_angular_cap_flags +=
                        usize::from(norm(&delta[3..]) > ANGULAR_CAP * dt + 1e-10);
                }
            }
        }
        if let Some((last_us, last_q, last_dq)) = previous_new
            && tx.at_us >= 2_500_000
            && tx.at_us <= 6_500_000
            && last_us >= 2_500_000
        {
            let elapsed = tx.at_us - last_us;
            if elapsed <= MAX_PAIR_US {
                new.add(
                    (new_q[3] - last_q[3]) / (elapsed as f64 * 1e-6),
                    (new_dq[3] + last_dq[3]) * 0.5,
                );
            }
        }
        previous = Some(Previous {
            at: at(origin, tx.at_us),
            q: sample.q,
            dq: sample.dq,
        });
        previous_new = Some((tx.at_us, new_q, new_dq));
    }
    let old_relative = old.relative_rms();
    let new_relative = new.relative_rms();
    println!(
        "{}",
        serde_json::json!({
            "kind": "offline_command_sampling", "physical_motor_replay": false,
            "matched_transmits": matched, "missing_sources": missing_source,
            "capture_dropped_samples": recording.trace.dropped, "transmit_gap_pairs_over_15ms": transmit_gap_pairs, "steady_gap_pairs_skipped": skipped_gaps,
            "sampler_rejections": rejected, "rejection_statuses": statuses,
            "steady_j4": { "old_pairs": old.count, "new_pairs": new.count, "old_relative_rms": old_relative, "new_relative_rms": new_relative },
            "path": { "origin_reconstruction": "approximate_first_control_fk", "approximate_origins": approximate_origins, "frame_and_tcp": "owner_confirmed_base_identity", "source_flags": source_path_flags, "old_wire_flags": old_path_flags, "new_wire_flags": new_path_flags },
            "raw_tcp_cap_flags": { "linear": raw_linear_cap_flags, "angular": raw_angular_cap_flags }
        })
    );
    assert!(matched > 900, "capture has too few matched transmits");
    assert!(
        old.count > 300 && new.count > 300,
        "steady capture window lacks enough command pairs"
    );
    assert!(
        new_relative.is_some_and(|rms| rms < 0.10),
        "new J4 consistency exceeds 10% or lacks meaningful speed"
    );
    assert!(
        old_relative.is_some_and(|rms| rms > 0.20),
        "capture does not reproduce old J4 inconsistency"
    );
    assert_eq!(
        rejected, 0,
        "production sampler rejected archived sources; inspect nonidentifying JSON summary"
    );
    assert_eq!(
        new_path_flags, 0,
        "sampled wire command violated reconstructed path"
    );
    assert_eq!((raw_linear_cap_flags, raw_angular_cap_flags), (0, 0));
}
