//! Time sampling between the controller and serial writer. No I/O or logging.
#[cfg(test)]
use crate::trajectory::JOINT_ACCELERATION;
use crate::{
    calibration_path::PlaybackPhase,
    kinematics::{CartesianPath, Kinematics, Pose, pose_error},
    teaching::Program as TeachingProgram,
    trajectory::{JOINT_SPEED, MotionTuning},
    tuning::ServoTuning,
};
use serde::{Deserialize, Serialize};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

pub const MAX_REFERENCE_AGE: Duration = Duration::from_millis(50);

#[derive(Clone, Debug)]
pub struct Reference {
    pub at: Instant,
    pub q: [f64; 7],
    pub dq: [f64; 7],
    pub ddq: [f64; 7],
    pub generation: u64,
    pub epoch: u64,
    pub valid_until: Option<Instant>,
    pub stopping: bool,
    pub joint_velocity: [f64; 7],
    pub joint_goal: Option<[f64; 7]>,
    pub cartesian: Option<CartesianGuard>,
    pub servo_tuning: ServoTuning,
    pub tuning_revision: u64,
    pub sampling: Sampling,
}

#[derive(Clone, Debug, Default)]
pub enum Sampling {
    #[default]
    Integrated,
    Absolute,
    Calibration {
        program: Arc<crate::calibration_path::Playback>,
        phase: PlaybackPhase,
        limits: MotionTuning,
    },
    Playback {
        program: Arc<TeachingProgram>,
        started: Instant,
    },
}

#[derive(Clone, Debug)]
pub struct CartesianGuard {
    pub model: Arc<Kinematics>,
    pub tcp: Pose,
    pub path: CartesianPath,
    pub goal: Option<Pose>,
}

#[derive(Clone, Copy, Debug)]
pub struct Previous {
    pub at: Instant,
    pub q: [f64; 7],
    pub dq: [f64; 7],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SampleStatus {
    Aligned,
    Bounded,
    Stale,
    Future,
    Invalid,
    Constraint,
    Expired,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Timing {
    pub status: SampleStatus,
    pub age_us: Option<u64>,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub q: [f64; 7],
    pub dq: [f64; 7],
    pub timing: Timing,
}

impl Reference {
    pub fn joint(
        at: Instant,
        q: [f64; 7],
        dq: [f64; 7],
        ddq: [f64; 7],
        generation: u64,
        epoch: u64,
    ) -> Self {
        Self {
            at,
            q,
            dq,
            ddq,
            generation,
            epoch,
            valid_until: None,
            stopping: false,
            joint_velocity: [JOINT_SPEED; 7],
            joint_goal: None,
            cartesian: None,
            servo_tuning: ServoTuning::default(),
            tuning_revision: 0,
            sampling: Sampling::Integrated,
        }
    }

    pub fn timing(&self, at: Instant, status: SampleStatus) -> Timing {
        Timing {
            status,
            age_us: at.checked_duration_since(self.at).map(micros),
        }
    }

    #[cfg(test)]
    pub fn sample(&self, at: Instant, previous: Option<Previous>) -> Result<Sample, SampleStatus> {
        self.sample_with_phase(at, previous, None)
    }

    pub fn sample_with_phase(
        &self,
        at: Instant,
        previous: Option<Previous>,
        send_phase: Option<PlaybackPhase>,
    ) -> Result<Sample, SampleStatus> {
        if self.valid_until.is_some_and(|until| at >= until) {
            return Err(SampleStatus::Expired);
        }
        let age = at
            .checked_duration_since(self.at)
            .ok_or(SampleStatus::Future)?;
        if age > MAX_REFERENCE_AGE {
            return Err(SampleStatus::Stale);
        }
        if previous.is_some_and(|p| !p.q.iter().chain(p.dq.iter()).all(|q| q.is_finite()))
            || self
                .joint_goal
                .is_some_and(|goal| !goal.iter().all(|q| q.is_finite()))
        {
            return Err(SampleStatus::Invalid);
        }
        for i in 0..7 {
            if !self.q[i].is_finite()
                || !self.dq[i].is_finite()
                || !self.ddq[i].is_finite()
                || !self.joint_velocity[i].is_finite()
                || self.joint_velocity[i] <= 0.0
            {
                return Err(SampleStatus::Invalid);
            }
        }
        match &self.sampling {
            Sampling::Absolute => {
                return Ok(Sample {
                    q: self.q,
                    dq: self.dq,
                    timing: self.timing(at, SampleStatus::Aligned),
                });
            }
            Sampling::Calibration { program, phase, .. } => {
                let phase = send_phase.unwrap_or(*phase);
                if matches!(phase, PlaybackPhase::Held { .. }) {
                    return Err(SampleStatus::Stale);
                }
                let sample = phase.sample(program, at);
                return Ok(Sample {
                    q: sample.q,
                    dq: sample.dq,
                    timing: self.timing(at, SampleStatus::Aligned),
                });
            }
            Sampling::Playback { program, started } => {
                let sample = program.sample(at.saturating_duration_since(*started).as_secs_f64());
                return Ok(Sample {
                    q: sample.q,
                    dq: sample.dq,
                    timing: self.timing(at, SampleStatus::Aligned),
                });
            }
            Sampling::Integrated => {}
        }
        let mut q = self.q;
        let mut dq = self.dq;
        let mut bounded = false;
        let dt = age.as_secs_f64();
        for i in 0..7 {
            (q[i], dq[i]) = project_axis(
                self.q[i],
                self.dq[i],
                self.ddq[i],
                dt,
                self.stopping,
                self.joint_velocity[i],
            );
        }
        let anchor = previous.map_or(self.q, |p| p.q);
        if let Some(previous) = previous {
            let elapsed = at
                .checked_duration_since(previous.at)
                .filter(|dt| !dt.is_zero())
                .ok_or(SampleStatus::Future)?
                .as_secs_f64();
            for i in 0..7 {
                // Keep position continuous when a new discrete control sample
                // replaces its predecessor. Re-anchoring each send to source q
                // can move backwards during braking even with positive dq.
                let integrated = previous.q[i] + 0.5 * (previous.dq[i] + dq[i]) * elapsed;
                q[i] = integrated;
            }
        }
        for i in 0..7 {
            if let Some(goal) = self.joint_goal {
                let remaining = goal[i] - anchor[i];
                let advance = q[i] - anchor[i];
                if remaining * advance < 0.0 || advance.abs() > remaining.abs() {
                    // Clamp only the pending forward motion. Do not chase an old
                    // goal after cancellation; the controller removes joint_goal.
                    q[i] = if remaining * advance >= 0.0 {
                        goal[i]
                    } else {
                        anchor[i]
                    };
                    dq[i] = 0.0;
                    bounded = true;
                }
            }
        }
        if let Some(guard) = &self.cartesian
            && guard.crosses_goal(&anchor, &q)?
        {
            q = previous.map_or(self.q, |p| p.q);
            dq = [0.0; 7];
            bounded = true;
        }
        if !q.iter().chain(dq.iter()).all(|x| x.is_finite()) {
            return Err(SampleStatus::Invalid);
        }
        Ok(Sample {
            q,
            dq,
            timing: Timing {
                status: if bounded {
                    SampleStatus::Bounded
                } else {
                    SampleStatus::Aligned
                },
                age_us: Some(micros(age)),
            },
        })
    }
}

fn micros(t: Duration) -> u64 {
    t.as_micros().min(u64::MAX as u128) as u64
}

fn project_axis(q: f64, v: f64, a: f64, dt: f64, stopping: bool, speed: f64) -> (f64, f64) {
    let v = v.clamp(-speed, speed);
    if dt == 0.0 {
        return (q, v);
    }
    if stopping && v.abs() < 1e-12 {
        return (q, 0.0);
    }
    // A new controller sample owns any direction reversal. Forecast braking
    // only to the first zero crossing, never through it into a new motion.
    let cap = if v * a < 0.0 { 0.0 } else { speed * a.signum() };
    let until = if a.abs() > 1e-12 {
        ((cap - v) / a).clamp(0.0, dt)
    } else {
        dt
    };
    let end_v = (v + a * until).clamp(-speed, speed);
    (
        q + v * until + 0.5 * a * until * until + end_v * (dt - until),
        end_v,
    )
}

impl CartesianGuard {
    fn crosses_goal(&self, source: &[f64; 7], q: &[f64; 7]) -> Result<bool, SampleStatus> {
        let Some(goal) = self.goal else {
            return Ok(false);
        };
        let from = self
            .model
            .fk(source, &self.tcp)
            .map_err(|_| SampleStatus::Constraint)?;
        let next = self
            .model
            .fk(q, &self.tcp)
            .map_err(|_| SampleStatus::Constraint)?;
        let axis = self
            .path
            .axis_in_base()
            .map_err(|_| SampleStatus::Constraint)?;
        let remaining = pose_error(&from, &goal);
        let advance = pose_error(&from, &next);
        let start = if self.path.axis < 3 { 0 } else { 3 };
        let remaining = (0..3).map(|i| remaining[start + i] * axis[i]).sum::<f64>();
        let advance = (0..3).map(|i| advance[start + i] * axis[i]).sum::<f64>();
        Ok(remaining * advance < -1e-14 || advance.abs() > remaining.abs() + 1e-9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[test]
    #[ignore = "requires R17A_CALIBRATION_TRACE pointing to a private calibration report"]
    fn recorded_calibration_sender_does_not_reject_secondary_fk_residuals() {
        let path = std::env::var("R17A_CALIBRATION_TRACE").unwrap();
        let report: crate::calibration::Report =
            serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let model = Arc::new(Kinematics::load(&report.side).unwrap());
        let tcp = report.tcp.unwrap();
        let now = Instant::now();
        let mut count = 0;
        for point in report.curves.iter() {
            let Some(q) = point.command else {
                continue;
            };
            let mut reference = Reference::joint(now, q, point.velocity, [0.0; 7], 1, 1);
            reference.cartesian = Some(CartesianGuard {
                model: model.clone(),
                tcp,
                path: CartesianPath {
                    origin: model.fk(&point.reference, &tcp).unwrap(),
                    axis: 0,
                    frame: crate::kinematics::Frame::Base,
                },
                goal: None,
            });
            assert!(
                reference.sample(now, None).is_ok(),
                "last successful command was rejected at {}ms",
                point.t_ms
            );
            count += 1;
        }
        assert!(count > 1000);
    }

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

    #[test]
    fn cap_and_zero_crossing_integrate_only_the_admissible_motion() {
        for sign in [-1.0, 1.0] {
            let (q, v) = project_axis(0.0, sign * 0.19, sign * 0.6, 0.05, false, 0.2);
            assert!((q - sign * 0.009916666666666667).abs() < 1e-12);
            assert_eq!(v, sign * 0.2);
            let (q, v) = project_axis(0.0, sign * 0.01, -sign * 0.6, 0.05, false, 0.2);
            assert!((q - sign * 0.0000833333333333333).abs() < 1e-12);
            assert_eq!(v, 0.0);
        }
        assert_eq!(project_axis(0.5, 0.0, 0.6, 0.05, true, 0.2), (0.5, 0.0));
    }

    #[test]
    fn freshness_and_invalid_state_never_invent_a_future_target() {
        let now = Instant::now();
        let mut reference = Reference::joint(now, [0.5; 7], [0.01; 7], [0.0; 7], 1, 0);
        assert!(reference.sample(now, None).is_ok());
        assert!(reference.sample(now + MAX_REFERENCE_AGE, None).is_ok());
        assert_eq!(
            reference
                .sample(now + MAX_REFERENCE_AGE + Duration::from_nanos(1), None)
                .unwrap_err(),
            SampleStatus::Stale
        );
        assert_eq!(
            reference
                .sample(now - Duration::from_nanos(1), None)
                .unwrap_err(),
            SampleStatus::Future
        );
        reference.valid_until = Some(now + Duration::from_millis(5));
        assert!(
            reference
                .sample(now + Duration::from_micros(4999), None)
                .is_ok()
        );
        assert_eq!(
            reference
                .sample(now + Duration::from_millis(5), None)
                .unwrap_err(),
            SampleStatus::Expired
        );
        reference.valid_until = None;
        reference.q[0] = f64::NAN;
        assert_eq!(
            reference.sample(now, None).unwrap_err(),
            SampleStatus::Invalid
        );
        reference.q = [0.5; 7];
        reference.joint_goal = Some([f64::NAN; 7]);
        assert_eq!(
            reference.sample(now, None).unwrap_err(),
            SampleStatus::Invalid
        );
        reference.joint_goal = None;
        let previous = Previous {
            at: now - Duration::from_millis(10),
            q: [f64::NAN; 7],
            dq: [0.0; 7],
        };
        assert_eq!(
            reference.sample(now, Some(previous)).unwrap_err(),
            SampleStatus::Invalid
        );
    }

    #[test]
    fn goals_bound_projection_and_cancellation_removes_the_old_goal() {
        let now = Instant::now();
        for sign in [-1.0, 1.0] {
            let mut reference = Reference::joint(now, [0.0; 7], [sign * 0.05; 7], [0.0; 7], 1, 0);
            reference.joint_goal = Some([sign * 0.001; 7]);
            let sample = reference.sample(now + MAX_REFERENCE_AGE, None).unwrap();
            assert_eq!(sample.q, [sign * 0.001; 7]);
            assert_eq!(sample.dq, [0.0; 7]);
            reference.joint_goal = None;
            reference.stopping = true;
            let sample = reference.sample(now + MAX_REFERENCE_AGE, None).unwrap();
            assert!((sample.q[0] - sign * 0.0025).abs() < 1e-12);
            reference.dq = [0.0; 7];
            reference.ddq = [0.0; 7];
            assert_eq!(
                reference.sample(now + MAX_REFERENCE_AGE, None).unwrap().q,
                reference.q
            );
        }
    }

    #[test]
    fn a_new_source_continues_from_the_previous_successful_target() {
        let now = Instant::now();
        let reference = Reference::joint(now, [0.02; 7], [0.0; 7], [0.0; 7], 1, 0);
        let previous = Previous {
            at: now - Duration::from_millis(10),
            q: [0.0; 7],
            dq: [0.0; 7],
        };
        let sample = reference.sample(now, Some(previous)).unwrap();
        assert_eq!(sample.q, [0.0; 7]);
        assert_eq!(sample.dq, [0.0; 7]);
    }

    #[test]
    fn jittered_source_changes_do_not_reverse_braking_or_exceed_send_speed() {
        use crate::trajectory::{JOINT_JERK, integrate, velocity_window};
        // Preserve the recorded 0.2 rad/s regression scenario independently of
        // the application's editable operating-speed preference.
        const SPEED: f64 = 0.2;
        let origin = Instant::now();
        for sign in [-1.0, 1.0] {
            let mut sources = vec![Reference::joint(origin, [0.0; 7], [0.0; 7], [0.0; 7], 1, 0)];
            let (mut q, mut v, mut a) = ([0.0; 7], [0.0; 7], [0.0; 7]);
            let mut time = 0.0;
            for tick in 0..200 {
                let dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                time += dt;
                let desired = [if time < 0.8 { sign * SPEED } else { 0.0 }; 7];
                let window = velocity_window(
                    v,
                    a,
                    desired,
                    [SPEED; 7],
                    [JOINT_ACCELERATION; 7],
                    [JOINT_JERK; 7],
                    dt,
                )
                .unwrap();
                let next = std::array::from_fn(|i| desired[i].clamp(window.low[i], window.high[i]));
                (q, a) = integrate(q, v, next, dt);
                v = next;
                let mut source =
                    Reference::joint(origin + Duration::from_secs_f64(time), q, v, a, 1, 0);
                source.joint_velocity = [SPEED; 7];
                source.stopping = time >= 0.8;
                sources.push(source);
            }
            let mut previous: Option<Previous> = None;
            let mut old_previous: Option<(Instant, f64, f64)> = None;
            let (mut source_index, mut old_speed_errors, mut old_reversals) = (0, 0, 0);
            for tick in 0..250 {
                let at = origin + Duration::from_millis(2 + tick * 10);
                while sources.get(source_index + 1).is_some_and(|s| s.at <= at) {
                    source_index += 1;
                }
                let source = &sources[source_index];
                let age = at.duration_since(source.at).as_secs_f64();
                let (old_q, old_v) = project_axis(
                    source.q[0],
                    source.dq[0],
                    source.ddq[0],
                    age,
                    source.stopping,
                    SPEED,
                );
                if let Some((last_at, last_q, last_v)) = old_previous {
                    let dt = at.duration_since(last_at).as_secs_f64();
                    old_speed_errors += usize::from((old_q - last_q).abs() > SPEED * dt + 1e-10);
                    old_reversals += usize::from(
                        sign * old_v > 0.0
                            && sign * last_v > 0.0
                            && sign * (old_q - last_q) < -1e-10,
                    );
                }
                old_previous = Some((at, old_q, old_v));
                let sample = source.sample(at, previous).unwrap();
                if let Some(p) = previous {
                    let delta = sample.q[0] - p.q[0];
                    assert!(delta.abs() <= SPEED * at.duration_since(p.at).as_secs_f64() + 1e-10);
                    assert!(sign * delta >= -1e-10);
                }
                previous = Some(Previous {
                    at,
                    q: sample.q,
                    dq: sample.dq,
                });
            }
            assert!(
                old_speed_errors > 0 && old_reversals > 0,
                "regression clock must reproduce both old defects"
            );
            assert_eq!(previous.unwrap().dq, [0.0; 7]);
            assert!((0..7).all(|i| (previous.unwrap().q[i] - q[i]).abs() < 0.01));
        }
    }

    #[test]
    fn recorded_send_clock_resamples_a_held_joint_jog() {
        let clock: Clock =
            serde_json::from_str(include_str!("../tests/fixtures/send-clock.json")).unwrap();
        let origin = Instant::now();
        let speed = 0.06825;
        let mut previous: Option<(u64, f64)> = None;
        let mut error2 = 0.0;
        let mut count = 0;
        for send in clock.sends {
            let source_us = clock.control_us[send.source];
            let source = Reference::joint(
                origin + Duration::from_micros(source_us),
                [speed * source_us as f64 * 1e-6; 7],
                [speed; 7],
                [0.0; 7],
                1,
                0,
            );
            let sample = source
                .sample(origin + Duration::from_micros(send.at_us), None)
                .unwrap();
            if let Some((at, q)) = previous {
                let dt = (send.at_us - at) as f64 * 1e-6;
                if send.at_us >= 2_500_000 && send.at_us <= 6_500_000 && dt < 0.015 {
                    error2 += ((sample.q[3] - q) / dt - speed).powi(2);
                    count += 1;
                }
            }
            previous = Some((send.at_us, sample.q[3]));
        }
        let relative = (error2 / count as f64).sqrt() / speed;
        assert!(count > 300);
        assert!(
            relative < 0.01,
            "send-time position/velocity RMS mismatch: {:.3}%",
            relative * 100.0
        );
    }
}
