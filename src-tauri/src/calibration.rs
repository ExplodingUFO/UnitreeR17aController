//! Offline low-speed calibration candidates, trajectories, and measurements.

use crate::tuning::ServoTuning;
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

const STILL_WINDOW: Duration = Duration::from_millis(300);
const STILL_MAX_GAP: Duration = Duration::from_millis(100);
const STILL_CAPACITY: usize = 128;
const STILL_POSITION_LIMIT: f64 = 0.001;

#[derive(Clone, Debug, Serialize)]
pub struct Readiness {
    pub state: &'static str,
    pub code: String,
    pub message: String,
    pub joint: Option<usize>,
    pub observed_ms: u64,
    pub required_ms: u64,
    pub position_span: Option<f64>,
    pub position_limit: f64,
}

impl Readiness {
    pub fn unavailable(code: &str, message: impl Into<String>) -> Self {
        Self {
            state: "unavailable",
            code: code.into(),
            message: message.into(),
            joint: None,
            observed_ms: 0,
            required_ms: STILL_WINDOW.as_millis() as u64,
            position_span: None,
            position_limit: STILL_POSITION_LIMIT,
        }
    }
    pub fn ready(&self) -> bool {
        self.state == "ready"
    }
}

#[derive(Clone, Copy)]
struct PositionObservation {
    at: Instant,
    q: f64,
}

/// Bounded position history for every received joint, not UI/control snapshots.
/// This observes holding stability; it never filters or modifies device feedback.
pub struct Stillness {
    joints: [VecDeque<PositionObservation>; 7],
}

impl Default for Stillness {
    fn default() -> Self {
        Self {
            joints: std::array::from_fn(|_| VecDeque::with_capacity(STILL_CAPACITY)),
        }
    }
}

impl Stillness {
    pub fn clear(&mut self) {
        for joint in &mut self.joints {
            joint.clear();
        }
    }
    pub fn observe(&mut self, joint: usize, q: f64, at: Instant) {
        let Some(history) = self.joints.get_mut(joint) else {
            return;
        };
        if !q.is_finite() {
            history.clear();
            return;
        }
        if history
            .back()
            .is_some_and(|last| at <= last.at || at.duration_since(last.at) > STILL_MAX_GAP)
        {
            history.clear();
        }
        if history.len() == STILL_CAPACITY {
            history.pop_front();
        }
        history.push_back(PositionObservation { at, q });
        if let Some(cutoff) = at.checked_sub(STILL_WINDOW) {
            // Keep the sample at/before the beginning: shorter coverage cannot
            // pass merely because the retained samples have a small span.
            while history.len() > 1 && history.get(1).is_some_and(|sample| sample.at <= cutoff) {
                history.pop_front();
            }
        }
    }
    pub fn status(&self, now: Instant) -> Readiness {
        let mut result = Readiness::unavailable("feedback_missing", "等待七轴位置反馈");
        let mut observed_ms = u64::MAX;
        let mut worst = (0usize, 0.0_f64);
        for (joint, history) in self.joints.iter().enumerate() {
            let Some(last) = history.back() else {
                result.joint = Some(joint);
                result.message = format!("等待 J{} 位置反馈", joint + 1);
                return result;
            };
            if now
                .checked_duration_since(last.at)
                .is_none_or(|age| age > STILL_MAX_GAP)
            {
                result.code = "feedback_stale".into();
                result.joint = Some(joint);
                result.message = format!("等待 J{} 最新位置反馈", joint + 1);
                return result;
            }
            let first = history.front().unwrap();
            observed_ms = observed_ms.min(
                last.at
                    .duration_since(first.at)
                    .as_millis()
                    .min(u64::MAX as u128) as u64,
            );
            let (low, high) = history
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), sample| {
                    (low.min(sample.q), high.max(sample.q))
                });
            let span = high - low;
            if !span.is_finite() {
                result.code = "invalid_feedback".into();
                result.message = format!("J{} 位置反馈无效", joint + 1);
                result.joint = Some(joint);
                return result;
            }
            if span > worst.1 {
                worst = (joint, span);
            }
        }
        result.observed_ms = observed_ms.min(result.required_ms);
        result.joint = Some(worst.0);
        result.position_span = Some(worst.1);
        if worst.1 > STILL_POSITION_LIMIT {
            result.state = "moving";
            result.code = "position_moving".into();
            result.message = format!("J{} 位置仍在变化，等待停稳", worst.0 + 1);
        } else if observed_ms < result.required_ms {
            result.state = "settling";
            result.code = "observing".into();
            result.message = format!(
                "正在确认静止（{} / {} ms）",
                result.observed_ms, result.required_ms
            );
        } else {
            result.state = "ready";
            result.code = "ready".into();
            result.message = "位置已稳定，可以开始校准".into();
        }
        result
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Settings {
    pub axis: usize,
    pub excursion: f64,
    pub speed: f64,
    pub max_friction: f64,
}

fn validate_settings(settings: &Settings) -> Result<(), String> {
    if settings.axis >= 7 {
        return Err("校准关节必须为 0..6".into());
    }
    for (name, value, min, max) in [
        // Include standard degree conversion and the published rounded limits.
        (
            "excursion",
            settings.excursion,
            1.0f64.to_radians(),
            10.0f64.to_radians().max(0.1745329252),
        ),
        ("speed", settings.speed, 0.005, 0.04),
        ("max_friction", settings.max_friction, 0.0, 2.0),
    ] {
        if !value.is_finite() || !(min..=max).contains(&value) {
            return Err(format!("校准 {name} 无效或超限"));
        }
    }
    Ok(())
}

pub fn validate(settings: &Settings, baseline: &ServoTuning) -> Result<(), String> {
    validate_settings(settings)?;
    baseline.validate()?;
    if !baseline.friction_enabled
        && baseline.joints.iter().enumerate().any(|(axis, joint)| {
            axis != settings.axis && (joint.coulomb != 0.0 || joint.viscous != 0.0)
        })
    {
        return Err("其他关节存在未启用的摩擦系数，不能安全开启全局摩擦补偿".into());
    }
    Ok(())
}

pub fn candidates(settings: &Settings, baseline: ServoTuning) -> Result<Vec<ServoTuning>, String> {
    validate(settings, &baseline)?;
    let mut result = Vec::with_capacity(4);
    result.push(baseline);
    if settings.max_friction > 0.0 {
        for coulomb in [settings.max_friction / 2.0, settings.max_friction] {
            let mut candidate = baseline;
            candidate.friction_enabled = true;
            candidate.joints[settings.axis].coulomb = coulomb;
            candidate.joints[settings.axis].max_friction = settings.max_friction;
            if !result.contains(&candidate) {
                result.push(candidate);
            }
        }
    }
    let mut feedforward = baseline;
    feedforward.joints[settings.axis].velocity_ff *= 0.75;
    if !result.contains(&feedforward) {
        result.push(feedforward);
    }
    Ok(result)
}

#[derive(Clone, Debug)]
pub struct Leg {
    pub target: [f64; 7],
    pub speed: f64,
}

pub fn legs(settings: &Settings, center: [f64; 7]) -> Vec<Leg> {
    if validate_settings(settings).is_err() || center.iter().any(|value| !value.is_finite()) {
        return Vec::new();
    }
    let mut result = Vec::with_capacity(6);
    for speed in [settings.speed, settings.speed * 2.0] {
        for offset in [settings.excursion / 2.0, -settings.excursion / 2.0, 0.0] {
            let mut target = center;
            if offset != 0.0 {
                target[settings.axis] += offset;
            }
            result.push(Leg { target, speed });
        }
    }
    result
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Metrics {
    pub windows: u64,
    pub velocity_rms: f64,
    pub tracking_rms: f64,
    pub tracking_peak: f64,
    pub other_peak: f64,
    pub valid_ms: u64,
}

const WINDOW_US: u64 = 100_000;
const MAX_GAP_US: u64 = 300_000;
const MIN_WINDOWS: u64 = 12;
const MIN_VALID_MS: u64 = 2000;

#[derive(Clone, Copy)]
struct WindowStart {
    at_us: u64,
    reference: f64,
    measured: f64,
}

#[derive(Default)]
struct Rms {
    count: u64,
    value: f64,
}

impl Rms {
    fn observe(&mut self, value: f64) {
        self.count = self.count.saturating_add(1);
        let weight = 1.0 / self.count as f64;
        // Avoid overflowing an intermediate sum of squares for finite inputs.
        self.value = (self.value * (1.0 - weight).sqrt()).hypot(value * weight.sqrt());
    }
}

/// Constant-space measurements of the original, unscaled motion reference.
/// A window closes at the first valid moving frame at least 100 ms after its
/// start, using the actual elapsed time. No interpolation creates extra windows.
pub struct Collector {
    axis: usize,
    initial_measured: [f64; 7],
    last_at_us: Option<u64>,
    window: Option<WindowStart>,
    velocity: Rms,
    tracking: Rms,
    tracking_peak: f64,
    other_peak: f64,
    valid_us: u64,
}

impl Collector {
    pub fn new(axis: usize, initial_measured: [f64; 7]) -> Self {
        Self {
            axis,
            initial_measured,
            last_at_us: None,
            window: None,
            velocity: Rms::default(),
            tracking: Rms::default(),
            tracking_peak: 0.0,
            other_peak: 0.0,
            valid_us: 0,
        }
    }

    /// Stops and invalid numbers discard the partial window. A finite moving
    /// frame after a clock discontinuity or long gap may start a new window,
    /// but contributes no velocity or valid duration across the discontinuity.
    pub fn observe(&mut self, at_us: u64, reference: [f64; 7], measured: [f64; 7], moving: bool) {
        if !moving
            || self.axis >= 7
            || self
                .initial_measured
                .iter()
                .chain(reference.iter())
                .chain(measured.iter())
                .any(|value| !value.is_finite())
        {
            self.break_window();
            return;
        }
        let tracking_error = reference[self.axis] - measured[self.axis];
        let mut other_peak: f64 = 0.0;
        for (axis, position) in measured.iter().enumerate() {
            if axis != self.axis {
                let drift = (position - self.initial_measured[axis]).abs();
                if !drift.is_finite() {
                    self.break_window();
                    return;
                }
                other_peak = other_peak.max(drift);
            }
        }
        if !tracking_error.is_finite() {
            self.break_window();
            return;
        }
        if let Some(last) = self.last_at_us
            && (at_us <= last || at_us - last > MAX_GAP_US)
        {
            self.window = None;
        }
        self.last_at_us = Some(at_us);
        let current = WindowStart {
            at_us,
            reference: reference[self.axis],
            measured: measured[self.axis],
        };
        if let Some(start) = self.window {
            let elapsed_us = at_us - start.at_us;
            if elapsed_us >= WINDOW_US {
                let reference_delta = current.reference - start.reference;
                let measured_delta = current.measured - start.measured;
                let error = (measured_delta - reference_delta) / (elapsed_us as f64 / 1_000_000.0);
                if !error.is_finite() {
                    self.break_window();
                    return;
                }
                self.velocity.observe(error);
                self.valid_us = self.valid_us.saturating_add(elapsed_us);
                self.window = Some(current);
            }
        } else {
            self.window = Some(current);
        }
        self.tracking.observe(tracking_error);
        self.tracking_peak = self.tracking_peak.max(tracking_error.abs());
        self.other_peak = self.other_peak.max(other_peak);
    }

    pub fn finish(&self) -> Option<Metrics> {
        let metrics = Metrics {
            windows: self.velocity.count,
            velocity_rms: self.velocity.value,
            tracking_rms: self.tracking.value,
            tracking_peak: self.tracking_peak,
            other_peak: self.other_peak,
            valid_ms: self.valid_us / 1000,
        };
        metrics_valid(&metrics).then_some(metrics)
    }

    fn break_window(&mut self) {
        self.window = None;
        self.last_at_us = None;
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Trial {
    pub index: usize,
    pub phase: String,
    pub config: ServoTuning,
    pub metrics: Option<Metrics>,
}

fn metrics_valid(metrics: &Metrics) -> bool {
    metrics.windows >= MIN_WINDOWS
        && metrics.valid_ms >= MIN_VALID_MS
        && [
            metrics.velocity_rms,
            metrics.tracking_rms,
            metrics.tracking_peak,
            metrics.other_peak,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
}

fn valid_metrics(trial: &Trial) -> Option<&Metrics> {
    trial
        .metrics
        .as_ref()
        .filter(|metrics| metrics_valid(metrics))
}

// Inclusive thresholds tolerate only floating-point roundoff, not a physical
// measurement allowance. The baseline's strict minimum remains strict.
fn at_most(value: f64, limit: f64) -> bool {
    value <= limit || value - limit <= 8.0 * f64::EPSILON * value.abs().max(limit.abs())
}

fn improvement(baseline: &Metrics, candidate: &Metrics) -> Option<f64> {
    if baseline.velocity_rms <= 0.0005 {
        return None;
    }
    let reduction = baseline.velocity_rms - candidate.velocity_rms;
    if !at_most(baseline.velocity_rms * 0.15, reduction)
        || !at_most(0.0001, reduction)
        || !at_most(
            candidate.tracking_peak,
            baseline.tracking_peak * 1.1 + 0.003,
        )
        || !at_most(candidate.other_peak, baseline.other_peak + 0.005)
    {
        return None;
    }
    Some(reduction / baseline.velocity_rms)
}

pub fn recommend(baseline: &Trial, candidates: &[Trial]) -> Option<usize> {
    let baseline = valid_metrics(baseline)?;
    let mut best: Option<(usize, f64)> = None;
    for candidate in candidates {
        let Some(metrics) = valid_metrics(candidate) else {
            continue;
        };
        if improvement(baseline, metrics).is_some()
            && best.is_none_or(|(_, error)| metrics.velocity_rms < error)
        {
            best = Some((candidate.index, metrics.velocity_rms));
        }
    }
    best.map(|(index, _)| index)
}

pub fn confirm(
    baseline: &Trial,
    baseline_check: &Trial,
    candidate: &Trial,
    validation: &Trial,
) -> Option<f64> {
    let baseline = valid_metrics(baseline)?;
    let baseline_check = valid_metrics(baseline_check)?;
    let candidate = valid_metrics(candidate)?;
    let validation = valid_metrics(validation)?;
    if !at_most(
        (baseline_check.velocity_rms - baseline.velocity_rms).abs(),
        baseline.velocity_rms * 0.25 + 0.0005,
    ) {
        return None;
    }
    let first = improvement(baseline, candidate)?;
    let repeated = improvement(baseline_check, validation)?;
    Some(first.min(repeated))
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    pub id: String,
    pub request_id: String,
    pub created_at_ms: i64,
    pub side: String,
    pub port: String,
    pub settings: Settings,
    pub center_reference: [f64; 7],
    pub center_measured: [f64; 7],
    pub tcp: Option<crate::kinematics::Pose>,
    pub baseline: ServoTuning,
    pub recommended: Option<ServoTuning>,
    pub trials: Vec<Trial>,
    pub outcome: String,
    pub improvement: Option<f64>,
    pub message: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Status {
    pub id: String,
    pub request_id: String,
    pub state: String,
    pub stage: String,
    pub axis: usize,
    pub trial_index: usize,
    pub trial_count: usize,
    pub leg_index: usize,
    pub leg_count: usize,
    pub progress: f64,
    pub report: Option<Report>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observe_hold(detector: &mut Stillness, origin: Instant, start: u64, end: u64) {
        for tick in start..=end {
            for joint in 0..7 {
                let q = joint as f64 * 0.1 + if tick % 2 == 0 { 0.00006 } else { -0.00006 };
                detector.observe(joint, q, origin + Duration::from_millis(tick * 10));
            }
        }
    }

    #[test]
    fn stillness_requires_a_fresh_window_for_every_axis() {
        let origin = Instant::now();
        let mut detector = Stillness::default();
        assert!(!detector.status(origin).ready());
        observe_hold(&mut detector, origin, 0, 29);
        let waiting = detector.status(origin + Duration::from_millis(290));
        assert_eq!(waiting.state, "settling");
        assert_eq!(waiting.observed_ms, 290);
        observe_hold(&mut detector, origin, 30, 30);
        assert!(detector.status(origin + Duration::from_millis(300)).ready());
        assert!(!detector.status(origin + Duration::from_millis(401)).ready());
    }

    #[test]
    fn stillness_rejects_drift_and_a_recovered_spike_in_the_window() {
        let origin = Instant::now();
        let mut detector = Stillness::default();
        for tick in 0..=40 {
            for joint in 0..7 {
                detector.observe(
                    joint,
                    if joint == 2 {
                        tick as f64 * 0.00005
                    } else {
                        0.0
                    },
                    origin + Duration::from_millis(tick * 10),
                );
            }
        }
        let moving = detector.status(origin + Duration::from_millis(400));
        assert_eq!(moving.state, "moving");
        assert_eq!(moving.joint, Some(2));
        assert!(moving.position_span.unwrap() > moving.position_limit);
        detector.clear();
        observe_hold(&mut detector, origin, 0, 40);
        detector.observe(5, 0.51, origin + Duration::from_millis(401));
        detector.observe(5, 0.5, origin + Duration::from_millis(402));
        assert_eq!(
            detector.status(origin + Duration::from_millis(402)).state,
            "moving"
        );
        observe_hold(&mut detector, origin, 41, 72);
        assert!(detector.status(origin + Duration::from_millis(720)).ready());
    }

    #[test]
    fn stillness_never_bridges_a_gap_or_a_clock_discontinuity() {
        let origin = Instant::now();
        for discontinuity in [
            Duration::from_millis(400),
            Duration::from_millis(390),
            Duration::from_millis(501),
        ] {
            let mut detector = Stillness::default();
            observe_hold(&mut detector, origin, 0, 40);
            detector.observe(4, 0.4, origin + discontinuity);
            assert!(!detector.status(origin + discontinuity).ready());
        }
    }

    #[test]
    fn stillness_invalid_or_missing_axis_cannot_be_hidden_by_other_axes() {
        let origin = Instant::now();
        let mut detector = Stillness::default();
        observe_hold(&mut detector, origin, 0, 40);
        detector.observe(3, f64::NAN, origin + Duration::from_millis(401));
        assert!(!detector.status(origin + Duration::from_millis(401)).ready());
        detector.observe(3, 0.3, origin + Duration::from_millis(402));
        assert!(!detector.status(origin + Duration::from_millis(402)).ready());
        detector.clear();
        observe_hold(&mut detector, origin, 0, 40);
        for tick in 41..=55 {
            for joint in 0..6 {
                detector.observe(
                    joint,
                    joint as f64 * 0.1,
                    origin + Duration::from_millis(tick * 10),
                );
            }
        }
        let stale = detector.status(origin + Duration::from_millis(550));
        assert_eq!(stale.code, "feedback_stale");
        assert_eq!(stale.joint, Some(6));
    }

    #[test]
    fn stillness_history_is_bounded_and_clear_removes_old_evidence() {
        let origin = Instant::now();
        let mut detector = Stillness::default();
        for tick in 0..2000 {
            for joint in 0..7 {
                detector.observe(joint, 0.0, origin + Duration::from_micros(tick));
            }
        }
        for joint in &detector.joints {
            assert!(joint.len() <= STILL_CAPACITY);
            assert_eq!(joint.capacity(), STILL_CAPACITY);
        }
        assert!(
            !detector
                .status(origin + Duration::from_micros(1999))
                .ready()
        );
        detector.clear();
        observe_hold(&mut detector, origin, 0, 40);
        assert!(detector.status(origin + Duration::from_millis(400)).ready());
        detector.clear();
        assert!(!detector.status(origin + Duration::from_millis(400)).ready());
    }
    use serde_json::{Value, json};

    fn settings() -> Settings {
        Settings {
            axis: 3,
            excursion: 0.1,
            speed: 0.02,
            max_friction: 0.8,
        }
    }

    fn close(actual: f64, expected: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() < 1e-10,
            "expected {expected}, got {actual}"
        );
    }

    fn tuning_bits_equal(actual: ServoTuning, expected: ServoTuning) {
        assert_eq!(actual.friction_enabled, expected.friction_enabled);
        for (actual, expected) in actual.joints.into_iter().zip(expected.joints) {
            for (a, e) in [
                (actual.kd, expected.kd),
                (actual.velocity_ff, expected.velocity_ff),
                (actual.coulomb, expected.coulomb),
                (actual.viscous, expected.viscous),
                (actual.velocity_scale, expected.velocity_scale),
                (actual.max_friction, expected.max_friction),
            ] {
                assert_eq!(a.to_bits(), e.to_bits());
            }
        }
    }

    #[test]
    fn settings_accept_inclusive_si_boundaries_on_every_axis() {
        for axis in 0..7 {
            for (excursion, speed, max_friction) in [
                (0.01745329252, 0.005, 0.0),
                (0.1745329252, 0.04, 2.0),
                (1.0f64.to_radians(), 0.005, 0.0),
                (10.0f64.to_radians(), 0.04, 2.0),
            ] {
                assert!(
                    validate(
                        &Settings {
                            axis,
                            excursion,
                            speed,
                            max_friction
                        },
                        &ServoTuning::default(),
                    )
                    .is_ok()
                );
            }
        }
    }

    #[test]
    fn settings_reject_wrong_axis_nonfinite_and_out_of_range_values() {
        let baseline = ServoTuning::default();
        for axis in [7, usize::MAX] {
            assert!(validate(&Settings { axis, ..settings() }, &baseline).is_err());
        }
        for (field, invalid) in [
            ("excursion", 0.017453292519),
            ("excursion", 0.174532925201),
            ("speed", 0.004999999),
            ("speed", 0.040000001),
            ("max_friction", -0.000000001),
            ("max_friction", 2.000000001),
        ]
        .into_iter()
        .chain(
            ["excursion", "speed", "max_friction"]
                .into_iter()
                .flat_map(|field| {
                    [f64::NAN, f64::INFINITY, f64::NEG_INFINITY].map(|value| (field, value))
                }),
        ) {
            let mut input = settings();
            match field {
                "excursion" => input.excursion = invalid,
                "speed" => input.speed = invalid,
                _ => input.max_friction = invalid,
            }
            assert!(validate(&input, &baseline).is_err(), "{field}={invalid}");
            assert!(candidates(&input, baseline).is_err());
        }
    }

    #[test]
    fn validation_rejects_invalid_baseline_and_dormant_other_axis_friction() {
        let input = settings();
        let mut baseline = ServoTuning::default();
        baseline.joints[0].kd = 31.0;
        assert!(validate(&input, &baseline).is_err());
        baseline = ServoTuning::default();
        baseline.joints[input.axis].coulomb = 0.4;
        baseline.joints[input.axis].viscous = 0.5;
        assert!(validate(&input, &baseline).is_ok());
        for other in [0, 6] {
            for viscous in [false, true] {
                let mut dormant = baseline;
                if viscous {
                    dormant.joints[other].viscous = 0.1;
                } else {
                    dormant.joints[other].coulomb = 0.1;
                }
                dormant.joints[other].max_friction = 0.0;
                let original = dormant;
                assert!(validate(&input, &dormant).is_err());
                assert!(candidates(&input, dormant).is_err());
                tuning_bits_equal(dormant, original);
                dormant.friction_enabled = true;
                assert!(validate(&input, &dormant).is_ok());
            }
        }
    }

    #[test]
    fn candidates_change_only_selected_coulomb_or_selected_velocity_feedforward() {
        let input = settings();
        let mut baseline = ServoTuning {
            friction_enabled: true,
            ..ServoTuning::default()
        };
        for (axis, joint) in baseline.joints.iter_mut().enumerate() {
            joint.kd = axis as f64 + 8.0;
            joint.coulomb = 0.05 * axis as f64;
            joint.viscous = 0.1 * axis as f64;
            joint.velocity_ff = 0.8;
            joint.velocity_scale = 0.03;
            joint.max_friction = 1.2;
        }
        baseline.joints[0].coulomb = -0.0;
        let actual = candidates(&input, baseline).unwrap();
        assert_eq!(actual.len(), 4);
        tuning_bits_equal(actual[0], baseline);
        for coulomb in [0.4, 0.8] {
            let mut expected = baseline;
            expected.friction_enabled = true;
            expected.joints[input.axis].coulomb = coulomb;
            expected.joints[input.axis].max_friction = 0.8;
            let candidate = actual[1..]
                .iter()
                .find(|candidate| **candidate == expected)
                .unwrap();
            tuning_bits_equal(*candidate, expected);
            assert!(candidate.validate().is_ok());
        }
        let mut expected = baseline;
        expected.joints[input.axis].velocity_ff = 0.6000000000000001;
        let ff = actual[1..]
            .iter()
            .find(|candidate| **candidate == expected)
            .unwrap();
        tuning_bits_equal(*ff, expected);
    }

    #[test]
    fn candidates_enable_only_safe_friction_and_deduplicate_zero_searches() {
        let input = settings();
        let baseline = ServoTuning::default();
        let actual = candidates(&input, baseline).unwrap();
        assert_eq!(actual.len(), 4);
        assert!(!actual[0].friction_enabled);
        assert_eq!(
            actual[1..]
                .iter()
                .filter(|candidate| candidate.friction_enabled)
                .count(),
            2
        );
        assert_eq!(
            actual[1..]
                .iter()
                .filter(|candidate| !candidate.friction_enabled)
                .count(),
            1
        );
        let mut zero = Settings {
            max_friction: 0.0,
            ..input.clone()
        };
        assert_eq!(candidates(&zero, baseline).unwrap().len(), 2);
        let mut no_ff = baseline;
        no_ff.joints[zero.axis].velocity_ff = 0.0;
        let actual = candidates(&zero, no_ff).unwrap();
        assert_eq!(actual.len(), 1);
        tuning_bits_equal(actual[0], no_ff);
        zero.max_friction = 0.8;
        let mut existing = baseline;
        existing.friction_enabled = true;
        existing.joints[zero.axis].coulomb = 0.4;
        existing.joints[zero.axis].max_friction = 0.8;
        let actual = candidates(&zero, existing).unwrap();
        assert_eq!(actual.len(), 3);
        tuning_bits_equal(actual[0], existing);
        assert!(
            actual[1..]
                .iter()
                .any(|candidate| candidate.joints[zero.axis].coulomb == 0.8)
        );
    }

    #[test]
    fn legs_use_half_excursion_then_return_to_center_at_two_speeds() {
        let input = settings();
        let center = [0.5, -0.4, 0.3, -0.2, 0.1, -0.0, 0.7];
        let actual = legs(&input, center);
        assert_eq!(actual.len(), 6);
        for (leg, (selected, speed)) in actual.iter().zip([
            (-0.15, 0.02),
            (-0.25, 0.02),
            (-0.2, 0.02),
            (-0.15, 0.04),
            (-0.25, 0.04),
            (-0.2, 0.04),
        ]) {
            close(leg.target[input.axis], selected);
            close(leg.speed, speed);
            for (axis, position) in center.iter().enumerate() {
                if axis != input.axis {
                    assert_eq!(leg.target[axis].to_bits(), position.to_bits());
                }
            }
        }
        let fastest = legs(
            &Settings {
                speed: 0.04,
                ..input
            },
            center,
        );
        close(fastest[5].speed, 0.08);
    }

    fn point(axis: usize, selected: f64) -> [f64; 7] {
        let mut point = [0.0; 7];
        point[axis] = selected;
        point
    }

    fn steady(windows: u64, step_us: u64, reference_speed: f64, measured_speed: f64) -> Collector {
        let mut collector = Collector::new(3, [0.0; 7]);
        for sample in 0..=windows {
            let at_us = sample * step_us;
            let seconds = at_us as f64 / 1_000_000.0;
            collector.observe(
                at_us,
                point(3, seconds * reference_speed),
                point(3, seconds * measured_speed),
                true,
            );
        }
        collector
    }

    #[test]
    fn collector_requires_both_window_count_and_two_seconds_of_valid_motion() {
        assert!(Collector::new(3, [0.0; 7]).finish().is_none());
        assert!(steady(12, 100_000, 0.02, 0.02).finish().is_none());
        assert!(steady(19, 100_000, 0.02, 0.02).finish().is_none());
        let exact = steady(20, 100_000, 0.02, 0.02).finish().unwrap();
        assert_eq!((exact.windows, exact.valid_ms), (20, 2000));
        // Sparse valid feedback closes at its first frame >=100 ms; use the
        // actual span, without inventing interpolated observations or windows.
        assert!(steady(11, 200_000, 0.02, 0.02).finish().is_none());
        let enough = steady(12, 200_000, 0.02, 0.02).finish().unwrap();
        assert_eq!((enough.windows, enough.valid_ms), (12, 2400));
        close(enough.velocity_rms, 0.0);
    }

    #[test]
    fn collector_uses_position_windows_despite_microsecond_packet_jitter() {
        let mut collector = Collector::new(3, [0.0; 7]);
        collector.observe(0, point(3, 0.0), point(3, -0.001), true);
        for window in 1..=24 {
            let at_us = window * 100_000;
            let before = (at_us - 1) as f64 / 1_000_000.0;
            collector.observe(
                at_us - 1,
                point(3, before * 0.02),
                point(3, before * 0.018 - 0.001 + 0.00005),
                true,
            );
            let seconds = at_us as f64 / 1_000_000.0;
            collector.observe(
                at_us,
                point(3, seconds * 0.02),
                point(3, seconds * 0.018 - 0.001),
                true,
            );
        }
        let actual = collector.finish().unwrap();
        assert_eq!((actual.windows, actual.valid_ms), (24, 2400));
        close(actual.velocity_rms, 0.002);
        close(actual.tracking_peak, 0.0058);
    }

    fn assert_interrupt_breaks_window(
        event_us: u64,
        moving: bool,
        bad_reference: bool,
        bad_measured: bool,
    ) {
        let mut collector = steady(20, 100_000, 0.02, 0.02);
        collector.observe(2_050_000, point(3, 0.041), point(3, 0.041), true);
        let mut reference = point(3, 0.041);
        let mut measured = point(3, 0.141);
        if bad_reference {
            reference[6] = f64::NAN;
        }
        if bad_measured {
            measured[0] = f64::INFINITY;
        }
        collector.observe(event_us, reference, measured, moving);
        let next = 2_100_000.max(event_us + 50_000);
        collector.observe(next, point(3, 0.042), point(3, 0.142), true);
        collector.observe(next + 100_000, point(3, 0.044), point(3, 0.144), true);
        let actual = collector.finish().unwrap();
        assert_eq!(actual.windows, 21);
        // A finite discontinuity frame may itself anchor the new window.
        assert!(
            (2100..=2160).contains(&actual.valid_ms),
            "valid_ms={}",
            actual.valid_ms
        );
        close(actual.velocity_rms, 0.0);
        close(actual.tracking_peak, 0.1);
    }

    #[test]
    fn collector_breaks_windows_on_stops_duplicates_and_backwards_times() {
        assert_interrupt_breaks_window(2_060_000, false, false, false);
        assert_interrupt_breaks_window(2_050_000, true, false, false);
        assert_interrupt_breaks_window(2_040_000, true, false, false);
    }

    #[test]
    fn collector_allows_300ms_feedback_intervals_but_never_bridges_a_longer_gap() {
        let mut continuous = steady(20, 100_000, 0.02, 0.02);
        continuous.observe(2_300_000, point(3, 0.046), point(3, 0.046), true);
        let actual = continuous.finish().unwrap();
        assert_eq!((actual.windows, actual.valid_ms), (21, 2300));
        close(actual.velocity_rms, 0.0);
        assert_interrupt_breaks_window(2_350_001, true, false, false);
    }

    #[test]
    fn collector_excludes_a_long_stationary_period_and_restarts_after_it() {
        let mut collector = steady(20, 100_000, 0.02, 0.02);
        let before = serde_json::to_value(collector.finish()).unwrap();
        for sample in 21..=60 {
            collector.observe(sample * 100_000, point(3, 10.0), point(3, -10.0), false);
        }
        assert_eq!(serde_json::to_value(collector.finish()).unwrap(), before);
        collector.observe(6_100_000, point(3, 0.0), point(3, 0.1), true);
        collector.observe(6_200_000, point(3, 0.002), point(3, 0.102), true);
        let actual = collector.finish().unwrap();
        assert_eq!((actual.windows, actual.valid_ms), (21, 2100));
        close(actual.velocity_rms, 0.0);
        close(actual.tracking_peak, 0.1);
    }

    #[test]
    fn collector_ignores_nonfinite_values_on_any_axis_and_preserves_valid_aggregates() {
        assert_interrupt_breaks_window(2_060_000, true, true, false);
        assert_interrupt_breaks_window(2_060_000, true, false, true);
        let mut collector = steady(20, 100_000, 0.02, 0.02);
        let before = serde_json::to_value(collector.finish()).unwrap();
        collector.observe(2_050_000, [f64::NEG_INFINITY; 7], [f64::NAN; 7], true);
        assert_eq!(serde_json::to_value(collector.finish()).unwrap(), before);
        for (axis, initial) in [(7, [0.0; 7]), (0, [f64::NAN; 7])] {
            let mut invalid = Collector::new(axis, initial);
            for sample in 0..=25 {
                invalid.observe(sample * 100_000, [0.0; 7], [0.0; 7], true);
            }
            assert!(invalid.finish().is_none());
        }
    }

    #[test]
    fn collector_tracks_selected_position_error_and_other_axis_initial_drift() {
        let initial = [0.5, -0.4, 0.3, -0.2, 0.1, 0.0, 0.7];
        let mut collector = Collector::new(3, initial);
        for sample in 0..=20 {
            let mut reference = initial;
            let mut measured = initial;
            reference[3] += sample as f64 * 0.002;
            measured[3] = reference[3] - 0.004;
            measured[6] += sample as f64 * 0.0005;
            // The other reference follows its measured drift; compare with initial instead.
            reference[6] = measured[6];
            collector.observe(sample * 100_000, reference, measured, true);
        }
        let actual = collector.finish().unwrap();
        close(actual.velocity_rms, 0.0);
        close(actual.tracking_rms, 0.004);
        close(actual.tracking_peak, 0.004);
        close(actual.other_peak, 0.01);
    }

    #[test]
    fn smooth_and_quantized_tracking_outperform_stall_then_catch_up() {
        let mut smooth = Collector::new(3, [0.0; 7]);
        let mut quantized = Collector::new(3, [0.0; 7]);
        let mut stalled = Collector::new(3, [0.0; 7]);
        for sample in 0..=240 {
            let reference = sample as f64 * 0.0002;
            smooth.observe(
                sample * 10_000,
                point(3, reference),
                point(3, reference * 0.9),
                true,
            );
            quantized.observe(
                sample * 10_000,
                point(3, reference),
                point(3, (reference / 0.000125).round() * 0.000125),
                true,
            );
            stalled.observe(
                sample * 10_000,
                point(3, reference),
                point(3, (sample / 20) as f64 * 0.004),
                true,
            );
        }
        let smooth = smooth.finish().unwrap();
        let quantized = quantized.finish().unwrap();
        let stalled = stalled.finish().unwrap();
        close(smooth.velocity_rms, 0.002);
        close(quantized.velocity_rms, 0.0);
        assert!(quantized.tracking_peak > 0.00001 && quantized.tracking_peak < 0.000063);
        close(stalled.velocity_rms, 0.02);
        assert!(smooth.velocity_rms < stalled.velocity_rms);
        close(stalled.tracking_peak, 0.0038);
    }

    fn trial(index: usize, velocity_rms: f64) -> Trial {
        Trial {
            index,
            phase: if index == 0 { "baseline" } else { "candidate" }.into(),
            config: ServoTuning::default(),
            metrics: Some(Metrics {
                windows: 24,
                valid_ms: 2400,
                velocity_rms,
                tracking_rms: 0.005,
                tracking_peak: 0.01,
                other_peak: 0.002,
            }),
        }
    }

    #[test]
    fn recommendation_returns_trial_index_with_best_velocity_error_and_stable_ties() {
        let baseline = trial(0, 0.004);
        assert_eq!(
            recommend(
                &baseline,
                &[trial(8, 0.003), trial(2, 0.002), trial(9, 0.002)]
            ),
            Some(2)
        );
        assert_eq!(
            recommend(&baseline, &[trial(8, 0.003), trial(2, 0.003)]),
            Some(8)
        );
    }

    #[test]
    fn recommendation_rejects_no_change_small_improvement_and_tiny_baselines() {
        let baseline = trial(0, 0.004);
        for rms in [0.004, 0.005, 0.003400001] {
            assert_eq!(recommend(&baseline, &[trial(1, rms)]), None);
        }
        assert_eq!(recommend(&trial(0, 0.0005), &[trial(1, 0.0)]), None);
        assert_eq!(recommend(&trial(0, 0.00051), &[trial(1, 0.00042)]), None);
        assert_eq!(recommend(&trial(0, 0.0008), &[trial(1, 0.0006)]), Some(1));
    }

    #[test]
    fn recommendation_accepts_threshold_equalities_with_only_roundoff_tolerance() {
        assert_eq!(recommend(&trial(0, 0.004), &[trial(1, 0.0034)]), Some(1));
        assert_eq!(recommend(&trial(0, 0.01), &[trial(1, 0.0085)]), Some(1));
        assert_eq!(recommend(&trial(0, 0.0006), &[trial(1, 0.0005)]), Some(1));
        assert_eq!(recommend(&trial(0, 0.0006), &[trial(1, 0.000500001)]), None);
        let above = f64::from_bits(0.0005f64.to_bits() + 1);
        assert_eq!(recommend(&trial(0, above), &[trial(1, 0.0)]), Some(1));
    }

    #[test]
    fn recommendation_gates_position_and_other_joint_regressions() {
        let baseline = trial(0, 0.004);
        let mut candidate = trial(4, 0.002);
        candidate.metrics.as_mut().unwrap().tracking_peak = 0.014000001;
        assert_eq!(recommend(&baseline, &[candidate.clone()]), None);
        candidate.metrics.as_mut().unwrap().tracking_peak = 0.01 * 1.1 + 0.003;
        candidate.metrics.as_mut().unwrap().other_peak = 0.007000001;
        assert_eq!(recommend(&baseline, &[candidate.clone()]), None);
        candidate.metrics.as_mut().unwrap().other_peak = 0.007;
        assert_eq!(recommend(&baseline, &[candidate]), Some(4));
    }

    #[test]
    fn recommendation_rejects_missing_short_nonfinite_and_negative_metrics() {
        let baseline = trial(0, 0.004);
        let good = trial(1, 0.002);
        let mut missing = good.clone();
        missing.metrics = None;
        assert_eq!(recommend(&baseline, &[missing.clone()]), None);
        assert_eq!(recommend(&missing, std::slice::from_ref(&good)), None);
        for field in [
            "velocity_rms",
            "tracking_rms",
            "tracking_peak",
            "other_peak",
            "windows",
            "valid_ms",
        ] {
            for value in [f64::NAN, f64::INFINITY, -0.1] {
                let mut invalid = good.clone();
                invalid.index = 9;
                let metrics = invalid.metrics.as_mut().unwrap();
                match field {
                    "velocity_rms" => metrics.velocity_rms = value,
                    "tracking_rms" => metrics.tracking_rms = value,
                    "tracking_peak" => metrics.tracking_peak = value,
                    "other_peak" => metrics.other_peak = value,
                    "windows" => metrics.windows = 11,
                    _ => metrics.valid_ms = 1999,
                }
                assert_eq!(
                    recommend(&baseline, &[invalid.clone()]),
                    None,
                    "{field}={value}"
                );
                assert_eq!(
                    recommend(&baseline, &[invalid.clone(), good.clone()]),
                    Some(1),
                    "skip invalid candidate {field}"
                );
                if field != "velocity_rms" {
                    invalid.metrics.as_mut().unwrap().velocity_rms = 0.004;
                }
                assert_eq!(
                    recommend(&invalid, std::slice::from_ref(&good)),
                    None,
                    "invalid baseline {field}"
                );
            }
        }
    }

    #[test]
    fn reduced_feedforward_is_scored_against_the_original_motion_reference() {
        let mut candidate = trial(1, 0.0);
        candidate.config.joints[3].velocity_ff = 0.75;
        candidate.metrics = steady(24, 100_000, 0.02, 0.015).finish();
        close(candidate.metrics.as_ref().unwrap().velocity_rms, 0.005);
        assert_eq!(recommend(&trial(0, 0.002), &[candidate]), None);
    }

    #[test]
    fn confirm_returns_the_smaller_of_two_measured_improvements() {
        let initial = trial(0, 0.004);
        let check = trial(7, 0.005);
        let candidate = trial(3, 0.002);
        let validation = trial(8, 0.0035);
        close(
            confirm(&initial, &check, &candidate, &validation).unwrap(),
            0.3,
        );
    }

    #[test]
    fn confirm_rejects_baseline_drift_and_either_failed_repeat() {
        let initial = trial(0, 0.004);
        let candidate = trial(3, 0.002);
        let validation = trial(8, 0.002);
        assert!(confirm(&initial, &trial(7, 0.0055), &candidate, &validation).is_some());
        assert!(confirm(&initial, &trial(7, 0.005500001), &candidate, &validation).is_none());
        assert!(confirm(&initial, &trial(7, 0.002499999), &candidate, &validation).is_none());
        assert!(confirm(&initial, &initial, &trial(3, 0.004), &validation).is_none());
        assert!(confirm(&initial, &initial, &candidate, &trial(8, 0.004)).is_none());
        let mut missing = validation.clone();
        missing.metrics = None;
        assert!(confirm(&initial, &initial, &candidate, &missing).is_none());
        let mut regression = validation;
        regression.metrics.as_mut().unwrap().other_peak = 0.1;
        assert!(confirm(&initial, &initial, &candidate, &regression).is_none());
        let mut invalid_initial = initial.clone();
        invalid_initial.metrics.as_mut().unwrap().tracking_rms = f64::NAN;
        assert!(confirm(&invalid_initial, &initial, &candidate, &trial(8, 0.002)).is_none());
        // For these values both drift differences equal the represented limit.
        assert!(
            confirm(
                &trial(0, 0.002),
                &trial(7, 0.003),
                &trial(3, 0.001),
                &trial(8, 0.0005)
            )
            .is_some()
        );
        assert!(
            confirm(
                &trial(0, 0.002),
                &trial(7, 0.001),
                &trial(3, 0.001),
                &trial(8, 0.0005)
            )
            .is_some()
        );
    }

    #[test]
    fn reports_and_status_round_trip_with_the_shared_json_field_names() {
        let baseline = ServoTuning::default();
        let report = Report {
            id: "cal-1".into(),
            request_id: "request-1".into(),
            created_at_ms: 1795305600000,
            side: "left".into(),
            port: "offline-test".into(),
            settings: settings(),
            center_reference: [0.0; 7],
            center_measured: [0.1; 7],
            tcp: Some(crate::kinematics::Pose {
                translation: [0.0, 0.0, 0.12],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            baseline,
            recommended: Some(baseline),
            trials: vec![trial(0, 0.004), trial(1, 0.002)],
            outcome: "improved".into(),
            improvement: Some(0.5),
            message: "离线重复验证".into(),
        };
        let expected = serde_json::to_value(&report).unwrap();
        assert_eq!(expected["request_id"], json!("request-1"));
        assert_eq!(expected["settings"]["axis"], json!(3));
        assert_eq!(expected["trials"][0]["metrics"]["valid_ms"], json!(2400));
        assert_eq!(expected["tcp"]["rotation"], json!([0.0, 0.0, 0.0, 1.0]));
        let decoded: Report = serde_json::from_value(expected.clone()).unwrap();
        assert_eq!(serde_json::to_value(decoded).unwrap(), expected);
        for outcome in [
            "improved",
            "unchanged",
            "insufficient",
            "cancelled",
            "failed",
        ] {
            let mut current = report.clone();
            current.outcome = outcome.into();
            current.recommended = None;
            current.improvement = None;
            current.tcp = None;
            let encoded = serde_json::to_value(current).unwrap();
            assert_eq!(encoded["recommended"], Value::Null);
            let decoded: Report = serde_json::from_value(encoded.clone()).unwrap();
            assert_eq!(serde_json::to_value(decoded).unwrap(), encoded);
        }
        for state in ["running", "stopping", "completed", "cancelled", "failed"] {
            for stage in [
                "baseline",
                "candidate",
                "baseline_check",
                "validation",
                "restoring",
                "done",
            ] {
                let status = Status {
                    id: "cal-1".into(),
                    request_id: "request-1".into(),
                    state: state.into(),
                    stage: stage.into(),
                    axis: 3,
                    trial_index: 2,
                    trial_count: 4,
                    leg_index: 1,
                    leg_count: 6,
                    progress: 0.5,
                    report: Some(report.clone()),
                };
                let encoded = serde_json::to_value(status).unwrap();
                let decoded: Status = serde_json::from_value(encoded.clone()).unwrap();
                assert_eq!(serde_json::to_value(decoded).unwrap(), encoded);
            }
        }
    }
}
