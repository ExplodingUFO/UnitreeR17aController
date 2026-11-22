//! Offline low-speed calibration candidates, trajectories, and measurements.

use crate::tuning::ServoTuning;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::sync::Arc;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub axis: Option<usize>,
    pub excursion: f64,
    pub speed: f64,
    pub max_friction: f64,
    pub cartesian: Option<Arc<crate::calibration_path::CartesianProgram>>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            axis: None,
            excursion: 16.0f64.to_radians(),
            speed: 2.0f64.to_radians(),
            max_friction: 0.2,
            cartesian: None,
        }
    }
}

fn validate_settings(settings: &Settings) -> Result<(), String> {
    if settings.axis.is_some_and(|axis| axis >= 7) {
        return Err("校准关节必须为 0..6".into());
    }
    for (name, value) in [("excursion", settings.excursion), ("speed", settings.speed)] {
        if !value.is_finite() || value <= 0.0 {
            return Err(format!("校准 {name} 必须为正数"));
        }
    }
    if !settings.max_friction.is_finite()
        || settings.max_friction < 0.0
        || settings.max_friction > 2.0
    {
        return Err("摩擦试验参数无效".into());
    }
    Ok(())
}

pub fn validate(settings: &Settings, baseline: &ServoTuning) -> Result<(), String> {
    validate_settings(settings)?;
    if let Some(path) = &settings.cartesian
        && (path.origin.iter().any(|v| !v.is_finite())
            || !path.amplitude.is_finite()
            || path.amplitude < 0.0
            || !path.speed.is_finite()
            || path.speed <= 0.0
            || path.segments.iter().any(|s| {
                s.axis > 2 || !s.offset.is_finite() || s.offset.abs() > path.amplitude + 1e-9
            }))
    {
        return Err("末端测试轨迹参数无效".into());
    }
    baseline.validate()
}

pub fn candidates(
    settings: &Settings,
    axis: usize,
    baseline: ServoTuning,
) -> Result<Vec<ServoTuning>, String> {
    validate(settings, &baseline)?;
    if axis >= 7 {
        return Err("校准关节必须为 0..6".into());
    }
    let mut result = Vec::with_capacity(4);
    result.push(baseline);
    if settings.max_friction > 0.0 {
        for coulomb in [settings.max_friction / 2.0, settings.max_friction] {
            let mut candidate = baseline;
            if !baseline.friction_enabled {
                // The global switch must not activate dormant coefficients on
                // any joint, including the measured joint's viscous term.
                for joint in &mut candidate.joints {
                    joint.coulomb = 0.0;
                    joint.viscous = 0.0;
                }
            }
            candidate.friction_enabled = true;
            candidate.joints[axis].coulomb = coulomb;
            candidate.joints[axis].max_friction = settings.max_friction;
            if !result.contains(&candidate) {
                result.push(candidate);
            }
        }
    }
    let mut feedforward = baseline;
    feedforward.joints[axis].velocity_ff = 0.75;
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

pub fn legs(settings: &Settings, axis: usize, center: [f64; 7]) -> Vec<Leg> {
    if validate_settings(settings).is_err()
        || axis >= 7
        || center.iter().any(|value| !value.is_finite())
    {
        return Vec::new();
    }
    let mut result = Vec::with_capacity(6);
    for speed in [settings.speed, settings.speed * 2.0] {
        for offset in [settings.excursion / 2.0, -settings.excursion / 2.0, 0.0] {
            let mut target = center;
            if offset != 0.0 {
                target[axis] += offset;
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
    #[serde(default)]
    pub axis: Option<usize>,
    pub config: ServoTuning,
    pub metrics: Option<Metrics>,
    #[serde(default = "joint_space")]
    pub space: String,
    #[serde(default)]
    pub metrics_by_joint: Option<Vec<Option<Metrics>>>,
}

fn joint_space() -> String {
    "joint".into()
}

fn metrics_valid(metrics: &Metrics) -> bool {
    metrics.windows > 0
        && metrics.valid_ms > 0
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

fn normalized_error(baseline: f64, candidate: f64) -> f64 {
    if baseline > 0.0 {
        candidate / baseline
    } else if candidate == 0.0 {
        1.0
    } else {
        f64::INFINITY
    }
}

/// Lower is better; the baseline scores 1.0. Each RMS is normalized in its
/// own units, with 60% velocity and 40% position tracking. An unchanged exact
/// zero scores 1.0 for that component; a regression from zero scores infinity.
/// Missing/invalid measurements score infinity, with no minimum duration or
/// improvement beyond the existence of a measured position window.
pub fn score(baseline: &Metrics, candidate: &Metrics) -> f64 {
    if !metrics_valid(baseline) || !metrics_valid(candidate) {
        return f64::INFINITY;
    }
    0.6 * normalized_error(baseline.velocity_rms, candidate.velocity_rms)
        + 0.4 * normalized_error(baseline.tracking_rms, candidate.tracking_rms)
}

pub fn recommend(baseline: &Trial, candidates: &[Trial]) -> Option<usize> {
    let baseline = valid_metrics(baseline)?;
    let mut best: Option<(usize, f64)> = None;
    for candidate in candidates {
        let Some(metrics) = valid_metrics(candidate) else {
            continue;
        };
        let error = score(baseline, metrics);
        if error < 1.0 && best.is_none_or(|(_, best_error)| error < best_error) {
            best = Some((candidate.index, error));
        }
    }
    best.map(|(index, _)| index)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Point {
    pub t_ms: u32,
    pub trial: usize,
    pub axis: usize,
    pub reference: [f64; 7],
    pub command: Option<[f64; 7]>,
    pub feedback: [f64; 7],
    pub velocity: [f64; 7],
    pub feedback_velocity: [f64; 7],
    pub feedback_age_ms: [u16; 7],
    pub command_at_ms: Option<u32>,
    #[serde(default = "joint_space")]
    pub space: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Choice {
    pub axis: usize,
    pub baseline_index: usize,
    pub candidate_index: usize,
    pub after_index: Option<usize>,
    pub velocity_change: f64,
    pub tracking_change: f64,
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
    #[serde(default)]
    pub choices: Vec<Choice>,
    #[serde(default)]
    pub sample_count: usize,
    #[serde(default)]
    pub curves: Arc<Vec<Point>>,
}

impl Report {
    /// Metadata for live snapshots/history; the recorded sample count survives.
    pub fn summary(&self) -> Self {
        Self {
            curves: Arc::default(),
            ..self.clone()
        }
    }

    /// Select real samples without interpolation. Keep each segment's endpoints
    /// and per-bucket extrema of all joints' position/velocity tracking errors.
    /// A zero budget returns all samples with an O(1) clone of their Arc.
    pub fn for_plot(&self, max_points: usize) -> Self {
        if max_points == 0 || self.curves.len() <= max_points {
            return self.clone();
        }
        let mut segments = Vec::new();
        let mut start = 0;
        for end in 1..=self.curves.len() {
            if end == self.curves.len()
                || self.curves[end].trial != self.curves[start].trial
                || self.curves[end].axis != self.curves[start].axis
            {
                segments.push((start, end));
                start = end;
            }
        }
        let endpoints: usize = segments
            .iter()
            .map(|(start, end)| (end - start).min(2))
            .sum();
        let mut interiors_left = self.curves.len() - endpoints;
        let mut extras_left = max_points.saturating_sub(endpoints).min(interiors_left);
        let mut curves = Vec::with_capacity(endpoints + extras_left);
        for (start, end) in segments {
            let interior = (end - start).saturating_sub(2);
            let extra = if interiors_left == 0 {
                0
            } else {
                (extras_left as u128 * interior as u128 / interiors_left as u128) as usize
            };
            interiors_left -= interior;
            extras_left -= extra;
            curves.push(self.curves[start].clone());
            for index in plot_indices(&self.curves, start, end, extra) {
                curves.push(self.curves[index].clone());
            }
            if end > start + 1 {
                curves.push(self.curves[end - 1].clone());
            }
        }
        Self {
            curves: Arc::new(curves),
            ..self.clone()
        }
    }
}

fn plot_indices(points: &[Point], start: usize, end: usize, extra: usize) -> Vec<usize> {
    let mut selected = BTreeSet::new();
    let interior = (end - start).saturating_sub(2);
    let buckets = extra / 28;
    for bucket in 0..buckets {
        let from = start + 1 + bucket * interior / buckets;
        let to = start + 1 + (bucket + 1) * interior / buckets;
        for axis in 0..7 {
            for velocity in [false, true] {
                let value = |index: usize| {
                    let p = &points[index];
                    if velocity {
                        p.feedback_velocity[axis] - p.velocity[axis]
                    } else {
                        p.feedback[axis] - p.command.as_ref().unwrap_or(&p.reference)[axis]
                    }
                };
                if let Some(min) = (from..to).min_by(|a, b| value(*a).total_cmp(&value(*b))) {
                    selected.insert(min);
                }
                if let Some(max) = (from..to).max_by(|a, b| value(*a).total_cmp(&value(*b))) {
                    selected.insert(max);
                }
            }
        }
    }
    for slot in 1..=extra {
        if selected.len() == extra {
            break;
        }
        let offset = (slot as u128 * (end - start - 1) as u128 / (extra + 1) as u128) as usize;
        selected.insert(start + offset);
    }
    selected.into_iter().collect()
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub live: Option<Point>,
    #[serde(default = "joint_space")]
    pub space: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn close(actual: f64, expected: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() < 1e-10,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn defaults_select_all_axes_with_a_small_six_leg_programme_for_each() {
        let settings: Settings = serde_json::from_str("{}").unwrap();
        assert_eq!(settings.axis, None);
        close(settings.excursion, 16.0f64.to_radians());
        close(settings.speed, 2.0f64.to_radians());
        close(settings.max_friction, 0.2);
        assert!(validate(&settings, &ServoTuning::default()).is_ok());
        let center = [0.5, -0.4, 0.3, -0.2, 0.1, -0.0, 0.7];
        for axis in 0..7 {
            let programme = legs(&settings, axis, center);
            assert_eq!(programme.len(), 6);
            for (index, leg) in programme.iter().enumerate() {
                let offset = [8.0f64.to_radians(), -8.0f64.to_radians(), 0.0][index % 3];
                close(leg.speed, (1 + index / 3) as f64 * 2.0f64.to_radians());
                close(leg.target[axis], center[axis] + offset);
                for other in (0..7).filter(|other| *other != axis) {
                    assert_eq!(leg.target[other].to_bits(), center[other].to_bits());
                }
                if index % 3 == 2 {
                    assert_eq!(leg.target.map(f64::to_bits), center.map(f64::to_bits));
                }
            }
        }
        let selected: Settings = serde_json::from_str(r#"{"axis":3}"#).unwrap();
        assert_eq!(selected.axis, Some(3));
        close(selected.excursion, settings.excursion);
    }

    #[test]
    fn explicit_small_ranges_move_only_the_requested_axis_relative_to_its_center() {
        let center = [-0.7, 0.6, -0.5, 0.4, -0.3, 0.2, -0.1];
        for axis in 0..7 {
            let settings = Settings {
                axis: Some(axis),
                excursion: 2.0f64.to_radians(),
                speed: 0.5f64.to_radians(),
                ..Settings::default()
            };
            assert!(validate(&settings, &ServoTuning::default()).is_ok());
            let programme = legs(&settings, axis, center);
            assert_eq!(programme.len(), 6);
            for (index, leg) in programme.iter().enumerate() {
                close(
                    leg.target[axis] - center[axis],
                    [1.0f64.to_radians(), -1.0f64.to_radians(), 0.0][index % 3],
                );
                close(leg.speed, settings.speed * (1 + index / 3) as f64);
                for other in (0..7).filter(|other| *other != axis) {
                    assert_eq!(leg.target[other], center[other]);
                }
            }
        }
    }

    #[test]
    fn settings_and_explicit_axes_reject_invalid_inputs_without_panicking() {
        let baseline = ServoTuning::default();
        for axis in [7, usize::MAX] {
            assert!(
                validate(
                    &Settings {
                        axis: Some(axis),
                        ..Settings::default()
                    },
                    &baseline
                )
                .is_err()
            );
            assert!(candidates(&Settings::default(), axis, baseline).is_err());
            assert!(legs(&Settings::default(), axis, [0.0; 7]).is_empty());
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            for field in 0..3 {
                let mut settings = Settings::default();
                match field {
                    0 => settings.excursion = invalid,
                    1 => settings.speed = invalid,
                    _ => settings.max_friction = invalid,
                }
                assert!(validate(&settings, &baseline).is_err());
                assert!(candidates(&settings, 0, baseline).is_err());
                assert!(legs(&settings, 0, [0.0; 7]).is_empty());
            }
        }
        let mut invalid_baseline = baseline;
        invalid_baseline.joints[0].kd = 31.0;
        assert!(validate(&Settings::default(), &invalid_baseline).is_err());
        assert!(legs(&Settings::default(), 0, [f64::NAN; 7]).is_empty());
    }

    #[test]
    fn default_candidates_are_independent_fc_point_one_point_two_and_ff_point_seven_five() {
        let mut baseline = ServoTuning::default();
        baseline.joints[3].velocity_ff = 0.8;
        for axis in 0..7 {
            let actual = candidates(&Settings::default(), axis, baseline).unwrap();
            assert_eq!(actual.len(), 4);
            assert_eq!(actual[0], baseline);
            for (candidate, coulomb) in actual[1..3].iter().zip([0.1, 0.2]) {
                let mut expected = baseline;
                expected.friction_enabled = true;
                expected.joints[axis].coulomb = coulomb;
                expected.joints[axis].max_friction = 0.2;
                assert_eq!(*candidate, expected);
                assert!(candidate.validate().is_ok());
            }
            let mut expected = baseline;
            expected.joints[axis].velocity_ff = 0.75;
            assert_eq!(actual[3], expected);
            assert_eq!(baseline.joints[3].velocity_ff, 0.8);
        }
    }

    #[test]
    fn dormant_friction_is_cleared_before_enabling_only_the_measured_axis() {
        let mut baseline = ServoTuning::default();
        for (axis, joint) in baseline.joints.iter_mut().enumerate() {
            joint.coulomb = 0.2 + axis as f64 * 0.05;
            joint.viscous = 0.3 + axis as f64 * 0.1;
            joint.kd += axis as f64;
        }
        assert!(validate(&Settings::default(), &baseline).is_ok());
        for axis in 0..7 {
            let actual = candidates(&Settings::default(), axis, baseline).unwrap();
            assert_eq!(actual[0], baseline);
            for candidate in &actual[1..3] {
                let applied = candidate.apply([0.02; 7], [0.0; 7]).unwrap();
                assert!(applied.friction_requested[axis] > 0.0);
                for other in 0..7 {
                    assert_eq!(candidate.joints[other].viscous, 0.0);
                    assert_eq!(candidate.joints[other].kd, baseline.joints[other].kd);
                    if other != axis {
                        assert_eq!(candidate.joints[other].coulomb, 0.0);
                        assert_eq!(applied.friction_requested[other], 0.0);
                    }
                }
            }
            let mut expected_ff = baseline;
            expected_ff.joints[axis].velocity_ff = 0.75;
            assert_eq!(actual[3], expected_ff);
        }
    }

    #[test]
    fn enabled_friction_preserves_other_axes_and_duplicate_candidates_are_omitted() {
        let mut baseline = ServoTuning {
            friction_enabled: true,
            ..ServoTuning::default()
        };
        baseline.joints[0].coulomb = 0.3;
        baseline.joints[2].viscous = 0.4;
        baseline.joints[3].coulomb = 0.1;
        baseline.joints[3].max_friction = 0.2;
        let actual = candidates(&Settings::default(), 3, baseline).unwrap();
        assert_eq!(actual.len(), 3);
        assert_eq!(actual[0], baseline);
        assert_eq!(actual[1].joints[3].coulomb, 0.2);
        for candidate in &actual {
            for axis in (0..7).filter(|axis| *axis != 3) {
                assert_eq!(candidate.joints[axis], baseline.joints[axis]);
            }
        }
        baseline.joints[3].velocity_ff = 0.75;
        assert_eq!(
            candidates(
                &Settings {
                    max_friction: 0.0,
                    ..Settings::default()
                },
                3,
                baseline
            )
            .unwrap(),
            vec![baseline]
        );
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
    fn collector_accepts_one_real_position_window_without_a_minimum_trial_duration() {
        let mut collector = Collector::new(3, [0.0; 7]);
        assert!(collector.finish().is_none());
        collector.observe(0, point(3, 0.0), point(3, 0.0), true);
        collector.observe(99_999, point(3, 0.002), point(3, 0.0015), true);
        assert!(collector.finish().is_none());
        collector.observe(140_000, point(3, 0.0028), point(3, 0.0021), true);
        let actual = collector.finish().unwrap();
        assert_eq!((actual.windows, actual.valid_ms), (1, 140));
        close(actual.velocity_rms, 0.005);
        let sparse = steady(1, 200_000, 0.02, 0.02).finish().unwrap();
        assert_eq!((sparse.windows, sparse.valid_ms), (1, 200));
        close(sparse.velocity_rms, 0.0);
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

    fn trial(index: usize, velocity_rms: f64, tracking_rms: f64) -> Trial {
        Trial {
            index,
            space: "joint".into(),
            metrics_by_joint: None,
            phase: if index == 0 { "baseline" } else { "candidate" }.into(),
            axis: Some(3),
            config: ServoTuning::default(),
            metrics: Some(Metrics {
                windows: 1,
                valid_ms: 100,
                velocity_rms,
                tracking_rms,
                tracking_peak: tracking_rms * 2.0,
                other_peak: 0.002,
            }),
        }
    }

    #[test]
    fn score_normalizes_each_rms_before_combining_velocity_and_tracking() {
        let baseline = trial(0, 0.004, 0.005).metrics.unwrap();
        let candidate = trial(1, 0.002, 0.004).metrics.unwrap();
        close(score(&baseline, &baseline), 1.0);
        close(score(&baseline, &candidate), 0.6 * 0.5 + 0.4 * 0.8);
        let mut rescaled_baseline = baseline.clone();
        let mut rescaled_candidate = candidate.clone();
        rescaled_baseline.velocity_rms *= 1000.0;
        rescaled_candidate.velocity_rms *= 1000.0;
        rescaled_baseline.tracking_rms *= 0.001;
        rescaled_candidate.tracking_rms *= 0.001;
        close(
            score(&rescaled_baseline, &rescaled_candidate),
            score(&baseline, &candidate),
        );
    }

    #[test]
    fn measured_report_values_recommend_fc_point_two_despite_small_velocity_improvement() {
        // Measured RMS values exercise the tradeoff: Fc=0.1 wins on velocity,
        // while Fc=0.2 wins overall with about 38% less position error.
        let mut baseline = trial(0, 0.010828694820050612, 0.005910435712582818);
        let configs = candidates(&Settings::default(), 3, baseline.config).unwrap();
        baseline.metrics.as_mut().unwrap().windows = 184;
        baseline.metrics.as_mut().unwrap().valid_ms = 20_599;
        let mut low_friction = trial(1, 0.010345783198140366, 0.004638951499589994);
        low_friction.config = configs[1];
        let mut high_friction = trial(2, 0.010688245119382338, 0.0036441167515515152);
        high_friction.config = configs[2];
        let mut feedforward = trial(3, 0.010893930573407125, 0.007077228086301605);
        feedforward.config = configs[3];
        let before = baseline.metrics.as_ref().unwrap();
        let after = high_friction.metrics.as_ref().unwrap();
        assert!((0.38..0.39).contains(&(1.0 - after.tracking_rms / before.tracking_rms)));
        assert!((0.01..0.02).contains(&(1.0 - after.velocity_rms / before.velocity_rms)));
        assert!(score(before, after) < score(before, low_friction.metrics.as_ref().unwrap()));
        assert_eq!(high_friction.config.joints[3].coulomb, 0.2);
        assert_eq!(
            recommend(&baseline, &[low_friction, high_friction, feedforward]),
            Some(2)
        );
    }

    #[test]
    fn recommendation_accepts_small_improvements_and_weighted_tradeoffs_without_peak_gates() {
        let baseline = trial(0, 1e-9, 2e-9);
        let mut candidate = trial(8, 0.99e-9, 2e-9);
        candidate.metrics.as_mut().unwrap().tracking_peak = 0.1;
        candidate.metrics.as_mut().unwrap().other_peak = 0.2;
        assert_eq!(recommend(&baseline, &[candidate]), Some(8));
        let candidate = trial(9, 1.1e-9, 1e-9);
        assert_eq!(recommend(&baseline, &[candidate]), Some(9));
    }

    #[test]
    fn recommendation_returns_real_trial_indices_with_stable_ties_and_no_change_is_none() {
        let baseline = trial(0, 0.004, 0.005);
        assert_eq!(
            recommend(
                &baseline,
                &[
                    trial(8, 0.003, 0.005),
                    trial(2, 0.002, 0.005),
                    trial(9, 0.002, 0.005)
                ]
            ),
            Some(2)
        );
        assert_eq!(
            recommend(&baseline, &[trial(8, 0.003, 0.005), trial(2, 0.003, 0.005)]),
            Some(8)
        );
        assert_eq!(
            recommend(&baseline, &[trial(1, 0.004, 0.005), trial(2, 0.005, 0.006)]),
            None
        );
        assert_eq!(recommend(&baseline, &[]), None);
    }

    #[test]
    fn zero_error_components_allow_improvement_in_the_other_component() {
        for (baseline, candidate) in [
            (trial(0, 0.0, 0.005), trial(1, 0.0, 0.004)),
            (trial(0, 0.004, 0.0), trial(1, 0.003, 0.0)),
        ] {
            close(
                score(
                    baseline.metrics.as_ref().unwrap(),
                    baseline.metrics.as_ref().unwrap(),
                ),
                1.0,
            );
            assert_eq!(recommend(&baseline, &[candidate]), Some(1));
        }
        let exact = trial(0, 0.0, 0.0);
        assert_eq!(recommend(&exact, &[trial(1, 0.0, 0.0)]), None);
        assert!(
            score(
                exact.metrics.as_ref().unwrap(),
                trial(1, 0.001, 0.0).metrics.as_ref().unwrap()
            )
            .is_infinite()
        );
    }

    #[test]
    fn recommendation_skips_missing_empty_and_invalid_measurements() {
        let baseline = trial(0, 0.004, 0.005);
        let good = trial(1, 0.002, 0.004);
        let mut missing = good.clone();
        missing.metrics = None;
        assert_eq!(recommend(&missing, std::slice::from_ref(&good)), None);
        assert_eq!(recommend(&baseline, &[missing]), None);
        for field in 0..6 {
            for invalid in [f64::NAN, f64::INFINITY, -0.1] {
                let mut bad = good.clone();
                let metrics = bad.metrics.as_mut().unwrap();
                match field {
                    0 => metrics.velocity_rms = invalid,
                    1 => metrics.tracking_rms = invalid,
                    2 => metrics.tracking_peak = invalid,
                    3 => metrics.other_peak = invalid,
                    4 => metrics.windows = 0,
                    _ => metrics.valid_ms = 0,
                }
                assert!(score(baseline.metrics.as_ref().unwrap(), metrics).is_infinite());
                assert_eq!(recommend(&baseline, &[bad.clone(), good.clone()]), Some(1));
                assert_eq!(recommend(&bad, std::slice::from_ref(&good)), None);
            }
        }
    }

    #[test]
    fn reduced_feedforward_is_scored_against_the_original_motion_reference() {
        let mut candidate = trial(1, 0.0, 0.0);
        candidate.config.joints[3].velocity_ff = 0.75;
        candidate.metrics = steady(24, 100_000, 0.02, 0.015).finish();
        close(candidate.metrics.as_ref().unwrap().velocity_rms, 0.005);
        assert_eq!(recommend(&trial(0, 0.002, 0.001), &[candidate]), None);
    }

    fn curve_point(t_ms: u32, trial: usize, axis: usize) -> Point {
        let reference = std::array::from_fn(|joint| t_ms as f64 * 0.00002 + joint as f64 * 0.01);
        Point {
            space: "joint".into(),
            t_ms,
            trial,
            axis,
            reference,
            command: (!t_ms.is_multiple_of(3)).then_some(reference.map(|q| q - 0.0001)),
            feedback: reference.map(|q| q - 0.001),
            velocity: [0.02; 7],
            feedback_velocity: [if t_ms.is_multiple_of(2) { -1.0 } else { 1.0 }; 7],
            feedback_age_ms: [(t_ms % 100) as u16; 7],
            command_at_ms: t_ms.checked_sub(4),
        }
    }

    fn report(curves: Vec<Point>) -> Report {
        Report {
            id: "cal-test".into(),
            request_id: "request-test".into(),
            created_at_ms: 1_795_305_600_000,
            side: "left".into(),
            port: "offline-test".into(),
            settings: Settings {
                axis: Some(3),
                ..Settings::default()
            },
            center_reference: [0.0; 7],
            center_measured: [0.1; 7],
            tcp: Some(crate::kinematics::Pose {
                translation: [0.0, 0.0, 0.12],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            baseline: ServoTuning::default(),
            recommended: None,
            trials: vec![trial(0, 0.004, 0.005), trial(1, 0.004, 0.005)],
            outcome: "unchanged".into(),
            improvement: None,
            message: "测量完成，当前参数已保持".into(),
            choices: vec![Choice {
                axis: 3,
                baseline_index: 0,
                candidate_index: 1,
                after_index: Some(2),
                velocity_change: 0.01,
                tracking_change: 0.38,
            }],
            sample_count: curves.len(),
            curves: Arc::new(curves),
        }
    }

    #[test]
    fn report_clones_and_unlimited_plots_share_the_curve_allocation() {
        let report = report((0..100).map(|i| curve_point(i * 20, 0, 3)).collect());
        let clone = report.clone();
        let full = report.for_plot(0);
        let ample_budget = report.for_plot(100);
        assert!(Arc::ptr_eq(&report.curves, &clone.curves));
        assert!(Arc::ptr_eq(&report.curves, &full.curves));
        assert!(Arc::ptr_eq(&report.curves, &ample_budget.curves));
    }

    #[test]
    fn summary_has_no_curves_and_preserves_counts_choices_and_completed_measurement() {
        let report = report((0..5).map(|i| curve_point(i * 20, 0, 3)).collect());
        let summary = report.summary();
        assert!(summary.curves.is_empty());
        assert_eq!(summary.sample_count, 5);
        assert_eq!(report.curves.len(), 5);
        let mut expected = serde_json::to_value(&report).unwrap();
        expected["curves"] = json!([]);
        assert_eq!(serde_json::to_value(&summary).unwrap(), expected);
        assert_eq!(summary.outcome, "unchanged");
        assert!(summary.recommended.is_none());
    }

    #[test]
    fn plot_decimation_preserves_real_samples_segment_endpoints_and_total_count() {
        assert!(report(Vec::new()).for_plot(1).curves.is_empty());
        let mut points = Vec::new();
        for (trial, axis, length) in [(5, 0, 5), (8, 1, 19), (8, 2, 2), (5, 0, 1)] {
            for _ in 0..length {
                points.push(curve_point(points.len() as u32 * 20, trial, axis));
            }
        }
        let report = report(points);
        for budget in [1, 6, 7, 8, 13, 26] {
            let plot = report.for_plot(budget);
            assert_eq!(plot.curves.len(), budget.max(7));
            assert_eq!(plot.sample_count, 27);
            assert!(
                plot.curves
                    .windows(2)
                    .all(|pair| pair[0].t_ms < pair[1].t_ms)
            );
            for point in plot.curves.iter() {
                let original = &report.curves[(point.t_ms / 20) as usize];
                assert_eq!(
                    serde_json::to_value(point).unwrap(),
                    serde_json::to_value(original).unwrap()
                );
            }
            for index in [0, 4, 5, 23, 24, 25, 26] {
                assert!(
                    plot.curves
                        .iter()
                        .any(|point| point.t_ms == report.curves[index].t_ms)
                );
            }
        }
    }
    #[test]
    fn plotted_long_records_preserve_passive_joint_jitter_peaks() {
        let mut points: Vec<_> = (0..2000).map(|i| curve_point(i * 20, 0, 3)).collect();
        for p in &mut points {
            p.feedback = p.reference;
            p.feedback_velocity = p.velocity;
        }
        points[139].feedback[6] += 0.03;
        points[823].feedback[6] -= 0.02;
        points[1537].feedback_velocity[1] += 0.4;
        let plotted = report(points).for_plot(200);
        for index in [139, 823, 1537] {
            assert!(
                plotted.curves.iter().any(|p| p.t_ms == index * 20),
                "passive joint peak {index} disappeared from the plot"
            );
        }
    }

    #[test]
    fn legacy_json_without_new_fields_and_with_numeric_axis_still_loads() {
        let tuning = ServoTuning::default();
        let legacy = json!({
            "id": "legacy-calibration", "request_id": "legacy-request",
            "created_at_ms": 1_795_305_600_000_i64, "side": "left", "port": "offline-test",
            "settings": { "axis": 3, "excursion": 0.1, "speed": 0.02, "max_friction": 0.2 },
            "center_reference": ([0.0; 7]), "center_measured": ([0.1; 7]), "tcp": null,
            "baseline": tuning, "recommended": null,
            "trials": [{ "index": 0, "phase": "baseline", "config": tuning, "metrics": {
                "windows": 1, "velocity_rms": 0.004, "tracking_rms": 0.005,
                "tracking_peak": 0.01, "other_peak": 0.002, "valid_ms": 100
            }}],
            "outcome": "unchanged", "improvement": null, "message": "测量完成"
        });
        let decoded: Report = serde_json::from_value(legacy).unwrap();
        assert_eq!(decoded.settings.axis, Some(3));
        assert_eq!(decoded.trials[0].axis, None);
        assert!(decoded.choices.is_empty());
        assert_eq!(decoded.sample_count, 0);
        assert!(decoded.curves.is_empty());
        assert_eq!(decoded.for_plot(1).outcome, "unchanged");
    }

    #[test]
    fn full_reports_round_trip_with_raw_points_choices_and_after_trials() {
        let mut report = report(vec![curve_point(0, 0, 3), curve_point(20, 2, 3)]);
        report.trials.push(Trial {
            index: 2,
            phase: "after".into(),
            ..trial(2, 0.003, 0.003)
        });
        let encoded = serde_json::to_value(&report).unwrap();
        let decoded: Report = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(serde_json::to_value(decoded).unwrap(), encoded);
        assert_eq!(encoded["settings"]["axis"], json!(3));
        assert_eq!(encoded["curves"][0]["command"], Value::Null);
        assert_eq!(encoded["choices"][0]["after_index"], json!(2));
        let mut status = Status {
            space: "joint".into(),
            id: report.id.clone(),
            request_id: report.request_id.clone(),
            state: "completed".into(),
            stage: "done".into(),
            axis: 3,
            trial_index: 2,
            trial_count: 3,
            leg_index: 5,
            leg_count: 6,
            progress: 1.0,
            report: Some(report.summary()),
            live: None,
        };
        let encoded = serde_json::to_value(&status).unwrap();
        assert!(encoded.get("live").is_none());
        let decoded: Status = serde_json::from_value(encoded.clone()).unwrap();
        assert!(decoded.live.is_none());
        assert_eq!(serde_json::to_value(decoded).unwrap(), encoded);

        status.state = "running".into();
        status.stage = "candidate".into();
        status.report = None;
        status.live = Some(report.curves[1].clone());
        let encoded = serde_json::to_value(&status).unwrap();
        assert_eq!(
            encoded["live"],
            serde_json::to_value(&report.curves[1]).unwrap()
        );
        let decoded: Status = serde_json::from_value(encoded.clone()).unwrap();
        assert_eq!(serde_json::to_value(decoded).unwrap(), encoded);
    }
}
