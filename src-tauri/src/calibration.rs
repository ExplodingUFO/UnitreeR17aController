//! Offline low-speed calibration candidates, trajectories, and measurements.

use crate::{friction, kinematics::Kinematics, tuning::ServoTuning};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::sync::Arc;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub axis: Option<usize>,
    pub excursion: f64,
    pub speed: f64,
    /// Empty uses three speed levels around `speed`; these are test settings,
    /// not a second set of hardware motion limits.
    pub speeds: Vec<f64>,
    /// None derives the compensation budget from the recorded effort residual.
    pub max_friction: Option<f64>,
    pub cartesian: Option<Arc<crate::calibration_path::CartesianProgram>>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            axis: None,
            excursion: 16.0f64.to_radians(),
            speed: 2.0f64.to_radians(),
            speeds: Vec::new(),
            max_friction: None,
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
    if speeds(settings).iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err("校准试验速度必须为可表示的正数".into());
    }
    if settings
        .max_friction
        .is_some_and(|v| !v.is_finite() || v < 0.0)
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

pub fn speeds(settings: &Settings) -> Vec<f64> {
    if settings.speeds.is_empty() {
        vec![settings.speed / 2.0, settings.speed, settings.speed * 2.0]
    } else {
        settings.speeds.clone()
    }
}

/// Raw feedback only: gravity, derivatives and parameter fitting run off-thread.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub axis: usize,
    pub at_us: u64,
    pub position: [f64; 7],
    pub velocity: f64,
    pub effort: f64,
    neighbors: [(u64, f64); 2],
}

impl Observation {
    pub fn new(axis: usize, at_us: u64, position: [f64; 7], velocity: f64, effort: f64) -> Self {
        Self {
            axis,
            at_us,
            position,
            velocity,
            effort,
            neighbors: [(at_us, velocity); 2],
        }
    }

    fn acceleration_and_weight(&self) -> Option<(f64, f64)> {
        let [(before_at, before_velocity), (after_at, after_velocity)] = self.neighbors;
        let dt = after_at.checked_sub(before_at)? as f64 / 1_000_000.0;
        (dt > 0.0).then(|| ((after_velocity - before_velocity) / dt, dt))
    }
}

/// Preserve original neighboring feedback before any recording subsampling.
/// One pending sample supplies the central derivative without extra I/O or a
/// scan of retained data on the control thread.
pub struct ObservationRecording {
    samples: Recording<Observation>,
    pending: Option<((usize, usize), Observation)>,
}

impl ObservationRecording {
    pub fn new(budget: usize, groups: usize) -> Self {
        Self {
            samples: Recording::new(budget, groups),
            pending: None,
        }
    }

    pub fn push(&mut self, group: (usize, usize), mut sample: Observation) {
        if let Some((previous_group, mut previous)) = self.pending.take() {
            if previous_group == group {
                previous.neighbors[1] = (sample.at_us, sample.velocity);
                sample.neighbors[0] = (previous.at_us, previous.velocity);
            }
            self.samples.push(previous_group, previous);
        }
        self.pending = Some((group, sample));
    }

    /// Keep the final observed sample; a later frame starts its own derivative window.
    pub fn break_window(&mut self) {
        if let Some((group, sample)) = self.pending.take() {
            self.samples.push(group, sample);
        }
    }

    pub fn into_samples(mut self) -> Vec<Observation> {
        self.break_window();
        self.samples.into_samples()
    }
}

/// Share a recording budget across the program's trials (or movement legs).
/// Reservoir replacement is O(1) per capture and keeps each group's endpoints.
/// Sorting and serialization belong to the analysis/database workers.
pub struct Recording<T> {
    samples: Vec<T>,
    quota: usize,
    group: Option<(usize, usize)>,
    start: usize,
    seen: usize,
    captured: usize,
    random: u64,
}

impl<T> Recording<T> {
    pub fn new(budget: usize, groups: usize) -> Self {
        Self {
            samples: Vec::with_capacity(budget),
            quota: budget / groups.max(1),
            group: None,
            start: 0,
            seen: 0,
            captured: 0,
            random: 0,
        }
    }

    pub fn push(&mut self, group: (usize, usize), sample: T) {
        if self.group != Some(group) {
            self.group = Some(group);
            self.start = self.samples.len();
            self.seen = 0;
        }
        self.seen += 1;
        self.captured += 1;
        let retained = self.samples.len() - self.start;
        if retained < self.quota {
            self.samples.push(sample);
        } else if retained > 0 {
            let previous = std::mem::replace(self.samples.last_mut().unwrap(), sample);
            if retained > 2 {
                // The previous endpoint now joins the interior reservoir.
                self.random = self.random.wrapping_add(0x9e3779b97f4a7c15);
                let mut random = self.random;
                random = (random ^ (random >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
                random = (random ^ (random >> 27)).wrapping_mul(0x94d049bb133111eb);
                let slot = ((random ^ (random >> 31)) % (self.seen as u64 - 2)) as usize;
                if slot < retained - 2 {
                    self.samples[self.start + 1 + slot] = previous;
                }
            }
        }
    }

    pub fn last(&self) -> Option<&T> {
        self.samples.last()
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn captured(&self) -> usize {
        self.captured
    }

    pub fn into_samples(self) -> Vec<T> {
        self.samples
    }
}

pub fn identify(
    observations: &[Observation],
    model: &Kinematics,
    settings: &Settings,
    baseline: &ServoTuning,
) -> Vec<friction::Identification> {
    let axes = settings
        .axis
        .map_or_else(|| (0..7).collect(), |axis| vec![axis]);
    axes.into_iter()
        .map(|axis| {
            let mut data: Vec<_> = observations
                .iter()
                .filter(|s| s.axis == axis)
                .filter_map(|s| {
                    model
                        .gravity(&s.position)
                        .ok()
                        .map(|gravity| (*s, s.effort - gravity[axis]))
                })
                .collect();
            data.sort_unstable_by_key(|(s, _)| s.at_us);
            let samples: Vec<_> = data
                .iter()
                .filter_map(|(s, effort)| {
                    let (acceleration, weight) = s.acceleration_and_weight()?;
                    Some(friction::Sample {
                        velocity: s.velocity,
                        acceleration,
                        effort: *effort,
                        weight,
                    })
                })
                .collect();
            let budget = settings.max_friction.unwrap_or_else(|| {
                samples
                    .iter()
                    .map(|s| s.effort.abs())
                    .filter(|e| e.is_finite())
                    .fold(0.0_f64, f64::max)
            });
            friction::identify(axis, &samples, baseline.joints[axis].velocity_scale, budget)
        })
        .collect()
}

pub fn fitted_config(
    settings: &Settings,
    baseline: ServoTuning,
    fit: &friction::Identification,
) -> Option<ServoTuning> {
    if fit.positive.is_none() && fit.negative.is_none() {
        return None;
    }
    let budget = settings.max_friction.unwrap_or(fit.max_friction);
    if budget == 0.0 {
        return None;
    }
    let previous = baseline.joints[fit.axis];
    let inherited = if !baseline.friction_enabled {
        crate::tuning::StribeckTuning::default()
    } else if previous.stribeck.enabled {
        previous.stribeck
    } else {
        let curve = crate::tuning::FrictionCurve {
            coulomb: previous.coulomb,
            static_friction: previous.coulomb,
            viscous: previous.viscous,
            stribeck_velocity: previous.velocity_scale,
        };
        crate::tuning::StribeckTuning {
            enabled: true,
            positive: curve,
            negative: curve,
        }
    };
    let mut config = baseline;
    if !baseline.friction_enabled {
        for joint in &mut config.joints {
            joint.coulomb = 0.0;
            joint.viscous = 0.0;
            joint.stribeck.enabled = false;
        }
    }
    config.friction_enabled = true;
    let joint = &mut config.joints[fit.axis];
    joint.max_friction = budget;
    joint.stribeck.enabled = true;
    joint.stribeck.positive = fit
        .positive
        .as_ref()
        .map_or(inherited.positive, |d| d.curve);
    joint.stribeck.negative = fit
        .negative
        .as_ref()
        .map_or(inherited.negative, |d| d.curve);
    config.validate().ok()?;
    Some(config)
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
    let speeds = speeds(settings);
    let mut result = Vec::with_capacity(speeds.len() * 3);
    for speed in speeds {
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
    /// frame after a clock discontinuity may start a new window, but contributes
    /// no velocity or valid duration across the discontinuity. Sparse moving
    /// frames use their actual elapsed time without an arbitrary gap cutoff.
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
            && at_us <= last
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

    pub fn break_window(&mut self) {
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
    #[serde(default)]
    pub effort: Option<[f64; 7]>,
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
    pub captured_samples: usize,
    #[serde(default)]
    pub curves: Arc<Vec<Point>>,
    #[serde(default)]
    pub identification: Vec<friction::Identification>,
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
    /// A zero budget returns all retained samples. Already sorted records share
    /// their allocation; reservoir records are sorted only on a worker thread.
    pub fn for_plot(&self, max_points: usize) -> Self {
        if !self
            .curves
            .windows(2)
            .all(|pair| pair[0].t_ms <= pair[1].t_ms)
        {
            let mut sorted = self.clone();
            Arc::make_mut(&mut sorted.curves).sort_unstable_by_key(|point| point.t_ms);
            return sorted.for_plot(max_points);
        }
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
    fn automatic_and_explicit_speed_programs_revisit_the_same_previewed_range() {
        let defaults: Settings = serde_json::from_str("{}").unwrap();
        assert_eq!(defaults.axis, None);
        assert_eq!(defaults.max_friction, None);
        assert_eq!(
            speeds(&defaults),
            vec![defaults.speed / 2.0, defaults.speed, defaults.speed * 2.0]
        );
        let center = [0.5, -0.4, 0.3, -0.2, 0.1, -0.0, 0.7];
        for speed_levels in [Vec::new(), vec![0.00001, 0.03, 0.8, 1.2]] {
            let settings = Settings {
                speeds: speed_levels,
                ..defaults.clone()
            };
            let levels = speeds(&settings);
            for axis in 0..7 {
                let programme = legs(&settings, axis, center);
                assert_eq!(programme.len(), levels.len() * 3);
                for (index, leg) in programme.iter().enumerate() {
                    let offset =
                        [settings.excursion / 2.0, -settings.excursion / 2.0, 0.0][index % 3];
                    close(leg.target[axis], center[axis] + offset);
                    close(leg.speed, levels[index / 3]);
                    for other in (0..7).filter(|other| *other != axis) {
                        assert_eq!(leg.target[other].to_bits(), center[other].to_bits());
                    }
                    if index % 3 == 2 {
                        assert_eq!(leg.target.map(f64::to_bits), center.map(f64::to_bits));
                    }
                }
            }
        }
    }

    #[test]
    fn settings_use_finite_positive_test_speeds_and_legacy_optional_budgets() {
        let baseline = ServoTuning::default();
        let legacy: Settings = serde_json::from_str(r#"{"axis":3,"max_friction":12.5}"#).unwrap();
        assert_eq!(legacy.max_friction, Some(12.5));
        assert!(validate(&legacy, &baseline).is_ok());
        assert!(
            validate(
                &Settings {
                    max_friction: Some(0.0),
                    ..legacy.clone()
                },
                &baseline
            )
            .is_ok()
        );
        for bad in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
            let settings = Settings {
                speeds: vec![bad],
                ..legacy.clone()
            };
            assert!(validate(&settings, &baseline).is_err());
            assert!(legs(&settings, 3, [0.0; 7]).is_empty());
        }
        assert!(
            validate(
                &Settings {
                    speed: f64::MAX,
                    ..legacy.clone()
                },
                &baseline
            )
            .is_err()
        );
        assert!(
            validate(
                &Settings {
                    speed: f64::MAX,
                    speeds: vec![0.1],
                    ..legacy.clone()
                },
                &baseline
            )
            .is_ok()
        );
        assert!(
            validate(
                &Settings {
                    axis: Some(7),
                    ..legacy.clone()
                },
                &baseline
            )
            .is_err()
        );
        assert!(legs(&legacy, 7, [0.0; 7]).is_empty());
    }

    fn identified(axis: usize) -> friction::Identification {
        serde_json::from_value(json!({
            "axis":axis,"source":"feedback_effort_residual","samples":1000,
            "max_friction":1.25,"inertia":0.02,
            "positive":{"curve":{"coulomb":0.2,"static_friction":0.4,"viscous":0.3,"stribeck_velocity":0.025},
                "samples":500,"velocity_min":0.002,"velocity_max":0.1,"rmse":0.003,"points":[]},
            "negative":{"curve":{"coulomb":0.3,"static_friction":0.5,"viscous":0.4,"stribeck_velocity":0.04},
                "samples":500,"velocity_min":0.002,"velocity_max":0.1,"rmse":0.004,"points":[]}
        })).unwrap()
    }

    #[test]
    fn fitted_compensation_is_directional_preserves_gains_and_keeps_dormant_axes_off() {
        let mut baseline = ServoTuning::default();
        for (axis, joint) in baseline.joints.iter_mut().enumerate() {
            joint.coulomb = 0.2;
            joint.viscous = 0.3;
            joint.stribeck.enabled = true;
            joint.kd += axis as f64;
            joint.velocity_ff = 0.8;
        }
        let fit = identified(3);
        let candidate = fitted_config(&Settings::default(), baseline, &fit).unwrap();
        assert!(candidate.friction_enabled && candidate.joints[3].stribeck.enabled);
        assert_eq!(candidate.joints[3].max_friction, fit.max_friction);
        assert_eq!(
            candidate.joints[3].stribeck.positive,
            fit.positive.as_ref().unwrap().curve
        );
        assert_eq!(
            candidate.joints[3].stribeck.negative,
            fit.negative.as_ref().unwrap().curve
        );
        let output = candidate.apply([0.02; 7], [0.0; 7]).unwrap();
        for axis in 0..7 {
            assert_eq!(candidate.joints[axis].kd, baseline.joints[axis].kd);
            assert_eq!(
                candidate.joints[axis].velocity_ff,
                baseline.joints[axis].velocity_ff
            );
            if axis != 3 {
                assert_eq!(output.friction_requested[axis], 0.0);
            }
        }
        let explicit = fitted_config(
            &Settings {
                max_friction: Some(3.0),
                ..Settings::default()
            },
            baseline,
            &fit,
        )
        .unwrap();
        assert_eq!(explicit.joints[3].max_friction, 3.0);
        assert!(
            fitted_config(
                &Settings {
                    max_friction: Some(0.0),
                    ..Settings::default()
                },
                baseline,
                &fit
            )
            .is_none()
        );
        let mut partial = fit;
        partial.negative = None;
        let partial_config = fitted_config(&Settings::default(), baseline, &partial).unwrap();
        assert_eq!(
            partial_config.joints[3].stribeck.negative,
            crate::tuning::FrictionCurve::default()
        );
        assert_eq!(
            partial_config.joints[3].stribeck.positive,
            partial.positive.as_ref().unwrap().curve
        );
        partial.positive = None;
        assert!(fitted_config(&Settings::default(), baseline, &partial).is_none());
    }

    #[test]
    fn feedback_effort_identification_subtracts_actual_pose_gravity_without_using_ff_commands() {
        let model = Kinematics::load("right").unwrap();
        let axis = 1;
        let mut recording = ObservationRecording::new(1600, 1);
        for i in 0..1600 {
            let t = i as f64 * 0.02;
            let mut q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
            q[axis] += 0.1 * (t * 0.8).sin();
            let velocity = 0.08 * (t * 0.8).cos();
            let acceleration = -0.064 * (t * 0.8).sin();
            recording.push(
                (axis, 0),
                Observation::new(
                    axis,
                    i * 20_000,
                    q,
                    velocity,
                    model.gravity(&q).unwrap()[axis]
                        + 0.12 * (velocity / 0.02).tanh()
                        + 0.2 * velocity
                        + 0.03 * acceleration,
                ),
            );
        }
        let result = identify(
            &recording.into_samples(),
            &model,
            &Settings {
                axis: Some(axis),
                ..Settings::default()
            },
            &ServoTuning::default(),
        );
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].source, "feedback_effort_residual");
        assert!(result[0].positive.is_some() && result[0].negative.is_some());
        assert!(
            result[0].max_friction < 0.2,
            "model gravity leaked into the friction budget"
        );
        for direction in [&result[0].positive, &result[0].negative] {
            assert!(direction.as_ref().unwrap().rmse < 0.02);
        }
    }

    #[test]
    fn friction_and_metrics_exclude_the_interval_without_feedback() {
        let mut recording = ObservationRecording::new(32, 1);
        let mut metrics = Collector::new(0, [0.0; 7]);
        for (start_us, velocity) in [(0, 0.02), (700_000, -0.02)] {
            recording.break_window();
            metrics.break_window();
            for i in 0..=5 {
                let at_us = start_us + i * 20_000;
                let mut q = [0.0; 7];
                q[0] = velocity * (i as f64 * 0.02);
                recording.push((0, 0), Observation::new(0, at_us, q, velocity, 0.0));
                metrics.observe(at_us, q, q, true);
            }
        }
        let measured = recording.into_samples();
        assert_eq!(measured.len(), 12);
        for sample in measured {
            let (acceleration, weight) = sample.acceleration_and_weight().unwrap();
            assert_eq!(
                acceleration, 0.0,
                "velocity from the other side of a feedback gap"
            );
            assert!(weight <= 0.04);
        }
        assert_eq!(metrics.finish().unwrap().valid_ms, 200);
    }

    #[test]
    fn subsampling_keeps_original_feedback_neighbors_for_acceleration_and_fit() {
        let mut recording = ObservationRecording::new(90_000, 63);
        let omega = std::f64::consts::TAU;
        for i in 0..24_000 {
            let t = i as f64 * 0.01;
            let velocity = 0.06 * (omega * t).sin();
            let acceleration = 0.06 * omega * (omega * t).cos();
            let effort = 0.3 * (velocity / 0.017).tanh() + 0.2 * velocity + 0.2 * acceleration;
            recording.push(
                (0, 0),
                Observation::new(0, i * 10_000, [0.0; 7], velocity, effort),
            );
        }
        let retained = recording.into_samples();
        assert_eq!(retained.len(), 90_000 / 63);
        let samples: Vec<_> = retained
            .iter()
            .map(|s| {
                let (acceleration, weight) = s.acceleration_and_weight().unwrap();
                let analytic = 0.06 * omega * (omega * s.at_us as f64 / 1_000_000.0).cos();
                assert!((acceleration - analytic).abs() < 0.001);
                assert!(
                    weight <= 0.02 + 1e-12,
                    "sampling gaps must not become derivative intervals"
                );
                friction::Sample {
                    velocity: s.velocity,
                    acceleration,
                    effort: s.effort,
                    weight,
                }
            })
            .collect();
        let fit = friction::identify(0, &samples, 0.017, 1.0);
        assert!((fit.inertia - 0.2).abs() < 0.002, "I={}", fit.inertia);
        for direction in [fit.positive, fit.negative] {
            let direction = direction.unwrap();
            assert!(
                (direction.curve.viscous - 0.2).abs() < 0.002,
                "B={}",
                direction.curve.viscous
            );
            assert!(direction.rmse < 0.001);
        }

        let mut boundaries = ObservationRecording::new(4, 2);
        boundaries.push((0, 0), Observation::new(0, 0, [0.0; 7], 1.0, 0.0));
        boundaries.push((0, 0), Observation::new(0, 10_000, [0.0; 7], 2.0, 0.0));
        boundaries.push((0, 1), Observation::new(0, 1_000_000, [0.0; 7], -10.0, 0.0));
        boundaries.push((0, 1), Observation::new(0, 1_010_000, [0.0; 7], -9.0, 0.0));
        for sample in boundaries.into_samples() {
            assert_eq!(sample.acceleration_and_weight(), Some((100.0, 0.01)));
        }
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
    fn collector_uses_actual_elapsed_time_for_sparse_moving_feedback() {
        for interval_us in [300_000, 350_001, 2_000_000] {
            let mut collector = steady(20, 100_000, 0.02, 0.02);
            let at_us = 2_000_000 + interval_us;
            let reference = 0.02 * at_us as f64 / 1_000_000.0;
            collector.observe(at_us, point(3, reference), point(3, reference), true);
            let actual = collector.finish().unwrap();
            assert_eq!((actual.windows, actual.valid_ms), (21, at_us / 1000));
            close(actual.velocity_rms, 0.0);
        }
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

        baseline.metrics.as_mut().unwrap().windows = 184;
        baseline.metrics.as_mut().unwrap().valid_ms = 20_599;
        let mut low_friction = trial(1, 0.010345783198140366, 0.004638951499589994);
        low_friction.config.friction_enabled = true;
        low_friction.config.joints[3].coulomb = 0.1;
        let mut high_friction = trial(2, 0.010688245119382338, 0.0036441167515515152);
        high_friction.config.friction_enabled = true;
        high_friction.config.joints[3].coulomb = 0.2;
        let mut feedforward = trial(3, 0.010893930573407125, 0.007077228086301605);
        feedforward.config.joints[3].velocity_ff = 0.75;
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
            effort: None,
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
            captured_samples: curves.len(),
            curves: Arc::new(curves),
            identification: Vec::new(),
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
    fn bounded_recording_keeps_every_axis_leg_and_final_comparison() {
        let mut observations = Recording::new(630, 7 * 9);
        for axis in 0..7 {
            for leg in 0..9 {
                for sample in 0..1000 {
                    observations.push((axis, leg), (axis, leg, sample));
                }
            }
        }
        assert_eq!(observations.captured(), 63_000);
        let retained = observations.into_samples();
        assert_eq!(retained.len(), 630);
        for axis in 0..7 {
            for leg in 0..9 {
                let group: Vec<_> = retained
                    .iter()
                    .filter(|p| p.0 == axis && p.1 == leg)
                    .collect();
                assert_eq!(group.len(), 10);
                assert_eq!(group.first().unwrap().2, 0);
                assert_eq!(group.last().unwrap().2, 999);
                assert!(group.iter().all(|p| p.2 < 1000));
            }
        }

        let mut curves = Recording::new(420, 21);
        for trial in 0..21 {
            for sample in 0..1000 {
                curves.push(
                    (trial, 0),
                    curve_point((trial * 1000 + sample) as u32, trial, trial % 7),
                );
            }
        }
        assert_eq!(curves.captured(), 21_000);
        let plotted = report(curves.into_samples()).for_plot(0);
        assert_eq!(plotted.curves.len(), 420);
        assert!(plotted.curves.windows(2).all(|p| p[0].t_ms < p[1].t_ms));
        for trial in 0..21 {
            let group: Vec<_> = plotted.curves.iter().filter(|p| p.trial == trial).collect();
            assert_eq!(group.len(), 20);
            assert_eq!(group.first().unwrap().t_ms, trial as u32 * 1000);
            assert_eq!(group.last().unwrap().t_ms, trial as u32 * 1000 + 999);
        }
    }

    #[test]
    fn sorting_recorded_curves_does_not_mutate_the_live_report() {
        let report = report(vec![
            curve_point(0, 0, 3),
            curve_point(80, 0, 3),
            curve_point(20, 0, 3),
            curve_point(100, 0, 3),
        ]);
        let full = report.for_plot(0);
        assert_eq!(report.curves[1].t_ms, 80);
        assert_eq!(
            full.curves.iter().map(|p| p.t_ms).collect::<Vec<_>>(),
            [0, 20, 80, 100]
        );
        assert!(!Arc::ptr_eq(&report.curves, &full.curves));
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
