//! Hand guidance, lossless capture, and offline joint-space replay planning.
//!
//! Fitting and FK previews belong on the caller's worker. Sampling and approach
//! rebasing have no I/O, allocation, filtering, or dependency on feedback cadence.

use crate::kinematics::{Kinematics, Pose};
use crate::trajectory::MotionTuning;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub t_ms: u64,
    pub q: [f64; 7],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Summary {
    pub id: String,
    pub name: String,
    pub created_at_ms: i64,
    pub side: String,
    pub tcp: Pose,
    pub duration_ms: u64,
    pub sample_count: usize,
    pub outcome: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Trajectory {
    #[serde(flatten)]
    pub summary: Summary,
    pub points: Vec<Point>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Status {
    pub id: String,
    pub request_id: String,
    pub state: String,
    pub recording: bool,
    pub sample_count: usize,
    pub duration_ms: u64,
    pub progress: f64,
    pub trajectory_id: Option<String>,
}

const CHUNK_SIZE: usize = 256;

#[derive(Debug)]
struct Chunk {
    previous: Option<Arc<Chunk>>,
    points: Vec<Point>,
}

// A long capture must not recursively drop an unbounded linked chunk chain.
impl Drop for Chunk {
    fn drop(&mut self) {
        let mut previous = self.previous.take();
        while let Some(chunk) = previous {
            match Arc::try_unwrap(chunk) {
                Ok(mut chunk) => previous = chunk.previous.take(),
                Err(_) => break,
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct Capture {
    pub summary: Summary,
    tail: Option<Arc<Chunk>>,
}

impl Capture {
    /// The journal retains one owner after commit until all control snapshots
    /// have released their copies. Only its worker retires the final chunks.
    pub(crate) fn can_retire(&self) -> bool {
        self.tail
            .as_ref()
            .is_none_or(|tail| Arc::strong_count(tail) == 1)
    }

    pub(crate) fn shares_data(&self, other: &Self) -> bool {
        match (&self.tail, &other.tail) {
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            (None, None) => true,
            _ => false,
        }
    }

    #[cfg(test)]
    pub(crate) fn data_alive(&self) -> impl Fn() -> bool + Send + use<> {
        let weak = self.tail.as_ref().map(Arc::downgrade);
        move || weak.as_ref().is_some_and(|weak| weak.strong_count() > 0)
    }

    /// Flatten only on a worker; cloning Capture itself shares every raw point.
    pub fn trajectory(&self) -> Trajectory {
        let mut chunks = Vec::new();
        let mut tail = self.tail.as_deref();
        while let Some(chunk) = tail {
            chunks.push(chunk);
            tail = chunk.previous.as_deref();
        }
        let mut points = Vec::with_capacity(self.summary.sample_count);
        for chunk in chunks.into_iter().rev() {
            points.extend_from_slice(&chunk.points);
        }
        Trajectory {
            summary: self.summary.clone(),
            points,
        }
    }
}

#[derive(Debug)]
pub struct Recorder {
    summary: Summary,
    tail: Option<Arc<Chunk>>,
    current: Vec<Point>,
}

impl Recorder {
    pub fn new(mut summary: Summary) -> Self {
        summary.sample_count = 0;
        summary.duration_ms = 0;
        Self {
            summary,
            tail: None,
            current: Vec::with_capacity(CHUNK_SIZE),
        }
    }

    /// Retain every feedback sample, including identical positions/timestamps.
    /// Chunk rollover never reallocates or copies the existing capture.
    pub fn push(&mut self, mut point: Point) {
        point.t_ms = point.t_ms.max(self.summary.duration_ms);
        self.summary.duration_ms = point.t_ms;
        self.summary.sample_count += 1;
        if self.current.len() == CHUNK_SIZE {
            let points = std::mem::replace(&mut self.current, Vec::with_capacity(CHUNK_SIZE));
            self.tail = Some(Arc::new(Chunk {
                previous: self.tail.take(),
                points,
            }));
        }
        self.current.push(point);
    }

    pub fn sample_count(&self) -> usize {
        self.summary.sample_count
    }

    pub fn duration_ms(&self) -> u64 {
        self.summary.duration_ms
    }

    pub fn finish(mut self, outcome: &str) -> Capture {
        self.summary.outcome = outcome.to_owned();
        if !self.current.is_empty() {
            self.tail = Some(Arc::new(Chunk {
                previous: self.tail.take(),
                points: self.current,
            }));
        }
        Capture {
            summary: self.summary,
            tail: self.tail,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GuidanceSample {
    pub q: [f64; 7],
    pub stiffness: f64,
    pub settled: bool,
    pub dragging: bool,
}

#[derive(Clone, Debug)]
pub struct Guidance {
    transition_s: f64,
    dragging: bool,
    started_at: Option<f64>,
    from_q: [f64; 7],
    hold_q: [f64; 7],
    from_stiffness: f64,
}

impl Guidance {
    /// The caller validates its editable, positive transition duration.
    pub fn new(q: [f64; 7], transition_s: f64) -> Self {
        Self {
            transition_s,
            dragging: false,
            started_at: None,
            from_q: q,
            hold_q: q,
            from_stiffness: 1.0,
        }
    }

    pub fn set_drag(&mut self, pressed: bool, measured: [f64; 7], t: f64) {
        if pressed == self.dragging {
            return;
        }
        let current = self.sample(t, measured);
        self.from_q = current.q;
        self.from_stiffness = current.stiffness;
        self.hold_q = measured;
        self.dragging = pressed;
        self.started_at = Some(t);
    }

    pub fn sample(&self, t: f64, measured: [f64; 7]) -> GuidanceSample {
        let Some(started_at) = self.started_at else {
            return GuidanceSample {
                q: self.hold_q,
                stiffness: 1.0,
                settled: true,
                dragging: false,
            };
        };
        let u = if self.transition_s > 0.0 {
            ((t - started_at) / self.transition_s).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let target = if self.dragging { measured } else { self.hold_q };
        let stiffness = if self.dragging { 0.0 } else { 1.0 };
        let blend = ease(u);
        GuidanceSample {
            q: std::array::from_fn(|i| lerp(self.from_q[i], target[i], blend)),
            stiffness: lerp(self.from_stiffness, stiffness, blend),
            settled: u >= 1.0,
            dragging: self.dragging,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PlaybackOptions {
    pub speed_scale: f64,
    pub smoothing_s: f64,
}

impl Default for PlaybackOptions {
    fn default() -> Self {
        Self {
            speed_scale: 1.0,
            smoothing_s: 0.12,
        }
    }
}

impl PlaybackOptions {
    pub fn validate(&self) -> Result<(), String> {
        if !self.speed_scale.is_finite() || self.speed_scale <= 0.0 {
            return Err("teaching speed_scale must be finite and positive".into());
        }
        if !self.smoothing_s.is_finite() || self.smoothing_s < 0.0 {
            return Err("teaching smoothing_s must be finite and nonnegative".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub q: [f64; 7],
    pub dq: [f64; 7],
    pub ddq: [f64; 7],
}

impl Sample {
    fn hold(q: [f64; 7]) -> Self {
        Self {
            q,
            dq: [0.0; 7],
            ddq: [0.0; 7],
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Limits {
    velocity: [f64; 7],
    acceleration: f64,
    jerk: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Approach {
    origin: [f64; 7],
    target: [f64; 7],
    pub(crate) duration: f64,
}

impl Approach {
    pub(crate) fn for_motion(
        origin: [f64; 7],
        target: [f64; 7],
        velocity: [f64; 7],
        tuning: MotionTuning,
    ) -> Result<Self, String> {
        Self::new(
            origin,
            target,
            Limits {
                velocity,
                acceleration: tuning.acceleration,
                jerk: tuning.jerk,
            },
        )
    }

    fn new(origin: [f64; 7], target: [f64; 7], limits: Limits) -> Result<Self, String> {
        let mut duration: f64 = 0.0;
        for i in 0..7 {
            let distance = (target[i] - origin[i]).abs();
            if !origin[i].is_finite() || !distance.is_finite() {
                return Err("teaching approach anchor is non-finite or unrepresentable".into());
            }
            // Exact extrema of 10u^3 - 15u^4 + 6u^5 and its derivatives.
            duration = duration
                .max(1.875 * (distance / limits.velocity[i]))
                .max((10.0 / 3.0_f64.sqrt()).sqrt() * (distance / limits.acceleration).sqrt())
                .max(60.0_f64.cbrt() * (distance / limits.jerk).cbrt());
        }
        duration *= 1.0 + 256.0 * f64::EPSILON;
        if !duration.is_finite() {
            return Err("teaching approach duration is unrepresentable".into());
        }
        Ok(Self {
            origin,
            target,
            duration,
        })
    }

    pub(crate) fn sample(&self, t: f64) -> Sample {
        if t <= 0.0 {
            return Sample::hold(self.origin);
        }
        if t >= self.duration {
            return Sample::hold(self.target);
        }
        let u = t / self.duration;
        let position = ease(u);
        let velocity = 30.0 * u * u * (1.0 - u) * (1.0 - u);
        let acceleration = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u);
        Sample {
            q: std::array::from_fn(|i| lerp(self.origin[i], self.target[i], position)),
            dq: std::array::from_fn(|i| {
                (self.target[i] - self.origin[i]) / self.duration * velocity
            }),
            ddq: std::array::from_fn(|i| {
                (self.target[i] - self.origin[i]) / self.duration / self.duration * acceleration
            }),
        }
    }
}

#[derive(Debug)]
struct Segment {
    start: f64,
    end: f64,
    // Power coefficients in local normalized time, with the actual q in c0.
    coefficients: [[f64; 6]; 7],
}

impl Segment {
    fn sample(&self, t: f64, speed: f64) -> Sample {
        let h = self.end - self.start;
        let u = ((t - self.start) / h).clamp(0.0, 1.0);
        let mut sample = Sample::hold([0.0; 7]);
        for i in 0..7 {
            let c = self.coefficients[i];
            sample.q[i] = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
            sample.dq[i] =
                polynomial(&[c[1], 2.0 * c[2], 3.0 * c[3], 4.0 * c[4], 5.0 * c[5]], u) / h * speed;
            sample.ddq[i] =
                polynomial(&[2.0 * c[2], 6.0 * c[3], 12.0 * c[4], 20.0 * c[5]], u) / h * speed / h
                    * speed;
        }
        sample
    }
}

#[derive(Debug)]
struct Path {
    segments: Vec<Segment>,
    terminal: [f64; 7],
    duration: f64,
}

impl Path {
    fn first(&self) -> [f64; 7] {
        self.segments
            .first()
            .map_or(self.terminal, |segment| segment.coefficients.map(|c| c[0]))
    }

    fn sample(&self, t: f64, speed: f64) -> Sample {
        if t <= 0.0 {
            return Sample::hold(self.first());
        }
        if t >= self.duration || self.segments.is_empty() {
            return Sample::hold(self.terminal);
        }
        let index = self.segments.partition_point(|segment| segment.end <= t);
        self.segments[index].sample(t, speed)
    }
}

/// Cloning shares the entire fitted spline. Core may rebase its small approach
/// after Arc::make_mut without copying any recorded points or coefficients.
#[derive(Clone, Debug)]
pub struct Program {
    trajectory_id: String,
    side: String,
    tcp: Pose,
    approach: Approach,
    path: Arc<Path>,
    limits: Limits,
    requested_speed_scale: f64,
    effective_speed_scale: f64,
    path_duration: f64,
}

impl Program {
    pub fn duration(&self) -> f64 {
        self.approach.duration + self.path_duration
    }

    pub fn approach_duration(&self) -> f64 {
        self.approach.duration
    }

    pub fn effective_speed_scale(&self) -> f64 {
        self.effective_speed_scale
    }

    pub fn side(&self) -> &str {
        &self.side
    }

    pub fn trajectory_id(&self) -> &str {
        &self.trajectory_id
    }

    /// O(7), independent of capture size. On error the old program is unchanged.
    pub fn rebase_approach(&mut self, origin: [f64; 7]) -> Result<(), String> {
        let approach = Approach::new(origin, self.path.first(), self.limits)?;
        if !(approach.duration + self.path_duration).is_finite() {
            return Err("teaching duration is unrepresentable".into());
        }
        self.approach = approach;
        Ok(())
    }

    /// Absolute elapsed program time; O(log N), with no allocation or feedback
    /// integration. The caller must publish this sampled position itself.
    pub fn sample(&self, t: f64) -> Sample {
        let t = if t.is_nan() { 0.0 } else { t };
        if t <= 0.0 {
            return Sample::hold(self.approach.origin);
        }
        if t < self.approach.duration {
            return self.approach.sample(t);
        }
        if t >= self.duration() {
            return Sample::hold(self.path.terminal);
        }
        self.path.sample(
            (t - self.approach.duration) * self.effective_speed_scale,
            self.effective_speed_scale,
        )
    }

    /// Display samples follow the actual program, including its approach and
    /// every fitted interval. Display density does not change replay fidelity.
    pub fn preview(&self, model: &Kinematics) -> Result<Preview, String> {
        let mut points = Vec::new();
        let mut add = |t: f64, phase: &str| -> Result<(), String> {
            let joints = self.sample(t).q;
            let tcp = model.fk(&joints, &self.tcp)?.translation;
            points.push(PreviewPoint {
                t_ms: t * 1000.0,
                joints,
                tcp,
                phase: phase.to_owned(),
            });
            Ok(())
        };
        add(0.0, "approach")?;
        if self.approach.duration > 0.0 {
            for i in 1..=32 {
                add(self.approach.duration * (i as f64 / 32.0), "approach")?;
            }
        }
        add(self.approach.duration, "path")?;
        for segment in &self.path.segments {
            for i in 1..=4 {
                let raw_t = segment.start + (segment.end - segment.start) * (i as f64 / 4.0);
                add(
                    self.approach.duration + raw_t / self.effective_speed_scale,
                    "path",
                )?;
            }
        }
        Ok(Preview {
            trajectory_id: self.trajectory_id.clone(),
            side: self.side.clone(),
            tcp: self.tcp,
            origin: self.approach.origin,
            duration_ms: self.duration() * 1000.0,
            approach_duration_ms: self.approach.duration * 1000.0,
            requested_speed_scale: self.requested_speed_scale,
            effective_speed_scale: self.effective_speed_scale(),
            points,
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Preview {
    pub trajectory_id: String,
    pub side: String,
    pub tcp: Pose,
    pub origin: [f64; 7],
    pub duration_ms: f64,
    pub approach_duration_ms: f64,
    pub requested_speed_scale: f64,
    pub effective_speed_scale: f64,
    pub points: Vec<PreviewPoint>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PreviewPoint {
    pub t_ms: f64,
    pub joints: [f64; 7],
    pub tcp: [f64; 3],
    pub phase: String,
}

/// Offline only. Time-aware smoothing is optional; fitting preserves nonzero
/// interior derivatives, with zero velocity/acceleration only at the two ends.
pub fn plan(
    trajectory: &Trajectory,
    origin: [f64; 7],
    model: &Kinematics,
    tuning: MotionTuning,
    options: PlaybackOptions,
) -> Result<Program, String> {
    options.validate()?;
    tuning.validate()?;
    model.fk(&origin, &trajectory.summary.tcp)?;
    if !matches!(trajectory.summary.side.as_str(), "right" | "left") {
        return Err("teaching side must be right or left".into());
    }
    let limits = Limits {
        velocity: tuning.speed_limits(model.speed_limits()),
        acceleration: tuning.acceleration,
        jerk: tuning.jerk,
    };
    let mut points: Vec<Point> = Vec::with_capacity(trajectory.points.len());
    for point in &trajectory.points {
        if point.q.iter().any(|q| !q.is_finite()) {
            return Err("teaching joint position is non-finite".into());
        }
        if let Some(last) = points.last_mut() {
            if point.t_ms < last.t_ms {
                return Err("teaching timestamps must be monotonic".into());
            }
            if point.t_ms == last.t_ms {
                *last = point.clone();
                continue;
            }
        }
        points.push(point.clone());
    }
    if points.is_empty() {
        return Err("teaching trajectory is empty".into());
    }
    let first_ms = points[0].t_ms;
    let times: Vec<f64> = points
        .iter()
        .map(|point| (point.t_ms - first_ms) as f64 / 1000.0)
        .collect();
    if times.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err("teaching time intervals are unrepresentable".into());
    }
    let mut positions: Vec<[f64; 7]> = points.into_iter().map(|point| point.q).collect();
    smooth(&times, &mut positions, options.smoothing_s)?;
    let path = fit(&times, &positions)?;
    let effective_speed_scale = retime(&path, limits, options.speed_scale)?;
    let path_duration = path.duration / effective_speed_scale;
    let approach = Approach::new(origin, path.first(), limits)?;
    if !path_duration.is_finite() || !(path_duration + approach.duration).is_finite() {
        return Err("teaching duration is unrepresentable".into());
    }
    Ok(Program {
        trajectory_id: trajectory.summary.id.clone(),
        side: trajectory.summary.side.clone(),
        tcp: trajectory.summary.tcp,
        approach,
        path: Arc::new(path),
        limits,
        requested_speed_scale: options.speed_scale,
        effective_speed_scale,
        path_duration,
    })
}

fn lerp(from: f64, to: f64, blend: f64) -> f64 {
    if blend <= 0.0 {
        from
    } else if blend >= 1.0 {
        to
    } else {
        from * (1.0 - blend) + to * blend
    }
}

fn ease(u: f64) -> f64 {
    // Equivalent to 10u^3 - 15u^4 + 6u^5, with exact endpoint values.
    if u >= 1.0 {
        1.0
    } else {
        u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    }
}

fn polynomial(c: &[f64], u: f64) -> f64 {
    c.iter().rev().fold(0.0, |value, c| value * u + c)
}

/// Symmetric exponential smoothing of the deviation from the endpoint chord.
/// Actual timestamp gaps set the filter weights; linear endpoint correction
/// retains both taught endpoint poses. No feedback sample is filtered online.
fn smooth(times: &[f64], q: &mut [[f64; 7]], smoothing_s: f64) -> Result<(), String> {
    if smoothing_s == 0.0 || q.len() <= 2 {
        return Ok(());
    }
    let duration = *times.last().unwrap();
    let first = q[0];
    let last = q[q.len() - 1];
    let residual: Vec<[f64; 7]> = q
        .iter()
        .zip(times)
        .map(|(q, t)| std::array::from_fn(|j| q[j] - lerp(first[j], last[j], t / duration)))
        .collect();
    let mut forward = vec![[0.0; 7]; q.len()];
    for i in 1..q.len() {
        let blend = -(-(times[i] - times[i - 1]) / smoothing_s).exp_m1();
        forward[i] = std::array::from_fn(|j| lerp(forward[i - 1][j], residual[i][j], blend));
    }
    let mut backward = [0.0; 7];
    let mut filtered = vec![[0.0; 7]; q.len()];
    for i in (0..q.len()).rev() {
        if i + 1 < q.len() {
            let blend = -(-(times[i + 1] - times[i]) / smoothing_s).exp_m1();
            backward = std::array::from_fn(|j| lerp(backward[j], residual[i][j], blend));
        }
        filtered[i] = std::array::from_fn(|j| 0.5 * forward[i][j] + 0.5 * backward[j]);
    }
    let left = filtered[0];
    let right = filtered[q.len() - 1];
    for i in 1..q.len() - 1 {
        let u = times[i] / duration;
        q[i] = std::array::from_fn(|j| {
            lerp(first[j], last[j], u) + filtered[i][j] - lerp(left[j], right[j], u)
        });
        if q[i].iter().any(|value| !value.is_finite()) {
            return Err("teaching smoothed position is unrepresentable".into());
        }
    }
    Ok(())
}

fn fit(times: &[f64], q: &[[f64; 7]]) -> Result<Path, String> {
    let mut velocity = vec![[0.0; 7]; q.len()];
    let mut acceleration = vec![[0.0; 7]; q.len()];
    for i in 1..q.len() - 1 {
        let left = times[i] - times[i - 1];
        let right = times[i + 1] - times[i];
        for j in 0..7 {
            let before = (q[i][j] - q[i - 1][j]) / left;
            let after = (q[i + 1][j] - q[i][j]) / right;
            // Derivatives of the local quadratic through the irregular knots.
            velocity[i][j] = lerp(before, after, left / (left + right));
            acceleration[i][j] = 2.0 * ((after - before) / (left + right));
        }
    }
    let mut segments = Vec::with_capacity(q.len() - 1);
    for i in 0..q.len() - 1 {
        let h = times[i + 1] - times[i];
        let coefficients = std::array::from_fn(|j| {
            let d = q[i + 1][j] - q[i][j];
            let v0 = velocity[i][j] * h;
            let v1 = velocity[i + 1][j] * h;
            let a0 = acceleration[i][j] * h * h;
            let a1 = acceleration[i + 1][j] * h * h;
            [
                q[i][j],
                v0,
                0.5 * a0,
                10.0 * d - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1,
                -15.0 * d + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1,
                6.0 * d - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1,
            ]
        });
        if coefficients
            .iter()
            .any(|c: &[f64; 6]| !c.iter().map(|value| value.abs()).sum::<f64>().is_finite())
        {
            return Err("teaching spline coefficients are unrepresentable".into());
        }
        segments.push(Segment {
            start: times[i],
            end: times[i + 1],
            coefficients,
        });
    }
    Ok(Path {
        segments,
        terminal: q[q.len() - 1],
        duration: *times.last().unwrap(),
    })
}

fn retime(path: &Path, limits: Limits, requested: f64) -> Result<f64, String> {
    let mut speed = requested;
    for segment in &path.segments {
        let h = segment.end - segment.start;
        for (j, c) in segment.coefficients.iter().enumerate() {
            let v = peak_bound(&[c[1], 2.0 * c[2], 3.0 * c[3], 4.0 * c[4], 5.0 * c[5]])? / h;
            let a = peak_bound(&[2.0 * c[2], 6.0 * c[3], 12.0 * c[4], 20.0 * c[5]])? / h / h;
            let jerk = peak_bound(&[6.0 * c[3], 24.0 * c[4], 60.0 * c[5]])? / h / h / h;
            if ![v, a, jerk].iter().all(|value| value.is_finite()) {
                return Err("teaching spline derivatives are unrepresentable".into());
            }
            if v > 0.0 {
                speed = speed.min(limits.velocity[j] / v);
            }
            if a > 0.0 {
                speed = speed.min(limits.acceleration.sqrt() / a.sqrt());
            }
            if jerk > 0.0 {
                speed = speed.min(limits.jerk.cbrt() / jerk.cbrt());
            }
        }
    }
    if !speed.is_finite() || speed <= 0.0 {
        return Err("teaching replay time scale is unrepresentable".into());
    }
    Ok(speed)
}

/// The convex hull of Bernstein coefficients bounds the polynomial throughout
/// [0,1]. de Casteljau subdivision tightens this bound without relying on a time
/// grid, so narrow peaks between samples cannot violate the configured limits.
fn peak_bound(power: &[f64]) -> Result<f64, String> {
    let n = power.len() - 1;
    let magnitude = power.iter().map(|c| c.abs()).sum::<f64>();
    if !magnitude.is_finite() {
        return Err("teaching polynomial bound is unrepresentable".into());
    }
    if magnitude == 0.0 {
        return Ok(0.0);
    }
    let mut bernstein = [0.0; 5];
    for (k, value) in bernstein.iter_mut().enumerate().take(n + 1) {
        let mut ratio = 1.0;
        for (i, coefficient) in power.iter().enumerate().take(k + 1) {
            if i > 0 {
                ratio *= (k + 1 - i) as f64 / (n + 1 - i) as f64;
            }
            *value += ratio * coefficient;
        }
    }
    let mut lower: f64 = 0.0;
    let bound = subdivided_bound(bernstein, n, 0, &mut lower);
    Ok(bound + magnitude * (256.0 * f64::EPSILON))
}

fn subdivided_bound(b: [f64; 5], n: usize, depth: u8, lower: &mut f64) -> f64 {
    let upper = b[..=n].iter().map(|v| v.abs()).fold(0.0, f64::max);
    *lower = lower.max(b[0].abs()).max(b[n].abs());
    if upper <= *lower * (1.0 + 1e-8) || depth == 14 {
        return upper;
    }
    let mut work = b;
    let mut left = [0.0; 5];
    let mut right = [0.0; 5];
    left[0] = work[0];
    right[n] = work[n];
    for level in 1..=n {
        for i in 0..=n - level {
            work[i] = 0.5 * work[i] + 0.5 * work[i + 1];
        }
        left[level] = work[0];
        right[n - level] = work[n - level];
    }
    subdivided_bound(left, n, depth + 1, lower).max(subdivided_bound(right, n, depth + 1, lower))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn summary(side: &str) -> Summary {
        Summary {
            id: "test-trajectory".into(),
            name: "hand guided".into(),
            created_at_ms: 1234,
            side: side.into(),
            tcp: Pose {
                translation: [0.03, -0.02, 0.11],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            duration_ms: 0,
            sample_count: 0,
            outcome: "completed".into(),
        }
    }

    fn recording(points: Vec<Point>) -> Trajectory {
        let mut summary = summary("right");
        summary.sample_count = points.len();
        summary.duration_ms = points.last().map_or(0, |point| point.t_ms);
        Trajectory { summary, points }
    }

    fn options(speed_scale: f64) -> PlaybackOptions {
        PlaybackOptions {
            speed_scale,
            smoothing_s: 0.0,
        }
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "actual={actual}, expected={expected}, tolerance={tolerance}"
        );
    }

    fn joints_close(actual: [f64; 7], expected: [f64; 7], tolerance: f64) {
        for i in 0..7 {
            close(actual[i], expected[i], tolerance);
        }
    }

    fn sample_bits(actual: GuidanceSample, expected: GuidanceSample) {
        assert_eq!(actual.q.map(f64::to_bits), expected.q.map(f64::to_bits));
        assert_eq!(actual.stiffness.to_bits(), expected.stiffness.to_bits());
    }

    fn irregular() -> Trajectory {
        recording(
            [1000, 1011, 1047, 1100, 1280, 1300, 1590, 1850]
                .into_iter()
                .map(|t_ms| {
                    let t = (t_ms - 1000) as f64 / 1000.0;
                    Point {
                        t_ms,
                        q: std::array::from_fn(|j| (j as f64 + 1.0) * (0.04 * t + 0.02 * t * t)),
                    }
                })
                .collect(),
        )
    }

    #[test]
    fn guidance_enters_drag_then_latches_release_pose_without_a_step() {
        let mut guidance = Guidance::new([0.2; 7], 0.4);
        let initial = guidance.sample(0.0, [0.9; 7]);
        assert_eq!(initial.q, [0.2; 7]);
        assert_eq!(initial.stiffness, 1.0);
        assert!(initial.settled && !initial.dragging);

        guidance.set_drag(true, [0.9; 7], 1.0);
        sample_bits(guidance.sample(1.0, [0.9; 7]), initial);
        let entering = guidance.sample(1.2, [0.9; 7]);
        joints_close(entering.q, [0.55; 7], 1e-14);
        close(entering.stiffness, 0.5, 1e-14);
        assert!(!entering.settled && entering.dragging);
        let dragging = guidance.sample(1.5, [1.2; 7]);
        assert_eq!(dragging.q, [1.2; 7]);
        assert_eq!(dragging.stiffness, 0.0);
        assert!(dragging.settled && dragging.dragging);

        guidance.set_drag(false, [1.2; 7], 1.5);
        sample_bits(guidance.sample(1.5, [1.2; 7]), dragging);
        let leaving = guidance.sample(1.7, [1.8; 7]);
        assert_eq!(leaving.q, [1.2; 7]);
        close(leaving.stiffness, 0.5, 1e-14);
        assert!(!leaving.settled && !leaving.dragging);
        let held = guidance.sample(2.0, [4.0; 7]);
        assert_eq!(held.q, [1.2; 7]);
        assert_eq!(held.stiffness, 1.0);
        assert!(held.settled && !held.dragging);
    }

    #[test]
    fn guidance_mid_transition_reversals_and_held_repeats_are_continuous() {
        let mut guidance = Guidance::new([0.0; 7], 0.4);
        for (pressed, t, measured) in [
            (true, 0.0, 0.6),
            (false, 0.13, 0.7),
            (true, 0.21, 0.8),
            (false, 0.25, 0.9),
            (true, 0.31, 1.0),
            (false, 0.34, 1.1),
        ] {
            let before = guidance.sample(t, [measured; 7]);
            guidance.set_drag(pressed, [measured; 7], t);
            let after = guidance.sample(t, [measured; 7]);
            sample_bits(after, before);
            let next = guidance.sample(t + 1e-8, [measured; 7]);
            joints_close(next.q, after.q, 1e-12);
            close(next.stiffness, after.stiffness, 1e-12);

            let unmodified = guidance.clone();
            guidance.set_drag(pressed, [measured + 10.0; 7], t + 0.01);
            for time in [t + 0.02, t + 0.2, t + 0.5] {
                sample_bits(
                    guidance.sample(time, [measured; 7]),
                    unmodified.sample(time, [measured; 7]),
                );
            }
        }
        let held = guidance.sample(1.0, [2.0; 7]);
        assert_eq!(held.q, [1.1; 7]);
        assert_eq!(held.stiffness, 1.0);
        assert!(held.settled && !held.dragging);
    }

    #[test]
    fn recorder_keeps_all_chunks_timestamps_and_cheap_capture_clones() {
        let mut metadata = summary("left");
        metadata.sample_count = 999;
        metadata.duration_ms = 999;
        let mut recorder = Recorder::new(metadata);
        assert_eq!(recorder.sample_count(), 0);
        assert_eq!(recorder.duration_ms(), 0);
        let count = CHUNK_SIZE * 4 + 37;
        for i in 0..count {
            recorder.push(Point {
                t_ms: (i * 3) as u64,
                q: [i as f64; 7],
            });
        }
        recorder.push(Point {
            t_ms: 0,
            q: [-2.0; 7],
        });
        assert_eq!(recorder.sample_count(), count + 1);
        assert_eq!(recorder.duration_ms(), ((count - 1) * 3) as u64);
        let capture = recorder.finish("stopped");
        let clone = capture.clone();
        assert!(Arc::ptr_eq(
            capture.tail.as_ref().unwrap(),
            clone.tail.as_ref().unwrap()
        ));
        let trajectory = clone.trajectory();
        assert_eq!(trajectory.summary.side, "left");
        assert_eq!(trajectory.summary.outcome, "stopped");
        assert_eq!(trajectory.points.len(), count + 1);
        assert_eq!(trajectory.summary.sample_count, count + 1);
        for (i, point) in trajectory.points[..count].iter().enumerate() {
            assert_eq!(point.t_ms, (i * 3) as u64);
            assert_eq!(point.q, [i as f64; 7]);
        }
        assert_eq!(trajectory.points[count].q, [-2.0; 7]);
        assert_eq!(
            trajectory.points[count].t_ms,
            trajectory.points[count - 1].t_ms
        );
        drop(capture);
        assert_eq!(clone.trajectory(), trajectory);
    }

    #[test]
    fn irregular_timestamp_spline_is_interpolating_c2_and_keeps_interior_motion() {
        let model = Kinematics::load("right").unwrap();
        let data = irregular();
        let program = plan(
            &data,
            data.points[0].q,
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        let speed = program.effective_speed_scale();
        assert_eq!(program.approach_duration(), 0.0);
        for point in &data.points {
            let t = (point.t_ms - data.points[0].t_ms) as f64 / 1000.0 / speed;
            joints_close(program.sample(t).q, point.q, 1e-12);
        }
        for pair in program.path.segments.windows(2) {
            let before = pair[0].sample(pair[0].end, speed);
            let after = pair[1].sample(pair[1].start, speed);
            joints_close(before.q, after.q, 2e-12);
            joints_close(before.dq, after.dq, 2e-10);
            joints_close(before.ddq, after.ddq, 2e-8);
            assert!(after.dq.iter().any(|velocity| velocity.abs() > 1e-5));
        }
        assert_eq!(program.sample(0.0).dq, [0.0; 7]);
        assert_eq!(program.sample(0.0).ddq, [0.0; 7]);
        assert_eq!(program.sample(program.duration()).dq, [0.0; 7]);
        assert_eq!(program.sample(program.duration()).ddq, [0.0; 7]);
        // Irregular control/send reads do not change the absolute-time result.
        let at = program.duration() * 0.417;
        let expected = program.sample(at);
        for fraction in [0.1, 0.9001, 0.12, 0.011, 0.7, 1.4, -0.1] {
            program.sample(program.duration() * fraction);
            assert_eq!(
                program.sample(at).q.map(f64::to_bits),
                expected.q.map(f64::to_bits)
            );
            assert_eq!(
                program.sample(at).dq.map(f64::to_bits),
                expected.dq.map(f64::to_bits)
            );
        }
    }

    #[test]
    fn duplicate_timestamps_use_latest_q_without_losing_raw_capture() {
        let model = Kinematics::load("right").unwrap();
        let mut recorder = Recorder::new(summary("right"));
        for (t_ms, q) in [(0, 0.1), (13, 0.2), (13, 0.3), (80, 0.4)] {
            recorder.push(Point { t_ms, q: [q; 7] });
        }
        let data = recorder.finish("completed").trajectory();
        assert_eq!(data.points.len(), 4);
        let program = plan(
            &data,
            [0.1; 7],
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        assert_eq!(program.path.segments.len(), 2);
        let knot_time = 0.013 / program.effective_speed_scale();
        joints_close(program.sample(knot_time).q, [0.3; 7], 1e-12);
    }

    #[test]
    fn smoothing_is_offline_adjustable_and_zero_preserves_every_knot() {
        let model = Kinematics::load("right").unwrap();
        let data = recording(
            (0..61)
                .map(|i| Point {
                    t_ms: i * 20,
                    q: [0.001 * i as f64 + if i % 2 == 0 { 0.006 } else { -0.006 }; 7],
                })
                .collect(),
        );
        let raw = plan(
            &data,
            data.points[0].q,
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        let smoothed = plan(
            &data,
            data.points[0].q,
            &model,
            MotionTuning::default(),
            PlaybackOptions {
                smoothing_s: 0.12,
                ..options(1.0)
            },
        )
        .unwrap();
        let mut roughness_raw = 0.0;
        let mut roughness_smoothed = 0.0;
        for point in &data.points {
            let t = point.t_ms as f64 / 1000.0;
            joints_close(raw.path.sample(t, 1.0).q, point.q, 1e-12);
        }
        for i in 1..data.points.len() - 1 {
            let at = |offset: isize| (i as isize + offset) as f64 * 0.02;
            let second = |path: &Path| {
                path.sample(at(-1), 1.0).q[0] - 2.0 * path.sample(at(0), 1.0).q[0]
                    + path.sample(at(1), 1.0).q[0]
            };
            roughness_raw += second(&raw.path).powi(2);
            roughness_smoothed += second(&smoothed.path).powi(2);
        }
        assert!(roughness_smoothed < roughness_raw * 0.05);
        assert_eq!(smoothed.path.first(), data.points[0].q);
        assert_eq!(smoothed.path.terminal, data.points.last().unwrap().q);
        assert_eq!(data.points.len(), 61);
        let stronger = plan(
            &data,
            data.points[0].q,
            &model,
            MotionTuning::default(),
            PlaybackOptions {
                smoothing_s: f64::MAX,
                ..options(1.0)
            },
        )
        .unwrap();
        close(stronger.path.sample(0.6, 1.0).q[0], 0.036, 1e-12);
    }

    #[test]
    fn bernstein_peak_bounds_enclose_between_sample_extrema_and_are_tight() {
        for (power, exact) in [
            (vec![0.0, 0.0, 30.0, -60.0, 30.0], 1.875),
            (vec![0.0, 60.0, -180.0, 120.0], 10.0 / 3.0_f64.sqrt()),
            (vec![60.0, -360.0, 360.0], 60.0),
        ] {
            let bound = peak_bound(&power).unwrap();
            assert!(bound >= exact);
            assert!(bound <= exact * (1.0 + 1e-6));
        }
        for power in [
            vec![0.06, -4.0, 15.0, -20.0, 10.0],
            vec![0.25, -1.0, 1.0],
            vec![1.0, -12.0, 37.0, -45.0, 19.0],
        ] {
            for scale in [1e-80, 1.0, 1e80] {
                let c: Vec<f64> = power.iter().map(|v| v * scale).collect();
                let bound = peak_bound(&c).unwrap();
                let mut observed: f64 = 0.0;
                for i in 0..=10_000 {
                    observed = observed.max(polynomial(&c, i as f64 / 10_000.0).abs());
                }
                assert!(bound >= observed, "bound={bound}, observed={observed}");
                assert!(bound <= observed * (1.0 + 1e-5));
            }
        }
    }

    fn dynamics_ratios(program: &Program) -> [f64; 3] {
        let mut ratios = [0.0_f64; 3];
        for segment in &program.path.segments {
            let h = segment.end - segment.start;
            for tick in 0..=1000 {
                let u = tick as f64 / 1000.0;
                let at = segment.start + h * u;
                let sample = segment.sample(at, program.effective_speed_scale());
                let sent = program
                    .sample(program.approach_duration() + at / program.effective_speed_scale());
                joints_close(sent.q, sample.q, 2e-10);
                for (j, c) in segment.coefficients.iter().enumerate() {
                    let jerk = polynomial(&[6.0 * c[3], 24.0 * c[4], 60.0 * c[5]], u) / h
                        * program.effective_speed_scale()
                        / h
                        * program.effective_speed_scale()
                        / h
                        * program.effective_speed_scale();
                    ratios[0] = ratios[0].max(sample.dq[j].abs() / program.limits.velocity[j]);
                    ratios[1] = ratios[1].max(sample.ddq[j].abs() / program.limits.acceleration);
                    ratios[2] = ratios[2].max(jerk.abs() / program.limits.jerk);
                    assert!(sent.dq[j].abs() <= program.limits.velocity[j]);
                    assert!(sent.ddq[j].abs() <= program.limits.acceleration);
                }
            }
        }
        for ratio in ratios {
            assert!(ratio <= 1.0, "dynamics ratio={ratio}");
        }
        ratios
    }

    #[test]
    fn uniform_retiming_respects_motion_tuning_and_urdf_without_an_extra_cap() {
        let model = Kinematics::load("right").unwrap();
        let data = recording(
            [0, 7, 50, 58, 190, 400, 437, 780]
                .into_iter()
                .enumerate()
                .map(|(i, t_ms)| Point {
                    t_ms,
                    q: std::array::from_fn(|j| (i as f64 * 0.8 + j as f64 * 0.4).sin() * 0.07),
                })
                .collect(),
        );
        let tuning = MotionTuning {
            joint_speed: 0.037,
            acceleration: 0.11,
            jerk: 0.43,
        };
        let program = plan(&data, data.points[0].q, &model, tuning, options(50.0)).unwrap();
        assert!(program.effective_speed_scale() < 50.0);
        let ratios = dynamics_ratios(&program);
        assert!(ratios.into_iter().fold(0.0, f64::max) > 0.999);

        let slow_data = recording(vec![
            Point {
                t_ms: 0,
                q: [0.0; 7],
            },
            Point {
                t_ms: 1_000_000,
                q: [0.0001; 7],
            },
        ]);
        let unrestricted = plan(&slow_data, [0.0; 7], &model, tuning, options(1000.0)).unwrap();
        assert_eq!(unrestricted.effective_speed_scale(), 1000.0);
        close(unrestricted.duration(), 1.0, 1e-14);
        dynamics_ratios(&unrestricted);

        let urdf_tuning = MotionTuning {
            joint_speed: 1000.0,
            acceleration: 1e12,
            jerk: 1e18,
        };
        let urdf = plan(&slow_data, [0.0; 7], &model, urdf_tuning, options(1e9)).unwrap();
        assert_eq!(urdf.limits.velocity, model.speed_limits());
        assert!(dynamics_ratios(&urdf)[0] > 0.999);

        let slower = plan(&slow_data, [0.0; 7], &model, tuning, options(0.25)).unwrap();
        assert_eq!(slower.effective_speed_scale(), 0.25);
        close(slower.duration(), 4000.0, 1e-12);
        joints_close(slower.sample(2000.0).q, unrestricted.sample(0.5).q, 1e-15);
    }

    #[test]
    fn approach_rebase_starts_at_passed_anchor_and_shares_the_fitted_path() {
        fn send_sync<T: Send + Sync>() {}
        send_sync::<Program>();
        let model = Kinematics::load("right").unwrap();
        let data = irregular();
        let original = plan(
            &data,
            [0.0; 7],
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        let mut rebased = original.clone();
        let anchor = [0.5, -0.3, 0.2, -0.1, 0.08, -0.04, 0.01];
        rebased.rebase_approach(anchor).unwrap();
        assert!(Arc::ptr_eq(&original.path, &rebased.path));
        assert_eq!(
            original.path.segments.as_ptr(),
            rebased.path.segments.as_ptr()
        );
        assert_eq!(rebased.sample(0.0).q, anchor);
        assert_eq!(rebased.sample(-1.0).q, anchor);
        assert_eq!(original.sample(0.0).q, [0.0; 7]);
        assert_eq!(rebased.path_duration, original.path_duration);
        assert_eq!(
            rebased.effective_speed_scale(),
            original.effective_speed_scale()
        );
        assert_eq!(
            rebased.sample(rebased.approach_duration()).q,
            data.points[0].q
        );
        assert_eq!(rebased.sample(rebased.approach_duration()).dq, [0.0; 7]);
        assert_eq!(rebased.sample(rebased.approach_duration()).ddq, [0.0; 7]);
        for tick in 0..=1000 {
            let t = rebased.approach_duration() * tick as f64 / 1000.0;
            let sample = rebased.sample(t);
            let u = t / rebased.approach_duration();
            for (j, anchor) in anchor.into_iter().enumerate() {
                let jerk = (data.points[0].q[j] - anchor)
                    / rebased.approach_duration()
                    / rebased.approach_duration()
                    / rebased.approach_duration()
                    * (60.0 - 360.0 * u + 360.0 * u * u);
                assert!(sample.dq[j].abs() <= rebased.limits.velocity[j]);
                assert!(sample.ddq[j].abs() <= rebased.limits.acceleration);
                assert!(jerk.abs() <= rebased.limits.jerk);
            }
        }
        let old_origin = rebased.sample(0.0).q;
        assert!(rebased.rebase_approach([f64::NAN; 7]).is_err());
        assert_eq!(rebased.sample(0.0).q, old_origin);
    }

    #[test]
    fn one_point_stationary_and_repeated_timestamp_pose_captures_are_valid() {
        let model = Kinematics::load("right").unwrap();
        let single = recording(vec![Point {
            t_ms: 0,
            q: [0.2; 7],
        }]);
        let program = plan(
            &single,
            [0.0; 7],
            &model,
            MotionTuning::default(),
            options(2.0),
        )
        .unwrap();
        assert!(program.duration() > 0.0);
        assert_eq!(program.duration(), program.approach_duration());
        assert_eq!(program.sample(program.duration()).q, [0.2; 7]);
        let stationary = recording(vec![
            Point {
                t_ms: 0,
                q: [0.2; 7],
            },
            Point {
                t_ms: 1,
                q: [0.2; 7],
            },
            Point {
                t_ms: 3000,
                q: [0.2; 7],
            },
        ]);
        let program = plan(
            &stationary,
            [0.2; 7],
            &model,
            MotionTuning::default(),
            options(17.0),
        )
        .unwrap();
        assert_eq!(program.effective_speed_scale(), 17.0);
        close(program.duration(), 3.0 / 17.0, 1e-14);
        for t in [0.0, 1e-8, 0.1, 0.2, f64::INFINITY, f64::NAN] {
            let sample = program.sample(t);
            assert_eq!(sample.q, [0.2; 7]);
            assert_eq!(sample.dq, [0.0; 7]);
            assert_eq!(sample.ddq, [0.0; 7]);
        }
        let repeated = recording(vec![
            Point {
                t_ms: 4,
                q: [0.1; 7],
            },
            Point {
                t_ms: 4,
                q: [0.3; 7],
            },
        ]);
        let program = plan(
            &repeated,
            [0.3; 7],
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        assert_eq!(program.duration(), 0.0);
        assert_eq!(program.sample(0.0).q, [0.3; 7]);
        assert_eq!(program.sample(0.1).q, [0.3; 7]);
    }

    #[test]
    fn preview_uses_real_program_joints_tcp_and_both_phases_for_both_arms() {
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            let mut data = irregular();
            data.summary.side = side.into();
            let origin = [-0.1; 7];
            let program =
                plan(&data, origin, &model, MotionTuning::default(), options(0.7)).unwrap();
            let preview = program.preview(&model).unwrap();
            assert_eq!(program.side(), side);
            assert_eq!(program.trajectory_id(), data.summary.id);
            assert_eq!(preview.origin, origin);
            assert_eq!(preview.tcp, data.summary.tcp);
            assert_eq!(preview.trajectory_id, data.summary.id);
            assert_eq!(preview.points[0].joints, origin);
            assert_eq!(preview.points[0].phase, "approach");
            assert_eq!(preview.points.last().unwrap().phase, "path");
            joints_close(
                preview.points.last().unwrap().joints,
                data.points.last().unwrap().q,
                1e-12,
            );
            assert!(
                preview
                    .points
                    .iter()
                    .any(|point| point.phase == "approach" && point.t_ms > 0.0)
            );
            assert!(preview.points.iter().any(|point| point.phase == "path"));
            for point in &preview.points {
                let expected = program.sample(point.t_ms / 1000.0);
                joints_close(point.joints, expected.q, 1e-11);
                let expected_tcp = model
                    .fk(&point.joints, &data.summary.tcp)
                    .unwrap()
                    .translation;
                for (actual, expected) in point.tcp.into_iter().zip(expected_tcp) {
                    close(actual, expected, 1e-13);
                }
            }
            close(
                preview.points.last().unwrap().t_ms,
                preview.duration_ms,
                1e-9,
            );
        }
    }

    #[test]
    fn validation_checks_finite_data_and_signs_without_pose_or_speed_caps() {
        let model = Kinematics::load("right").unwrap();
        let mut data = irregular();
        assert!(
            plan(
                &recording(Vec::new()),
                [0.0; 7],
                &model,
                MotionTuning::default(),
                options(1.0)
            )
            .is_err()
        );
        for speed in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                PlaybackOptions {
                    speed_scale: speed,
                    smoothing_s: 0.0
                }
                .validate()
                .is_err()
            );
        }
        for smoothing in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                PlaybackOptions {
                    speed_scale: 1.0,
                    smoothing_s: smoothing
                }
                .validate()
                .is_err()
            );
        }
        for speed in [1e-12, 1.0, 1000.0, f64::MAX] {
            for smoothing in [0.0, 0.12, f64::MAX] {
                let options = PlaybackOptions {
                    speed_scale: speed,
                    smoothing_s: smoothing,
                };
                assert!(options.validate().is_ok());
                assert!(plan(&data, [0.0; 7], &model, MotionTuning::default(), options).is_ok());
            }
        }
        let outside = recording(vec![Point {
            t_ms: 0,
            q: [1000.0; 7],
        }]);
        let uncapped = plan(
            &outside,
            [999.0; 7],
            &model,
            MotionTuning::default(),
            options(1.0),
        )
        .unwrap();
        assert_eq!(uncapped.sample(uncapped.duration()).q, [1000.0; 7]);
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            data.points[2].q[3] = value;
            assert!(
                plan(
                    &data,
                    [0.0; 7],
                    &model,
                    MotionTuning::default(),
                    options(1.0)
                )
                .is_err()
            );
        }
        data.points[2].q[3] = 0.1;
        assert!(
            plan(
                &data,
                [f64::NAN; 7],
                &model,
                MotionTuning::default(),
                options(1.0)
            )
            .is_err()
        );
        let defaults: PlaybackOptions = serde_json::from_str("{}").unwrap();
        assert_eq!(defaults.speed_scale, 1.0);
        assert_eq!(defaults.smoothing_s, 0.12);
        let partial: PlaybackOptions = serde_json::from_str("{\"speed_scale\":2.5}").unwrap();
        assert_eq!(partial.speed_scale, 2.5);
        assert_eq!(partial.smoothing_s, 0.12);
    }
}
