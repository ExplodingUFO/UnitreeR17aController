//! Offline interpolation of a single ordered calibration leg. The eighth
//! coordinate is signed Cartesian progress in metres. It participates in the
//! speed certificate alongside the seven joints, then gets discarded.
use crate::{
    kinematics::{Kinematics, Pose},
    teaching::Sample,
    trajectory::MotionTuning,
};
use nalgebra::SVector;

#[derive(Debug, Clone, PartialEq)]
pub struct TimedLeg {
    knots: Vec<Knot>,
    time_scale: f64,
}

#[derive(Debug, Clone, PartialEq)]
struct Knot {
    time: f64,
    q: [f64; 7],
    dq: [f64; 7],
}

type Rates = [f64; 8];

struct Span {
    delta: Rates,
    duration: f64,
}

impl Span {
    fn rates(&self) -> Rates {
        self.delta.map(|d| d / self.duration)
    }
}

// One physical tangent per knot, shared by both interpolants. A reversal or
// plateau only stops the affected coordinate. Otherwise each adjacent span
// can sustain at least this velocity without changing its direction.
fn tangent(a: f64, b: f64) -> f64 {
    if a.is_sign_positive() == b.is_sign_positive() {
        a.abs().min(b.abs()).copysign(a)
    } else {
        0.0
    }
}

fn neighbors(spans: &[Span], i: usize) -> [Rates; 2] {
    [
        i.checked_sub(1).map_or([0.0; 8], |j| spans[j].rates()),
        spans.get(i + 1).map_or([0.0; 8], Span::rates),
    ]
}

fn middle_rate(mean: f64, left: f64, right: f64) -> f64 {
    mean + 0.5 * (mean - left) + 0.5 * (mean - right)
}

// On each half-span, velocity blends from its knot value to a shared middle
// value with 3x² - 2x³ (x runs from the knot toward the middle). The integral
// of that blend is 1/2, so vm = 2*mean - (v0+v1)/2 hits the other knot.
// All three velocities have the chord's sign. Peak |v| is |vm|, peak |a|
// is 3*max|vm-ve|/h, and peak |jerk| is 24*max|vm-ve|/h² (e = either end).
// The half-span joins and all original knots have exactly zero acceleration.
fn required_duration(
    span: &Span,
    h: f64,
    adjacent: &[Rates; 2],
    speed: &Rates,
    motion: MotionTuning,
) -> f64 {
    let mut required = 0.0_f64;
    for (axis, &d) in span.delta.iter().enumerate() {
        if d == 0.0 {
            continue;
        }
        let mean = d / h;
        let left = tangent(mean, adjacent[0][axis]);
        let right = tangent(mean, adjacent[1][axis]);
        let middle = middle_rate(mean, left, right);
        // Form differences in physical units before normalization. Nearly
        // equal secants can still give appreciable jerk on very short spans.
        let (peak, gap) = if mean != 0.0 && middle.is_finite() {
            (
                middle / mean,
                (middle - left).abs().max((middle - right).abs()) / mean.abs(),
            )
        } else {
            (2.0, 2.0)
        };
        required = required.max(d.abs() / speed[axis] * peak);
        if axis < 7 && gap > 0.0 {
            // Take roots before division to avoid overflow for short,
            // tightly spaced spans with a large unscaled jerk.
            required = required
                .max(d.abs().sqrt() / motion.acceleration.sqrt() * (3.0 * gap).sqrt())
                .max(d.abs().cbrt() / motion.jerk.cbrt() * (24.0 * gap).cbrt());
        }
    }
    required
}

// With adjacent times fixed, increasing h can only increase the normalized
// end tangents toward one. Thus required_duration(h) is nonincreasing and
// gives an upper bracket for the shortest feasible duration. Only adjacent
// spans need another check after changing h. Batch updates to make reversal
// symmetric. The small margin bounds repeated repairs near an active limit.
const TIME_MARGIN: f64 = 1.000_01;

fn retime(spans: &mut [Span], speed: &Rates, motion: MotionTuning) -> Result<(), String> {
    let mut pending: Vec<_> = (0..spans.len()).collect();
    let mut changes = Vec::new();
    while !pending.is_empty() {
        for i in pending.drain(..) {
            let span = &spans[i];
            let adjacent = neighbors(spans, i);
            let minimum = |h| required_duration(span, h, &adjacent, speed, motion);
            let mut low = span.duration;
            let mut high = minimum(low);
            if high <= low {
                continue;
            }
            if !high.is_finite() {
                return Err("calibration spline derivative time is unrepresentable".into());
            }
            // Geometric bisection also brackets very short clipped end spans.
            while high - low > (TIME_MARGIN - 1.0) * high {
                let mid = low.sqrt() * high.sqrt();
                if mid <= low || mid >= high {
                    break;
                }
                if mid >= minimum(mid) {
                    high = mid;
                } else {
                    low = mid;
                }
            }
            let duration = high * TIME_MARGIN;
            if !duration.is_finite() {
                return Err("calibration spline interval is unrepresentable".into());
            }
            changes.push((i, duration));
        }
        for (i, duration) in changes.drain(..) {
            spans[i].duration = duration;
            if i > 0 {
                pending.push(i - 1);
            }
            if i + 1 < spans.len() {
                pending.push(i + 1);
            }
        }
        pending.sort_unstable();
        pending.dedup();
    }
    Ok(())
}

impl TimedLeg {
    pub fn fit(
        knots: &[(f64, [f64; 7])],
        speed: f64,
        motion: MotionTuning,
        model_velocity: [f64; 7],
    ) -> Result<Self, String> {
        motion.validate()?;
        if !speed.is_finite()
            || speed <= 0.0
            || model_velocity.iter().any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err("calibration spline speeds must be finite and positive".into());
        }
        if knots.len() < 2 {
            return Err("calibration spline needs at least two knots".into());
        }
        if knots
            .iter()
            .any(|(s, q)| !s.is_finite() || q.iter().any(|v| !v.is_finite()))
        {
            return Err("calibration spline knot is non-finite".into());
        }
        // A repeated stationary target is used for an already-complete leg.
        if knots.iter().all(|k| *k == knots[0]) {
            let hold = Knot {
                time: 0.0,
                q: knots[0].1,
                dq: [0.0; 7],
            };
            return Ok(Self {
                knots: vec![hold; knots.len()],
                time_scale: 1.0,
            });
        }
        let mut limits = [speed; 8];
        limits[..7].copy_from_slice(&motion.speed_limits(model_velocity));
        let increasing = knots[1].0 > knots[0].0;
        let mut spans = Vec::with_capacity(knots.len() - 1);
        for pair in knots.windows(2) {
            let (s0, q0) = pair[0];
            let (s1, q1) = pair[1];
            let ds = s1 - s0;
            if ds == 0.0 || (ds > 0.0) != increasing {
                return Err("calibration spline displacement must be strictly ordered".into());
            }
            let mut delta = [ds; 8];
            for axis in 0..7 {
                delta[axis] = q1[axis] - q0[axis];
            }
            let duration = delta
                .iter()
                .zip(limits)
                .map(|(d, limit)| d.abs() / limit)
                .fold(0.0, f64::max)
                * TIME_MARGIN;
            if !duration.is_finite()
                || duration <= 0.0
                || delta.iter().any(|d| !(d / duration).is_finite())
            {
                return Err("calibration spline chord or time is unrepresentable".into());
            }
            spans.push(Span { delta, duration });
        }
        retime(&mut spans, &limits, motion)?;

        let mut time = 0.0;
        let mut result = vec![Knot {
            time,
            q: knots[0].1,
            dq: [0.0; 7],
        }];
        for (span, &(_, q)) in spans.iter_mut().zip(&knots[1..]) {
            let mut end = time + span.duration;
            if end - time < span.duration {
                end = end.next_up();
            }
            if !end.is_finite() {
                return Err("calibration spline elapsed time is unrepresentable".into());
            }
            span.duration = end - time;
            result.push(Knot {
                time: end,
                q,
                dq: [0.0; 7],
            });
            time = end;
        }
        // Certify the durations actually representable on the absolute clock.
        // In particular, rounding a tiny tail can also change its neighbor's
        // tangent. Runtime evaluation uses exactly these spans and tangents.
        for (i, span) in spans.iter().enumerate() {
            let adjacent = neighbors(&spans, i);
            if required_duration(span, span.duration, &adjacent, &limits, motion) > span.duration {
                return Err("calibration spline clock precision cannot certify limits".into());
            }
            let rates = span.rates();
            if rates
                .iter()
                .zip(span.delta)
                .any(|(rate, d)| *rate == 0.0 && d != 0.0)
            {
                return Err("calibration spline chord rate is unrepresentable".into());
            }
            result[i].dq = std::array::from_fn(|j| tangent(adjacent[0][j], rates[j]));
        }
        Ok(Self {
            knots: result,
            time_scale: 1.0,
        })
    }

    /// Bound the actual TCP translation speed throughout every interval, for
    /// the specified tool and model. A common time dilation preserves geometry,
    /// knot continuity and joint derivative limits. Reapplying is idempotent.
    pub fn limit_tcp_speed(
        &mut self,
        model: &Kinematics,
        tcp: &Pose,
        speed: f64,
    ) -> Result<(), String> {
        if !speed.is_finite() || speed <= 0.0 {
            return Err("calibration TCP speed must be finite and positive".into());
        }
        if self.duration() == 0.0 {
            return model.fk(&self.knots[0].q, tcp).map(|_| ());
        }
        let reach = model.reach_radius(tcp);
        let mut scale = self.time_scale;
        for pair in self.knots.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let h = b.time - a.time;
            let center = std::array::from_fn(|j| a.q[j] + 0.5 * (b.q[j] - a.q[j]));
            let matrix = model.jacobian(&center, tcp)?;
            let linear = matrix.fixed_rows::<3>(0);
            let radii = SVector::<f64, 7>::from_fn(|j, _| {
                (b.q[j] - center[j]).abs().max((center[j] - a.q[j]).abs())
            });
            // |dJ_j/dq_k| <= |J_max(j,k)| <= reach. Apply the coarse
            // reach bound once to enclose each column over the box, then use
            // these shorter local lever arms to bound its actual variation.
            let coarse = reach * radii.sum();
            let arms =
                SVector::<f64, 7>::from_fn(|j, _| (linear.column(j).norm() + coarse).min(reach));
            let variation = SVector::<f64, 7>::from_fn(|j, _| {
                radii
                    .iter()
                    .enumerate()
                    .map(|(k, r)| r * arms[j.max(k)])
                    .sum::<f64>()
                    + reach * (64.0 * f64::EPSILON)
            });
            let middle =
                std::array::from_fn(|j| middle_rate((b.q[j] - a.q[j]) / h, a.dq[j], b.dq[j]));
            // For a revolute chain, each partial derivative of a linear
            // Jacobian column is an axis crossed with another column, so its
            // norm is at most reach. Monotonicity puts q in the endpoint box.
            // v is a convex combination of its knot and middle vectors; the
            // norm bound attains its maximum at one of these three controls.
            for velocity in [a.dq, middle, b.dq] {
                let v = SVector::from(velocity);
                let bound = (linear * v).norm() + variation.dot(&v.abs());
                let required = bound / speed * (1.0 + 64.0 * f64::EPSILON);
                if !required.is_finite() {
                    return Err("calibration TCP speed bound is unrepresentable".into());
                }
                scale = scale.max(required);
            }
        }
        let mut previous = 0.0;
        for knot in &self.knots[1..] {
            let time = knot.time * scale;
            if !time.is_finite() || time <= previous {
                return Err("calibration TCP time is unrepresentable".into());
            }
            previous = time;
        }
        self.time_scale = scale;
        Ok(())
    }

    pub fn duration(&self) -> f64 {
        self.knots.last().unwrap().time * self.time_scale
    }

    /// Elapsed seconds for the input knot at this zero-based index.
    pub fn knot_time(&self, index: usize) -> f64 {
        self.knots[index].time * self.time_scale
    }

    pub fn sample(&self, elapsed: f64) -> Sample {
        let hold = |q| Sample {
            q,
            dq: [0.0; 7],
            ddq: [0.0; 7],
        };
        if elapsed.is_nan() || elapsed <= 0.0 {
            return hold(self.knots[0].q);
        }
        if elapsed >= self.duration() {
            return hold(self.knots.last().unwrap().q);
        }
        let i = self
            .knots
            .partition_point(|k| k.time * self.time_scale <= elapsed);
        let a = &self.knots[i - 1];
        let b = &self.knots[i];
        let elapsed = if elapsed == a.time * self.time_scale {
            a.time
        } else {
            (elapsed / self.time_scale).clamp(a.time, b.time)
        };
        let h = b.time - a.time;
        let from_left = elapsed - a.time;
        let from_right = b.time - elapsed;
        let (edge, dt, sign) = if from_left <= from_right {
            (a, from_left, 1.0)
        } else {
            (b, from_right, -1.0)
        };
        let x = 2.0 * (dt / h);
        let blend = x * x * (3.0 - 2.0 * x);
        let integrated = x * x * (1.0 - 0.5 * x);
        let mut sample = hold(edge.q);
        for j in 0..7 {
            let mean = (b.q[j] - a.q[j]) / h;
            let middle = middle_rate(mean, a.dq[j], b.dq[j]);
            let change = middle - edge.dq[j];
            let q = edge.q[j] + sign * dt * (edge.dq[j] + change * integrated);
            sample.q[j] = q.clamp(a.q[j].min(b.q[j]), a.q[j].max(b.q[j]));
            sample.dq[j] = (edge.dq[j] + change * blend) / self.time_scale;
            sample.ddq[j] =
                sign * (change / h) * (12.0 * x * (1.0 - x)) / self.time_scale / self.time_scale;
        }
        sample
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected} (±{tolerance})"
        );
    }

    fn check_spans(
        leg: &TimedLeg,
        knots: &[(f64, [f64; 7])],
        speed: f64,
        motion: MotionTuning,
        model: [f64; 7],
    ) {
        assert_eq!(leg.knot_time(0), 0.0);
        assert_eq!(leg.knot_time(knots.len() - 1), leg.duration());
        for (i, &(s, q)) in knots.iter().enumerate() {
            let t = leg.knot_time(i);
            let at_knot = leg.sample(t);
            assert_eq!(at_knot.q, q);
            assert_eq!(at_knot.ddq, [0.0; 7]);
            if i == 0 || i == knots.len() - 1 {
                assert_eq!(at_knot.dq, [0.0; 7]);
            }
            if i + 1 == knots.len() {
                break;
            }
            let (next_s, next_q) = knots[i + 1];
            let h = leg.knot_time(i + 1) - t;
            assert!(h.is_finite() && h > 0.0 && (next_s - s).abs() <= h * speed);
            let mut previous = at_knot;
            for k in 1..=64 {
                let sample = leg.sample(t + h * (k as f64 / 64.0));
                for j in 0..7 {
                    let (lo, hi) = (q[j].min(next_q[j]), q[j].max(next_q[j]));
                    assert!((lo..=hi).contains(&sample.q[j]));
                    let dir = (next_q[j] - q[j]).signum();
                    assert!((sample.q[j] - previous.q[j]) * dir >= -2e-15);
                    assert!(sample.dq[j] * dir >= 0.0);
                    assert!(sample.dq[j].abs() <= motion.joint_speed.min(model[j]) * (1.0 + 1e-10));
                    assert!(sample.ddq[j].abs() <= motion.acceleration * (1.0 + 1e-10));
                }
                previous = sample;
            }
            // Each half's acceleration is a quadratic with zeros at its ends.
            // Its endpoint jerk magnitudes are exactly 8*|a(quarter)|/h.
            for u in [0.25, 0.75] {
                let quarter = leg.sample(t + h * u);
                for a in quarter.ddq {
                    assert!(8.0 * a.abs() / h <= motion.jerk * (1.0 + 1e-9));
                }
            }
        }
    }

    #[test]
    fn two_knots_reach_each_analytic_limit() {
        let knots = [(0.0, [0.0; 7]), (1.0, [1.0; 7])];
        let loose = MotionTuning {
            joint_speed: 1e5,
            acceleration: 1e5,
            jerk: 1e5,
        };
        for (speed, motion, model, expected) in [
            (0.25, loose, [1e5; 7], 8.0),
            (
                1e5,
                MotionTuning {
                    joint_speed: 0.5,
                    ..loose
                },
                [1e5; 7],
                4.0,
            ),
            (1e5, loose, [0.1, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], 20.0),
            (
                1e5,
                MotionTuning {
                    acceleration: 0.25,
                    ..loose
                },
                [1e5; 7],
                24.0_f64.sqrt(),
            ),
            (
                1e5,
                MotionTuning { jerk: 0.5, ..loose },
                [1e5; 7],
                96.0_f64.cbrt(),
            ),
        ] {
            let leg = TimedLeg::fit(&knots, speed, motion, model).unwrap();
            check_spans(&leg, &knots, speed, motion, model);
            assert!(leg.duration() >= expected);
            close(leg.duration(), expected, 3e-5 * expected);
            let h = leg.duration();
            close(leg.sample(h / 2.0).dq[0], 2.0 / h, 1e-12);
            close(leg.sample(h / 4.0).ddq[0], 6.0 / h / h, 1e-12);
            close(leg.sample(3.0 * h / 4.0).ddq[0], -6.0 / h / h, 1e-12);
        }
    }

    #[test]
    fn irregular_turns_and_plateaus_preserve_each_joint_and_c2() {
        let knots = [
            (-0.01, [-0.1, 0.0, 0.3, 0.5, 0.0, 1.1, -1.0]),
            (0.007, [0.3, 0.0, 0.2, 0.5, 0.0, 1.0, -0.8]),
            (0.0071, [0.28, 0.1, 0.2, 0.5, 0.0, 0.98, -0.79]),
            (0.1, [-0.2, 0.6, 0.3, 0.5, 0.0, 0.9, -0.6]),
        ];
        let motion = MotionTuning::default();
        let model = [0.2; 7];
        let leg = TimedLeg::fit(&knots, 0.03, motion, model).unwrap();
        check_spans(&leg, &knots, 0.03, motion, model);
        let turn = leg.sample(leg.knot_time(1));
        assert_eq!(turn.dq[..5], [0.0; 5]);
        assert!(turn.dq[5] < 0.0 && turn.dq[6] > 0.0);
        for i in 1..knots.len() - 1 {
            let t = leg.knot_time(i);
            let center = leg.sample(t);
            for dt in [-1e-7, 1e-7] {
                let side = leg.sample(t + dt);
                for (j, limit) in model.iter().enumerate() {
                    close(side.q[j], center.q[j], dt.abs() * limit);
                    close(side.dq[j], center.dq[j], dt.abs() * motion.acceleration);
                    close(side.ddq[j], center.ddq[j], dt.abs() * motion.jerk);
                }
            }
        }
        // Check physical derivatives independently of the fitted peak formula.
        for i in 1..knots.len() {
            let h = leg.knot_time(i) - leg.knot_time(i - 1);
            for u in [0.17, 0.5, 0.79] {
                let t = leg.knot_time(i - 1) + u * h;
                let dt = 1e-6 * h;
                let (left, sample, right) = (leg.sample(t - dt), leg.sample(t), leg.sample(t + dt));
                for j in 0..7 {
                    close((right.q[j] - left.q[j]) / (2.0 * dt), sample.dq[j], 2e-8);
                    close(
                        (right.dq[j] - left.dq[j]) / (2.0 * dt),
                        sample.ddq[j],
                        motion.jerk * dt,
                    );
                }
            }
        }
    }

    #[test]
    fn tiny_clipped_tail_keeps_cruise_local_in_both_directions() {
        // q[0] equals signed Cartesian progress, exposing instantaneous work
        // speed through the public sample API, including at the half-spans.
        let mut knots: Vec<_> = (0..=100)
            .map(|i| {
                let s = i as f64 * 0.002;
                (s, [s; 7])
            })
            .collect();
        let end = 0.200_000_000_1;
        knots.push((end, [end; 7]));
        let speed = 0.02;
        let motion = MotionTuning::default();
        for _ in 0..2 {
            let leg = TimedLeg::fit(&knots, speed, motion, [1.0; 7]).unwrap();
            check_spans(&leg, &knots, speed, motion, [1.0; 7]);
            assert!(leg.duration() < 1.1 * end / speed);
            for i in 0..knots.len() - 1 {
                let middle = (leg.knot_time(i) + leg.knot_time(i + 1)) / 2.0;
                let peak = leg.sample(middle).dq[0].abs();
                assert!(peak <= speed);
                if (10..90).contains(&i) {
                    assert!(peak > 0.99 * speed);
                }
            }
            knots.reverse();
        }
    }

    #[test]
    fn captured_cartesian_reference_retraces_at_reversed_times() {
        // Right-arm calibration 4a3ca783, reference at 20, 49, 249, 680 and
        // 759 ms. Offsets use base X from the URDF FK of these rounded poses.
        #[rustfmt::skip]
        let mut knots = [
            (6.25462946365896e-9, [1.048512678514, 0.336960211597, -0.024104441502, 0.522173894843, 1.683941714697, 0.073632819544, 0.349720248379]),
            (1.58585326166111e-7, [1.048512692673, 0.336959396741, -0.024104022245, 0.522173762915, 1.683941310560, 0.073632668154, 0.349720951204]),
            (2.08108844161371e-5, [1.048514609392, 0.336848925219, -0.024047180933, 0.522155882669, 1.683886518671, 0.073612141226, 0.349816239203]),
            (0.000308704315531542, [1.048540879013, 0.335309589286, -0.023254704113, 0.521907903716, 1.683122466054, 0.073325674913, 0.351145035474]),
            (0.000407433016448012, [1.048549714863, 0.334781974639, -0.022982887053, 0.521823412622, 1.682860331018, 0.073227316043, 0.351600930765]),
        ];
        let fit = |k: &[(f64, [f64; 7])]| {
            TimedLeg::fit(k, 0.01, MotionTuning::default(), [1.0; 7]).unwrap()
        };
        let forward = fit(&knots);
        check_spans(&forward, &knots, 0.01, MotionTuning::default(), [1.0; 7]);
        knots.reverse();
        let reverse = fit(&knots);
        check_spans(&reverse, &knots, 0.01, MotionTuning::default(), [1.0; 7]);
        close(forward.duration(), reverse.duration(), 1e-12);
        for step in 0..=257 {
            let t = forward.duration() * step as f64 / 257.0;
            let a = forward.sample(t);
            let b = reverse.sample(reverse.duration() - t);
            for j in 0..7 {
                close(a.q[j], b.q[j], 1e-13);
                close(a.dq[j], -b.dq[j], 1e-13);
                close(a.ddq[j], b.ddq[j], 1e-12);
            }
        }
    }

    #[test]
    fn actual_tcp_stays_on_axis_and_below_work_speed_between_preview_knots() {
        use crate::calibration_path::{PreviewRequest, preview};
        use crate::kinematics::{CartesianPath, Frame, offset_pose};
        let model = Kinematics::load("right").unwrap();
        let tool = offset_pose(
            &Pose::identity(),
            [0.025, -0.015, 0.10, 0.2, -0.3, 0.4],
            Frame::Tool,
        )
        .unwrap();
        let capture_981 = [
            1.0588989507340771,
            0.4330010897186444,
            0.02301950359586734,
            0.6914591879254376,
            1.5165053704519271,
            -0.06798913184098254,
            1.0028188649979972,
        ];
        for (name, q, tcp) in [
            ("default", [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3], tool),
            ("981", capture_981, Pose::identity()),
        ] {
            let preview = preview(&model, "right", q, tcp, PreviewRequest::default()).unwrap();
            let origin = model.fk(&q, &tcp).unwrap();
            assert_eq!(preview.program.segments.len(), 9);
            assert_eq!(preview.program.timed.len(), preview.program.segments.len());
            for (leg, segment) in preview.program.timed.iter().zip(&preview.program.segments) {
                let path = CartesianPath {
                    origin,
                    axis: segment.axis,
                    frame: Frame::Base,
                };
                for i in 0..leg.knots.len() - 1 {
                    let start = leg.knot_time(i);
                    let h = leg.knot_time(i + 1) - start;
                    for k in 0..=16 {
                        let sample = leg.sample(start + h * (k as f64 / 16.0));
                        path.check(&model.fk(&sample.q, &tcp).unwrap()).unwrap();
                        let v = model.jacobian(&sample.q, &tcp).unwrap() * SVector::from(sample.dq);
                        let speed = v.fixed_rows::<3>(0).norm();
                        assert!(speed <= preview.program.speed, "{name}: TCP {speed} m/s");
                    }
                }
            }
        }
    }

    #[test]
    fn tcp_limiting_preserves_shape_and_is_atomic_and_idempotent() {
        let motion = MotionTuning::default();
        let model = Kinematics::load("right").unwrap();
        let tcp = Pose::identity();
        let knots = [(0.0, [0.0; 7]), (0.001, [0.01; 7]), (0.002, [0.02; 7])];
        let original = TimedLeg::fit(&knots, 1.0, motion, model.speed_limits()).unwrap();
        let mut leg = original.clone();
        leg.limit_tcp_speed(&model, &tcp, 0.0001).unwrap();
        let scale = leg.duration() / original.duration();
        assert!(scale > 1.0);
        for (i, knot) in knots.iter().enumerate() {
            assert_eq!(leg.sample(leg.knot_time(i)).q, knot.1);
        }
        let t = original.duration() * 0.37;
        let (a, b) = (original.sample(t), leg.sample(t * scale));
        for j in 0..7 {
            close(a.q[j], b.q[j], 1e-14);
            close(a.dq[j] / scale, b.dq[j], 1e-14);
            close(a.ddq[j] / scale / scale, b.ddq[j], 1e-14);
        }
        let limited = leg.clone();
        leg.limit_tcp_speed(&model, &tcp, 0.0001).unwrap();
        assert_eq!(leg, limited);
        for bad in [f64::NAN, 0.0, f64::INFINITY, f64::from_bits(1)] {
            assert!(leg.limit_tcp_speed(&model, &tcp, bad).is_err());
            assert_eq!(leg, limited);
        }
    }

    #[test]
    fn validates_inputs_and_holds_outside_elapsed_range() {
        let a = (0.0, [0.0; 7]);
        let b = (1.0, [1.0; 7]);
        let motion = MotionTuning::default();
        let model = [1.0; 7];
        let fit = |k: &[(f64, [f64; 7])]| TimedLeg::fit(k, 1.0, motion, model);
        let leg = fit(&[a, b]).unwrap();
        for (t, q) in [
            (f64::NAN, a.1),
            (f64::NEG_INFINITY, a.1),
            (f64::INFINITY, b.1),
        ] {
            let sample = leg.sample(t);
            assert_eq!(sample.q, q);
            assert_eq!(sample.dq, [0.0; 7]);
            assert_eq!(sample.ddq, [0.0; 7]);
        }
        let stationary = fit(&[b, b]).unwrap();
        assert_eq!(stationary.duration(), 0.0);
        assert_eq!(stationary.sample(42.0).q, b.1);
        assert!(fit(&[]).is_err());
        assert!(fit(&[a]).is_err());
        assert!(fit(&[a, b, a]).is_err());
        assert!(fit(&[a, (a.0, b.1)]).is_err());
        assert!(fit(&[(-f64::MAX, a.1), (f64::MAX, b.1)]).is_err());
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(TimedLeg::fit(&[a, b], bad, motion, model).is_err());
            let mut invalid = model;
            invalid[6] = bad;
            assert!(TimedLeg::fit(&[a, b], 1.0, motion, invalid).is_err());
            for tuning in [
                MotionTuning {
                    joint_speed: bad,
                    ..motion
                },
                MotionTuning {
                    acceleration: bad,
                    ..motion
                },
                MotionTuning {
                    jerk: bad,
                    ..motion
                },
            ] {
                assert!(TimedLeg::fit(&[a, b], 1.0, tuning, model).is_err());
            }
        }
        for bad in [f64::NAN, f64::NEG_INFINITY, f64::INFINITY] {
            assert!(fit(&[a, (bad, b.1)]).is_err());
            assert!(fit(&[a, (b.0, [bad; 7])]).is_err());
        }
    }
}
