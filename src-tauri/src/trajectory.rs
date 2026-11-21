//! Discrete command derivatives. Limits apply to the velocity actually integrated,
//! including unequal control intervals; no filtering is applied to measured joints.
use nalgebra::{SMatrix, SVector};

pub const JOINT_JERK: f64 = 6.0;
pub const JOINT_ACCELERATION: f64 = 0.60;
pub const JOINT_SPEED: f64 = 0.20;

#[derive(Clone, Copy, Debug)]
pub struct VelocityWindow {
    pub low: [f64; 7],
    pub high: [f64; 7],
}

/// Largest acceleration whose velocity increment and subsequent jerk-limited
/// removal fit inside `distance` in velocity space. The rationalized root avoids
/// cancellation near the target. This is a braking envelope, not a filter.
fn brake_acceleration(distance: f64, jerk: f64, dt: f64) -> f64 {
    let jd = jerk * dt;
    2.0 * jerk * distance.max(0.0) / ((jd * jd + 2.0 * jerk * distance.max(0.0)).sqrt() + jd)
}

pub fn velocity_window(
    velocity: [f64; 7],
    acceleration: [f64; 7],
    target: [f64; 7],
    speed: [f64; 7],
    max_acceleration: [f64; 7],
    jerk: [f64; 7],
    dt: f64,
) -> Result<VelocityWindow, String> {
    if !dt.is_finite() || !(0.0001..=0.05).contains(&dt) {
        return Err("invalid trajectory period".into());
    }
    let mut low = [0.0; 7];
    let mut high = [0.0; 7];
    for i in 0..7 {
        if [
            velocity[i],
            acceleration[i],
            target[i],
            speed[i],
            max_acceleration[i],
            jerk[i],
        ]
        .iter()
        .any(|x| !x.is_finite())
            || speed[i] <= 0.0
            || max_acceleration[i] <= 0.0
            || jerk[i] <= 0.0
        {
            return Err("invalid trajectory state or limits".into());
        }
        let a_low = (-max_acceleration[i])
            .max(acceleration[i] - jerk[i] * dt)
            .max(-brake_acceleration(speed[i] + velocity[i], jerk[i], dt));
        let a_high = max_acceleration[i]
            .min(acceleration[i] + jerk[i] * dt)
            .min(brake_acceleration(speed[i] - velocity[i], jerk[i], dt));
        low[i] = (-speed[i]).max(velocity[i] + a_low * dt);
        high[i] = speed[i].min(velocity[i] + a_high * dt);
        if low[i] > high[i] {
            return Err("trajectory state cannot satisfy derivative limits".into());
        }
        // Approach a constant velocity with zero acceleration. A newly changed
        // goal may make that envelope temporarily infeasible; global hard limits
        // still apply while acceleration is brought back toward the new goal.
        let target = target[i].clamp(-speed[i], speed[i]);
        let difference = target - velocity[i];
        if difference >= 0.0 {
            let cap = velocity[i] + brake_acceleration(difference, jerk[i], dt) * dt;
            if cap >= low[i] - 1e-12 {
                high[i] = high[i].min(cap.max(low[i]));
            }
        }
        if difference <= 0.0 {
            let cap = velocity[i] - brake_acceleration(-difference, jerk[i], dt) * dt;
            if cap <= high[i] + 1e-12 {
                low[i] = low[i].max(cap.min(high[i]));
            }
        }
    }
    Ok(VelocityWindow { low, high })
}

pub fn integrate(
    position: [f64; 7],
    previous_velocity: [f64; 7],
    velocity: [f64; 7],
    dt: f64,
) -> ([f64; 7], [f64; 7]) {
    (
        std::array::from_fn(|i| position[i] + velocity[i] * dt),
        std::array::from_fn(|i| (velocity[i] - previous_velocity[i]) / dt),
    )
}

/// Strictly convex seven-variable box QP. A bounded active-set solve avoids
/// per-axis post-filtering of the Cartesian inverse-kinematics result.
pub fn bounded_quadratic(
    h: SMatrix<f64, 7, 7>,
    gradient: SVector<f64, 7>,
    window: VelocityWindow,
    seed: [f64; 7],
) -> Result<[f64; 7], String> {
    if !h.iter().chain(gradient.iter()).all(|x| x.is_finite()) {
        return Err("non-finite trajectory objective".into());
    }
    if (0..7).any(|i| {
        !seed[i].is_finite()
            || !window.low[i].is_finite()
            || !window.high[i].is_finite()
            || window.low[i] > window.high[i]
    }) {
        return Err("invalid trajectory velocity window".into());
    }
    if (h - h.transpose()).norm() > 1e-10 * h.norm().max(1.0) || h.cholesky().is_none() {
        return Err("trajectory objective must be symmetric positive definite".into());
    }
    let mut x = SVector::<f64, 7>::from_fn(|i, _| seed[i].clamp(window.low[i], window.high[i]));
    let mut active = [0i8; 7];
    for i in 0..7 {
        if window.high[i] - window.low[i] < 1e-14 {
            active[i] = 2;
        } else if x[i] <= window.low[i] + 1e-14 {
            active[i] = -1;
        } else if x[i] >= window.high[i] - 1e-14 {
            active[i] = 1;
        }
    }
    for _ in 0..96 {
        let g = h * x + gradient;
        let mut system = h;
        let mut rhs = -g;
        for i in 0..7 {
            if active[i] != 0 {
                for j in 0..7 {
                    system[(i, j)] = 0.0;
                    system[(j, i)] = 0.0;
                }
                system[(i, i)] = 1.0;
                rhs[i] = 0.0;
            }
        }
        let p = system
            .cholesky()
            .ok_or("trajectory QP factorization failed")?
            .solve(&rhs);
        if !p.iter().all(|v| v.is_finite()) {
            return Err("invalid trajectory QP step".into());
        }
        // Even a tiny free-variable correction must be applied before checking
        // bound multipliers: the weighted Hessian can turn a 1e-11 velocity
        // correction into a material residual and otherwise cause cycling.
        if p.norm() > 0.0 {
            let mut amount: f64 = 1.0;
            let mut blocking = None;
            for i in 0..7 {
                if active[i] != 0 {
                    continue;
                }
                let distance = if p[i] > 1e-15 {
                    (window.high[i] - x[i]) / p[i]
                } else if p[i] < -1e-15 {
                    (window.low[i] - x[i]) / p[i]
                } else {
                    continue;
                };
                if distance < amount {
                    amount = distance.max(0.0);
                    blocking = Some((i, if p[i] > 0.0 { 1 } else { -1 }));
                }
            }
            x += p * amount;
            for i in 0..7 {
                x[i] = x[i].clamp(window.low[i], window.high[i]);
            }
            if let Some((i, bound)) = blocking {
                active[i] = bound;
                continue;
            }
        }
        let g = h * x + gradient;
        let release = (0..7)
            .filter(|i| active[*i].abs() == 1)
            .map(|i| {
                (
                    i,
                    (if active[i] == -1 { -g[i] } else { g[i] }) / h[(i, i)].max(1e-30),
                )
            })
            .max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((i, violation)) = release
            && violation > 1e-10
        {
            active[i] = 0;
        } else {
            return Ok(std::array::from_fn(|i| x[i]));
        }
    }
    Err("trajectory QP iteration limit".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changing_velocity_goals_keep_jerk_bounded_at_low_speed_and_at_the_cap() {
        let mut v = [0.0; 7];
        let mut a = [0.0; 7];
        let mut random = 17u64;
        for tick in 0..12_000 {
            random = random.wrapping_mul(6364136223846793005).wrapping_add(1);
            let dt = [0.001, 0.002, 0.007, 0.01, 0.04, 0.049][(random as usize) % 6];
            let goal = [0.001, 0.2, -0.2, 0.0, -0.001, 0.05][(tick / 193) % 6];
            let target = std::array::from_fn(|i| if i % 2 == 0 { goal } else { -goal });
            let window = velocity_window(
                v,
                a,
                target,
                [JOINT_SPEED; 7],
                [JOINT_ACCELERATION; 7],
                [JOINT_JERK; 7],
                dt,
            )
            .unwrap();
            let next = std::array::from_fn(|i| target[i].clamp(window.low[i], window.high[i]));
            let next_a: [f64; 7] = std::array::from_fn(|i| (next[i] - v[i]) / dt);
            for i in 0..7 {
                assert!(next[i].abs() <= JOINT_SPEED + 1e-10);
                assert!(next_a[i].abs() <= JOINT_ACCELERATION + 1e-10);
                assert!(
                    (next_a[i] - a[i]).abs() <= JOINT_JERK * dt + 1e-9,
                    "tick={tick} dt={dt} v={} a={} next_a={}",
                    v[i],
                    a[i],
                    next_a[i]
                );
            }
            v = next;
            a = next_a;
        }
    }

    #[test]
    fn quadratic_solution_satisfies_box_optimality_including_fixed_axes() {
        for coupling in [-0.9, 0.0, 0.9] {
            let mut h = SMatrix::<f64, 7, 7>::identity();
            h[(0, 1)] = coupling;
            h[(1, 0)] = coupling;
            let gradient =
                SVector::<f64, 7>::from_row_slice(&[-2.0, 0.4, -0.7, 1.2, 0.0, 0.4, 2.0]);
            let low = [-0.3, -0.2, -1.0, -0.5, 0.0, -0.1, -0.2];
            let high = [0.2, 0.1, 1.0, 0.5, 0.0, 0.1, 0.2];
            for seed in [[0.0; 7], low, high] {
                let x = bounded_quadratic(h, gradient, VelocityWindow { low, high }, seed).unwrap();
                let derivative = h * SVector::from_row_slice(&x) + gradient;
                for i in 0..7 {
                    assert!(x[i] >= low[i] && x[i] <= high[i]);
                    if low[i] == high[i] {
                        continue;
                    }
                    if (x[i] - low[i]).abs() < 1e-9 {
                        assert!(derivative[i] >= -1e-9);
                    } else if (x[i] - high[i]).abs() < 1e-9 {
                        assert!(derivative[i] <= 1e-9);
                    } else {
                        assert!(derivative[i].abs() < 1e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn rejects_invalid_periods_limits_and_nonfinite_state() {
        for dt in [0.0, -0.01, 0.051, f64::NAN] {
            assert!(
                velocity_window(
                    [0.0; 7], [0.0; 7], [0.0; 7], [0.2; 7], [0.6; 7], [6.0; 7], dt
                )
                .is_err()
            );
        }
        for target in [[f64::NAN; 7], [f64::INFINITY; 7]] {
            assert!(
                velocity_window(
                    [0.0; 7], [0.0; 7], target, [0.2; 7], [0.6; 7], [6.0; 7], 0.01
                )
                .is_err()
            );
        }
        assert!(
            velocity_window(
                [0.0; 7], [0.0; 7], [0.0; 7], [0.0; 7], [0.6; 7], [6.0; 7], 0.01
            )
            .is_err()
        );
    }
    #[test]
    fn start_stop_and_reverse_obey_final_discrete_derivatives_with_jitter() {
        let mut v = [0.0; 7];
        let mut a = [0.0; 7];
        for target in [0.12, 0.0, -0.18, 0.0] {
            for dt in [0.040, 0.002, 0.007, 0.003].into_iter().cycle().take(1500) {
                let window = velocity_window(
                    v,
                    a,
                    [target; 7],
                    [JOINT_SPEED; 7],
                    [JOINT_ACCELERATION; 7],
                    [JOINT_JERK; 7],
                    dt,
                )
                .unwrap();
                let next = std::array::from_fn(|i| target.clamp(window.low[i], window.high[i]));
                let next_a: [f64; 7] = std::array::from_fn(|i| (next[i] - v[i]) / dt);
                for i in 0..7 {
                    assert!(next[i].abs() <= JOINT_SPEED + 1e-10);
                    assert!(next_a[i].abs() <= JOINT_ACCELERATION + 1e-10);
                    assert!((next_a[i] - a[i]).abs() <= JOINT_JERK * dt + 1e-9);
                }
                v = next;
                a = next_a;
            }
            assert!((v[0] - target).abs() < 1e-7, "target {target}: {v:?} {a:?}");
        }
    }
}
