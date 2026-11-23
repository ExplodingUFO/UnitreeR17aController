//! Stateless velocity feedforward and friction compensation for seven joints.

use crate::protocol::{DriveMode, Setpoint, encode};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ServoTuning {
    pub friction_enabled: bool,
    pub joints: [JointTuning; 7],
}

/// Directional friction magnitudes in SI units. Fs may be smaller than Fc.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrictionCurve {
    pub coulomb: f64,
    pub static_friction: f64,
    pub viscous: f64,
    pub stribeck_velocity: f64,
}

impl Default for FrictionCurve {
    fn default() -> Self {
        Self {
            coulomb: 0.0,
            static_friction: 0.0,
            viscous: 0.0,
            stribeck_velocity: 0.02,
        }
    }
}

impl FrictionCurve {
    /// Unclipped compensation; callers supply the joint's smooth zero scale.
    pub(crate) fn evaluate(&self, velocity: f64, velocity_scale: f64) -> f64 {
        if velocity == 0.0 {
            return 0.0;
        }
        let ratio = velocity.abs() / self.stribeck_velocity;
        let decay = (-(ratio * ratio)).exp();
        // Interpolate from the smaller endpoint to avoid cancellation and
        // intermediate overflow even for the largest finite magnitudes.
        let magnitude = if self.static_friction >= self.coulomb {
            self.coulomb + (self.static_friction - self.coulomb) * decay
        } else {
            self.static_friction + (self.coulomb - self.static_friction) * (1.0 - decay)
        };
        magnitude * (velocity / velocity_scale).tanh() + self.viscous * velocity
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StribeckTuning {
    pub enabled: bool,
    pub positive: FrictionCurve,
    pub negative: FrictionCurve,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointTuning {
    pub kd: f64,
    pub velocity_ff: f64,
    /// Coulomb friction torque, in Nm.
    pub coulomb: f64,
    /// Viscous friction coefficient, in Nm s/rad.
    pub viscous: f64,
    /// Smooth zero-crossing scale, in rad/s.
    pub velocity_scale: f64,
    /// Absolute friction compensation limit, in Nm.
    pub max_friction: f64,
    #[serde(default)]
    pub stribeck: StribeckTuning,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Applied {
    pub dq_feedforward: [f64; 7],
    /// Friction after the configured max_friction limit.
    pub friction_requested: [f64; 7],
    /// Change in delivered feedforward relative to sending gravity alone.
    pub friction_applied: [f64; 7],
    pub tau: [f64; 7],
    pub clamped: [bool; 7],
}

impl Default for JointTuning {
    fn default() -> Self {
        Self {
            kd: 10.0,
            velocity_ff: 1.0,
            coulomb: 0.0,
            viscous: 0.0,
            velocity_scale: 0.02,
            max_friction: 0.5,
            stribeck: StribeckTuning::default(),
        }
    }
}

/// Fraction of the tracking Kp used at rest. Half gain at the anchor, and
/// halfway back to full gain at two degrees. These are initial software
/// settings. The restoring torque grows continuously on either side of zero.
pub(crate) fn hold_stiffness_scale(error_rad: f64) -> f64 {
    const HALF_RECOVERY_RAD: f64 = 2.0_f64.to_radians();
    1.0 - 0.5 / (1.0 + error_rad.abs() / HALF_RECOVERY_RAD)
}

impl ServoTuning {
    pub fn validate(&self) -> Result<(), String> {
        for (index, joint) in self.joints.iter().enumerate() {
            for (name, value) in [
                ("kd", joint.kd),
                ("velocity_ff", joint.velocity_ff),
                ("coulomb", joint.coulomb),
                ("viscous", joint.viscous),
                ("velocity_scale", joint.velocity_scale),
                ("max_friction", joint.max_friction),
            ] {
                if !value.is_finite() {
                    return Err(format!("J{} {name}非有限", index + 1));
                }
                if value < 0.0 || (name == "velocity_scale" && value == 0.0) {
                    return Err(format!("J{} {name}符号无效", index + 1));
                }
            }
            for (direction, curve) in [
                ("positive", joint.stribeck.positive),
                ("negative", joint.stribeck.negative),
            ] {
                for (name, value) in [
                    ("coulomb", curve.coulomb),
                    ("static_friction", curve.static_friction),
                    ("viscous", curve.viscous),
                    ("stribeck_velocity", curve.stribeck_velocity),
                ] {
                    if !value.is_finite()
                        || value < 0.0
                        || (name == "stribeck_velocity" && value == 0.0)
                    {
                        return Err(format!("J{} stribeck.{direction}.{name}无效", index + 1));
                    }
                }
            }
            // Kd is a wire field; keep its SDK and signed-integer limits in
            // the encoder instead of maintaining a second tuning range.
            encode(
                index as u8,
                Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.0,
                    dq: 0.0,
                    tau: 0.0,
                    kp: 0.0,
                    kd: joint.kd,
                },
            )
            .map_err(|error| format!("J{} kd: {error}", index + 1))?;
        }
        Ok(())
    }

    #[cfg(test)]
    pub fn apply(&self, dq: [f64; 7], gravity: [f64; 7]) -> Result<Applied, String> {
        // Exercise the compensation formula independently of the arm model.
        self.apply_with_effort_limits(dq, gravity, [f64::INFINITY; 7])
            .map(|(applied, _)| applied)
    }

    /// Apply the active model's effort ratings to the combined torque feedforward.
    /// The friction switch does not disable the independent velocity FF weight.
    /// Also return the gravity-only command for the same joint budgets.
    pub fn apply_with_effort_limits(
        &self,
        dq: [f64; 7],
        gravity: [f64; 7],
        effort_limits: [f64; 7],
    ) -> Result<(Applied, [f64; 7]), String> {
        for index in 0..7 {
            // Motion speeds are already bounded by the active profile and
            // model before this stateless feedforward calculation.
            if !dq[index].is_finite() {
                return Err(format!("J{} 速度无效", index + 1));
            }
            if !gravity[index].is_finite() {
                return Err(format!("J{} 重力无效", index + 1));
            }
        }

        let mut result = Applied {
            dq_feedforward: dq,
            friction_requested: [0.0; 7],
            friction_applied: [0.0; 7],
            tau: gravity,
            clamped: [false; 7],
        };
        let mut delivered_gravity = gravity;
        for index in 0..7 {
            let joint = self.joints[index];
            let velocity = dq[index];
            if joint.velocity_ff != 1.0 {
                result.dq_feedforward[index] = velocity * joint.velocity_ff;
            }
            if !result.dq_feedforward[index].is_finite() {
                return Err(format!("J{} 速度前馈非有限", index + 1));
            }
            if self.friction_enabled && velocity != 0.0 {
                let raw = if joint.stribeck.enabled {
                    let curve = if velocity > 0.0 {
                        joint.stribeck.positive
                    } else {
                        joint.stribeck.negative
                    };
                    curve.evaluate(velocity, joint.velocity_scale)
                } else {
                    joint.coulomb * (velocity / joint.velocity_scale).tanh()
                        + joint.viscous * velocity
                };
                let requested = raw.clamp(-joint.max_friction, joint.max_friction);
                result.clamped[index] = requested != raw;
                // Preserve gravity's bits (including -0.0) and canonical +0.0 friction.
                if requested != 0.0 {
                    result.friction_requested[index] = requested;
                    result.tau[index] += requested;
                }
            }

            let limit = effort_limits[index];
            delivered_gravity[index] = gravity[index].clamp(-limit, limit);
            let tau = result.tau[index].clamp(-limit, limit);
            result.clamped[index] |= tau != result.tau[index];
            let friction = tau - delivered_gravity[index];
            if !tau.is_finite() || !friction.is_finite() {
                return Err(format!("J{} 力矩前馈非有限", index + 1));
            }
            result.friction_applied[index] = friction;
            result.tau[index] = tau;
        }
        Ok((result, delivered_gravity))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIELDS: [(&str, f64); 6] = [
        ("kd", 50.0),
        ("velocity_ff", 2.0),
        ("coulomb", 3.0),
        ("viscous", 20.0),
        ("velocity_scale", 1.0),
        ("max_friction", 3.0),
    ];

    fn field_mut(joint: &mut JointTuning, field: usize) -> &mut f64 {
        match field {
            0 => &mut joint.kd,
            1 => &mut joint.velocity_ff,
            2 => &mut joint.coulomb,
            3 => &mut joint.viscous,
            4 => &mut joint.velocity_scale,
            5 => &mut joint.max_friction,
            _ => unreachable!(),
        }
    }

    fn enabled(joint: JointTuning) -> ServoTuning {
        ServoTuning {
            friction_enabled: true,
            joints: [joint; 7],
        }
    }

    fn assert_bits(actual: [f64; 7], expected: [f64; 7]) {
        for (index, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
            assert_eq!(actual.to_bits(), expected.to_bits(), "J{}", index + 1);
        }
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "expected {expected}, got {actual} (tolerance {tolerance})"
        );
    }

    fn friction(tuning: &ServoTuning, velocity: f64) -> f64 {
        tuning
            .apply([velocity; 7], [0.0; 7])
            .unwrap()
            .friction_requested[0]
    }

    fn asymmetric() -> StribeckTuning {
        StribeckTuning {
            enabled: true,
            positive: FrictionCurve {
                coulomb: 0.2,
                static_friction: 0.7,
                viscous: 0.12,
                stribeck_velocity: 0.06,
            },
            negative: FrictionCurve {
                coulomb: 0.5,
                static_friction: 0.1,
                viscous: 0.24,
                stribeck_velocity: 0.03,
            },
        }
    }

    #[test]
    fn legacy_json_defaults_to_disabled_curves_and_exact_old_formula() {
        let tuning = enabled(JointTuning {
            coulomb: 0.31,
            viscous: 0.27,
            max_friction: 0.6,
            ..JointTuning::default()
        });
        let mut json = serde_json::to_value(tuning).unwrap();
        for joint in json["joints"].as_array_mut().unwrap() {
            joint.as_object_mut().unwrap().remove("stribeck");
        }
        let restored: ServoTuning = serde_json::from_value(json).unwrap();
        assert_eq!(restored, tuning);
        assert_eq!(restored.joints[0].stribeck, StribeckTuning::default());
        assert_eq!(FrictionCurve::default().stribeck_velocity, 0.02);
        for velocity in [-3.0, -0.1, -1e-9, -0.0, 0.0, 1e-9, 0.05, 3.0] {
            let joint = restored.joints[0];
            let expected = if velocity == 0.0 {
                0.0
            } else {
                (joint.coulomb * (velocity / joint.velocity_scale).tanh()
                    + joint.viscous * velocity)
                    .clamp(-joint.max_friction, joint.max_friction)
            };
            assert_eq!(friction(&restored, velocity).to_bits(), expected.to_bits());
            let mut off = restored;
            off.joints[0].stribeck = asymmetric();
            off.joints[0].stribeck.enabled = false;
            assert_eq!(friction(&off, velocity).to_bits(), expected.to_bits());
        }
    }

    #[test]
    fn directional_stribeck_matches_formula_and_both_sides_approach_exact_zero() {
        let tuning = enabled(JointTuning {
            stribeck: asymmetric(),
            max_friction: 10.0,
            ..JointTuning::default()
        });
        for velocity in [-0.3_f64, -0.05, -1e-6, -1e-10, 1e-10, 1e-6, 0.05, 0.3] {
            let joint = tuning.joints[0];
            let curve = if velocity > 0.0 {
                joint.stribeck.positive
            } else {
                joint.stribeck.negative
            };
            let expected = (curve.coulomb
                + (curve.static_friction - curve.coulomb)
                    * (-(velocity.abs() / curve.stribeck_velocity).powi(2)).exp())
                * (velocity / joint.velocity_scale).tanh()
                + curve.viscous * velocity;
            let actual = friction(&tuning, velocity);
            assert_close(actual, expected, 1e-14);
            assert_eq!(actual.signum(), velocity.signum());
        }
        for velocity in [-0.0, 0.0] {
            assert_eq!(friction(&tuning, velocity).to_bits(), 0.0_f64.to_bits());
        }
        assert!(friction(&tuning, 1e-10).abs() < 4e-9);
        assert!(friction(&tuning, -1e-10).abs() < 4e-9);
        assert_ne!(friction(&tuning, 0.05), -friction(&tuning, -0.05));
    }

    #[test]
    fn directional_switches_and_limits_preserve_gravity_and_velocity_weight() {
        let mut tuning = enabled(JointTuning {
            velocity_ff: 0.25,
            stribeck: asymmetric(),
            max_friction: 0.1,
            ..JointTuning::default()
        });
        tuning.joints[2].max_friction = 0.0;
        let dq = [1.0, -1.0, 1.0, 0.0, -0.0, 0.0, 0.0];
        let gravity = [8.0, -8.0, -0.0, 0.0, -0.0, 0.0, 0.0];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(
            result.friction_requested,
            [0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        assert_eq!(&result.clamped[..3], &[true, true, true]);
        assert_eq!(result.tau[0], 8.1);
        tuning.friction_enabled = false;
        let off = tuning.apply(dq, gravity).unwrap();
        assert_bits(off.tau, gravity);
        assert_bits(off.friction_requested, [0.0; 7]);
        assert_bits(off.dq_feedforward, dq.map(|v| v * 0.25));
    }

    #[test]
    fn directional_validation_accepts_fs_below_fc_and_has_no_arbitrary_caps() {
        let mut tuning = enabled(JointTuning {
            stribeck: asymmetric(),
            ..JointTuning::default()
        });
        assert!(tuning.validate().is_ok());
        for direction in 0..2 {
            for field in 0..4 {
                for value in [
                    -1e-20,
                    f64::NAN,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    0.0,
                    f64::MAX,
                ] {
                    let mut candidate = tuning;
                    let curve = if direction == 0 {
                        &mut candidate.joints[0].stribeck.positive
                    } else {
                        &mut candidate.joints[0].stribeck.negative
                    };
                    let target = match field {
                        0 => &mut curve.coulomb,
                        1 => &mut curve.static_friction,
                        2 => &mut curve.viscous,
                        _ => &mut curve.stribeck_velocity,
                    };
                    *target = value;
                    assert_eq!(
                        candidate.validate().is_ok(),
                        value.is_finite() && value >= 0.0 && (field != 3 || value > 0.0)
                    );
                }
            }
        }
        tuning.joints[0].stribeck.positive = FrictionCurve {
            coulomb: f64::MAX,
            static_friction: f64::MAX,
            viscous: f64::MAX,
            stribeck_velocity: f64::from_bits(1),
        };
        assert!(tuning.validate().is_ok());
        assert_eq!(friction(&tuning, 2.0), tuning.joints[0].max_friction);
    }

    #[test]
    fn defaults_preserve_baseline_bits() {
        let tuning = ServoTuning::default();
        assert!(!tuning.friction_enabled);
        assert_eq!(
            tuning.joints,
            [JointTuning {
                kd: 10.0,
                velocity_ff: 1.0,
                coulomb: 0.0,
                viscous: 0.0,
                velocity_scale: 0.02,
                max_friction: 0.5,
                stribeck: StribeckTuning::default(),
            }; 7]
        );
        assert!(tuning.validate().is_ok());
        let dq = [-0.2, -0.0, 0.0, f64::from_bits(1), 0.025, 0.2, -0.125];
        let gravity = [-8.0, -0.0, 0.0, 1e-12, -3.5, 7.999999999, 8.0];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(result.dq_feedforward, dq);
        assert_bits(result.tau, gravity);
        assert_bits(result.friction_requested, [0.0; 7]);
        assert_bits(result.friction_applied, [0.0; 7]);
        assert_eq!(result.clamped, [false; 7]);
    }

    #[test]
    fn global_off_ignores_nonzero_friction_coefficients() {
        let mut tuning = enabled(JointTuning {
            coulomb: 2.0,
            viscous: 10.0,
            max_friction: 0.01,
            ..JointTuning::default()
        });
        tuning.friction_enabled = false;
        let dq = [-0.2, 0.2, -0.1, 0.1, -0.0, 0.0, 0.05];
        let gravity = [-8.0, 8.0, 2.0, -2.0, -0.0, 0.0, 7.9];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(result.dq_feedforward, dq);
        assert_bits(result.tau, gravity);
        assert_bits(result.friction_requested, [0.0; 7]);
        assert_bits(result.friction_applied, [0.0; 7]);
        assert_eq!(result.clamped, [false; 7]);
    }

    #[test]
    fn zero_velocity_has_exact_zero_friction_with_compensation_enabled() {
        let tuning = enabled(JointTuning {
            coulomb: 2.0,
            viscous: 10.0,
            ..JointTuning::default()
        });
        let dq = [-0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0];
        let gravity = [-8.0, 8.0, -0.0, 0.0, -2.0, 2.0, 0.5];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(result.dq_feedforward, dq);
        assert_bits(result.tau, gravity);
        assert_bits(result.friction_requested, [0.0; 7]);
        assert_bits(result.friction_applied, [0.0; 7]);
        assert_eq!(result.clamped, [false; 7]);
    }

    #[test]
    fn velocity_feedforward_weight_is_independent_of_the_friction_switch() {
        let dq = [-0.2, -0.1, 0.08, 0.12, -0.04, -0.0, 0.2];
        let weights = [0.0, 0.125, 0.25, 0.5, 0.75, 1.0, 1.0];
        let gravity = [1.0, -1.0, 2.0, -2.0, 0.0, -0.0, 8.0];
        let mut tuning = ServoTuning::default();
        for (joint, weight) in tuning.joints.iter_mut().zip(weights) {
            joint.velocity_ff = weight;
            joint.kd = 30.0;
        }
        for flag in [false, true] {
            tuning.friction_enabled = flag;
            let result = tuning.apply(dq, gravity).unwrap();
            assert_bits(
                result.dq_feedforward,
                std::array::from_fn(|i| dq[i] * weights[i]),
            );
            assert_bits(result.tau, gravity);
            assert_bits(result.friction_requested, [0.0; 7]);
        }
    }

    #[test]
    fn friction_is_odd_and_continuous_through_zero() {
        let tuning = enabled(JointTuning {
            coulomb: 0.4,
            viscous: 0.8,
            velocity_scale: 0.03,
            max_friction: 2.0,
            ..JointTuning::default()
        });
        for velocity in [0.2, 0.1, 0.01, 1e-6, 1e-10] {
            let positive = friction(&tuning, velocity);
            let negative = friction(&tuning, -velocity);
            assert!(positive > 0.0 && negative < 0.0);
            assert_close(positive, -negative, 1e-14);
        }
        assert_eq!(friction(&tuning, 0.0).to_bits(), 0.0f64.to_bits());
        assert_eq!(friction(&tuning, -0.0).to_bits(), 0.0f64.to_bits());
        assert!(friction(&tuning, 1e-10) < 2e-9);
        assert!(friction(&tuning, 1e-10) < friction(&tuning, 1e-6));
    }

    #[test]
    fn unclipped_friction_has_finite_partial_derivatives() {
        let tuning = enabled(JointTuning {
            coulomb: 0.3,
            viscous: 0.7,
            velocity_scale: 0.04,
            max_friction: 2.0,
            ..JointTuning::default()
        });
        let step = 1e-7;
        for velocity in [-0.05, 0.0, 0.05] {
            let joint = tuning.joints[0];
            let tanh = (velocity / joint.velocity_scale).tanh();
            let sech_squared = 1.0 - tanh * tanh;
            let derivative = (friction(&tuning, velocity + step)
                - friction(&tuning, velocity - step))
                / (2.0 * step);
            assert_close(
                derivative,
                joint.coulomb / joint.velocity_scale * sech_squared + joint.viscous,
                1e-7,
            );
            for (field, expected) in [
                (2, tanh),
                (3, velocity),
                (
                    4,
                    -joint.coulomb * velocity / joint.velocity_scale.powi(2) * sech_squared,
                ),
            ] {
                let mut above = tuning;
                let mut below = tuning;
                *field_mut(&mut above.joints[0], field) += step;
                *field_mut(&mut below.joints[0], field) -= step;
                let derivative =
                    (friction(&above, velocity) - friction(&below, velocity)) / (2.0 * step);
                assert_close(derivative, expected, 1e-7);
            }
        }
    }

    #[test]
    fn friction_limit_clips_both_signs_and_zero_limit_delivers_exact_zero() {
        let mut tuning = enabled(JointTuning {
            coulomb: 1.0,
            viscous: 5.0,
            max_friction: 0.25,
            ..JointTuning::default()
        });
        tuning.joints[2].max_friction = 0.0;
        tuning.joints[3].max_friction = 0.0;
        let dq = [0.2, -0.2, 0.2, -0.2, 0.0, 0.0, 0.0];
        let gravity = [0.0, 0.0, -0.0, -8.0, 0.0, 0.0, 0.0];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(
            result.friction_requested,
            [0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        assert_bits(result.friction_applied, result.friction_requested);
        assert_bits(result.tau, [0.25, -0.25, -0.0, -8.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            result.clamped,
            [true, true, true, true, false, false, false]
        );
    }

    #[test]
    fn friction_adds_to_gravity_without_a_second_torque_cap() {
        let tuning = enabled(JointTuning {
            viscous: 5.0,
            max_friction: 2.0,
            ..JointTuning::default()
        });
        let dq = [0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.0];
        let gravity = [7.75, -7.75, -7.75, 7.75, 8.0, -8.0, -0.0];
        let result = tuning.apply(dq, gravity).unwrap();
        assert_bits(
            result.friction_requested,
            [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0],
        );
        assert_bits(result.tau, [8.75, -8.75, -6.75, 6.75, 9.0, -9.0, -0.0]);
        assert_bits(
            result.friction_applied,
            [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0],
        );
        assert_eq!(result.clamped, [false; 7]);
        for (index, gravity) in gravity.into_iter().enumerate() {
            assert_eq!(result.friction_applied[index], result.tau[index] - gravity);
            assert_eq!(
                result.tau[index],
                gravity + result.friction_requested[index]
            );
        }
    }

    #[test]
    fn reaching_a_limit_without_clipping_does_not_set_clamped() {
        let tuning = enabled(JointTuning {
            viscous: 1.0,
            max_friction: 0.125,
            ..JointTuning::default()
        });
        let result = tuning
            .apply(
                [0.125, -0.125, 0.0, 0.0, 0.0, 0.0, 0.0],
                [7.875, -7.875, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            .unwrap();
        assert_bits(result.tau, [8.0, -8.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(result.clamped, [false; 7]);
    }

    #[test]
    fn changing_one_joint_never_changes_another_joint() {
        let original = enabled(JointTuning::default());
        let dq = [0.2, -0.18, 0.16, -0.14, 0.12, -0.1, 0.08];
        let gravity = [8.0, -8.0, 3.0, -3.0, 1.0, -1.0, 0.0];
        let baseline = original.apply(dq, gravity).unwrap();
        for changed in 0..7 {
            let mut tuning = original;
            tuning.joints[changed] = JointTuning {
                kd: 25.0,
                velocity_ff: 0.25,
                coulomb: 0.4,
                viscous: 2.0,
                velocity_scale: 0.05,
                max_friction: 0.3,
                stribeck: StribeckTuning::default(),
            };
            let result = tuning.apply(dq, gravity).unwrap();
            assert_eq!(result.dq_feedforward[changed], dq[changed] * 0.25);
            assert_ne!(result.friction_requested[changed], 0.0);
            for index in (0..7).filter(|index| *index != changed) {
                assert_eq!(
                    result.dq_feedforward[index].to_bits(),
                    baseline.dq_feedforward[index].to_bits()
                );
                assert_eq!(
                    result.friction_requested[index].to_bits(),
                    baseline.friction_requested[index].to_bits()
                );
                assert_eq!(
                    result.friction_applied[index].to_bits(),
                    baseline.friction_applied[index].to_bits()
                );
                assert_eq!(result.tau[index].to_bits(), baseline.tau[index].to_bits());
                assert_eq!(result.clamped[index], baseline.clamped[index]);
            }
        }
    }

    #[test]
    fn tuning_uses_mathematical_signs_instead_of_software_caps() {
        for index in 0..7 {
            for (field, (name, above_old_cap)) in FIELDS.into_iter().enumerate() {
                for allowed in [
                    if name == "velocity_scale" {
                        0.0001
                    } else {
                        0.0
                    },
                    above_old_cap,
                ] {
                    let mut tuning = ServoTuning::default();
                    *field_mut(&mut tuning.joints[index], field) = allowed;
                    assert!(tuning.validate().is_ok(), "J{} {name}={allowed}", index + 1);
                }
                let mut tuning = ServoTuning::default();
                *field_mut(&mut tuning.joints[index], field) = -1e-9;
                let error = tuning.validate().unwrap_err();
                assert!(error.starts_with(&format!("J{} ", index + 1)));
                assert!(error.contains(name));
            }
            let mut tuning = ServoTuning::default();
            tuning.joints[index].velocity_scale = 0.0;
            assert!(tuning.validate().is_err());
        }
    }

    #[test]
    fn every_tuning_field_rejects_nan_and_infinity() {
        for index in 0..7 {
            for field in 0..FIELDS.len() {
                for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                    let mut tuning = ServoTuning::default();
                    *field_mut(&mut tuning.joints[index], field) = invalid;
                    let error = tuning.validate().unwrap_err();
                    assert!(error.starts_with(&format!("J{} ", index + 1)));
                }
            }
        }
    }

    #[test]
    fn sampled_inputs_reject_nonfinite_velocity_and_invalid_gravity() {
        let tuning = ServoTuning::default();
        for index in 0..7 {
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut dq = [0.0; 7];
                dq[index] = invalid;
                let error = tuning.apply(dq, [0.0; 7]).unwrap_err();
                assert!(error.starts_with(&format!("J{} ", index + 1)));
            }
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut gravity = [0.0; 7];
                gravity[index] = invalid;
                let error = tuning.apply([0.0; 7], gravity).unwrap_err();
                assert!(error.starts_with(&format!("J{} ", index + 1)));
            }
        }
        for velocity in [-1.0, -0.6, -0.21, 0.21, 0.6, 1.0] {
            let result = tuning.apply([velocity; 7], [8.0; 7]).unwrap();
            assert_bits(result.dq_feedforward, [velocity; 7]);
        }
        for gravity in [8.0 + 1e-12, -8.0 - 1e-12, -32.0, 32.0, f64::MAX] {
            let result = tuning.apply([0.0; 7], [gravity; 7]).unwrap();
            assert_bits(result.tau, [gravity; 7]);
        }
    }

    #[test]
    fn finite_parameters_must_still_produce_finite_velocity_feedforward() {
        let mut tuning = ServoTuning::default();
        tuning.joints[0].velocity_ff = f64::MAX;
        assert!(tuning.validate().is_ok());
        let error = tuning.apply([2.0; 7], [0.0; 7]).unwrap_err();
        assert!(error.starts_with("J1 "));
        assert!(error.contains("非有限"));
        tuning.joints[0].velocity_ff = 1.0;
        tuning.joints[0].velocity_scale = f64::from_bits(1);
        assert!(tuning.apply([2.0; 7], [0.0; 7]).is_ok());
    }

    #[test]
    fn finite_parameters_must_still_produce_finite_torque() {
        let tuning = enabled(JointTuning {
            coulomb: f64::MAX,
            max_friction: f64::MAX,
            ..JointTuning::default()
        });
        assert!(tuning.validate().is_ok());
        for sign in [-1.0, 1.0] {
            let error = tuning.apply([sign; 7], [sign * f64::MAX; 7]).unwrap_err();
            assert!(error.starts_with("J1 "));
            assert!(error.contains("非有限"));
        }
    }

    #[test]
    fn serde_requires_every_field_but_allows_unknown_fields() {
        let tuning = ServoTuning::default();
        let full = serde_json::to_value(tuning).unwrap();
        assert_eq!(
            serde_json::from_value::<ServoTuning>(full.clone()).unwrap(),
            tuning
        );
        for name in ["friction_enabled", "joints"] {
            let mut missing = full.clone();
            missing.as_object_mut().unwrap().remove(name);
            assert!(serde_json::from_value::<ServoTuning>(missing).is_err());
        }
        for index in 0..7 {
            for (name, _) in FIELDS {
                let mut missing = full.clone();
                missing["joints"][index]
                    .as_object_mut()
                    .unwrap()
                    .remove(name);
                assert!(serde_json::from_value::<ServoTuning>(missing).is_err());
            }
        }
        let mut extended = full;
        extended["future_field"] = serde_json::json!(true);
        extended["joints"][0]["future_field"] = serde_json::json!(42);
        assert_eq!(
            serde_json::from_value::<ServoTuning>(extended).unwrap(),
            tuning
        );
    }

    #[test]
    fn serde_enforces_seven_joint_entries() {
        for length in [0, 6, 8] {
            let mut value = serde_json::to_value(ServoTuning::default()).unwrap();
            let joint = value["joints"][0].clone();
            value["joints"] = serde_json::json!(vec![joint; length]);
            assert!(serde_json::from_value::<ServoTuning>(value).is_err());
        }
    }

    #[test]
    fn applied_serializes_the_fixed_fields_and_seven_element_arrays() {
        let defaults = Applied::default();
        assert_bits(defaults.dq_feedforward, [0.0; 7]);
        assert_bits(defaults.friction_requested, [0.0; 7]);
        assert_bits(defaults.friction_applied, [0.0; 7]);
        assert_bits(defaults.tau, [0.0; 7]);
        assert_eq!(defaults.clamped, [false; 7]);
        let result = ServoTuning::default().apply([0.1; 7], [2.0; 7]).unwrap();
        let value = serde_json::to_value(result).unwrap();
        assert_eq!(value.as_object().unwrap().len(), 5);
        let restored: Applied = serde_json::from_value(value.clone()).unwrap();
        assert_bits(restored.dq_feedforward, result.dq_feedforward);
        assert_bits(restored.friction_requested, result.friction_requested);
        assert_bits(restored.friction_applied, result.friction_applied);
        assert_bits(restored.tau, result.tau);
        assert_eq!(restored.clamped, result.clamped);
        for name in [
            "dq_feedforward",
            "friction_requested",
            "friction_applied",
            "tau",
            "clamped",
        ] {
            for length in [6, 8] {
                let mut invalid = value.clone();
                let item = value[name][0].clone();
                invalid[name] = serde_json::json!(vec![item; length]);
                assert!(serde_json::from_value::<Applied>(invalid).is_err());
            }
        }
    }
}
