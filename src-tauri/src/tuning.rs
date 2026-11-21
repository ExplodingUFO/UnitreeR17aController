//! Stateless velocity feedforward and friction compensation for seven joints.

use serde::{Deserialize, Serialize};

const TORQUE_LIMIT: f64 = 8.0;
const VELOCITY_LIMIT: f64 = 0.2;
// Accept roundoff at the planner's velocity boundary, in rad/s.
const VELOCITY_TOLERANCE: f64 = 1e-9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ServoTuning {
    pub friction_enabled: bool,
    pub joints: [JointTuning; 7],
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
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Applied {
    pub dq_feedforward: [f64; 7],
    /// Friction after its own limit, before the total torque limit.
    pub friction_requested: [f64; 7],
    /// Delivered torque minus the supplied gravity torque.
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
        }
    }
}

impl ServoTuning {
    pub fn validate(&self) -> Result<(), String> {
        for (index, joint) in self.joints.iter().enumerate() {
            for (name, value, min, max) in [
                ("kd", joint.kd, 0.0, 30.0),
                ("velocity_ff", joint.velocity_ff, 0.0, 1.0),
                ("coulomb", joint.coulomb, 0.0, 2.0),
                ("viscous", joint.viscous, 0.0, 10.0),
                ("velocity_scale", joint.velocity_scale, 0.001, 0.2),
                ("max_friction", joint.max_friction, 0.0, 2.0),
            ] {
                if !value.is_finite() {
                    return Err(format!("J{} {name}非有限", index + 1));
                }
                if !(min..=max).contains(&value) {
                    return Err(format!("J{} {name}超限", index + 1));
                }
            }
        }
        Ok(())
    }

    /// Compute from the sampled motion velocity, without changing the motion plan.
    /// The friction switch does not disable the independent velocity FF weight.
    pub fn apply(&self, dq: [f64; 7], gravity: [f64; 7]) -> Result<Applied, String> {
        self.validate()?;
        for index in 0..7 {
            if !dq[index].is_finite() || dq[index].abs() > VELOCITY_LIMIT + VELOCITY_TOLERANCE {
                return Err(format!("J{} 速度无效", index + 1));
            }
            if !gravity[index].is_finite() || gravity[index].abs() > TORQUE_LIMIT {
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
        for index in 0..7 {
            let joint = self.joints[index];
            let velocity = dq[index];
            if joint.velocity_ff != 1.0 {
                result.dq_feedforward[index] = velocity * joint.velocity_ff;
            }
            if !self.friction_enabled || velocity == 0.0 {
                continue;
            }

            let raw =
                joint.coulomb * (velocity / joint.velocity_scale).tanh() + joint.viscous * velocity;
            let requested = raw.clamp(-joint.max_friction, joint.max_friction);
            result.clamped[index] = raw < -joint.max_friction || raw > joint.max_friction;
            // Preserve gravity's bits (including -0.0) and canonical +0.0 friction.
            if requested == 0.0 {
                continue;
            }

            let total = gravity[index] + requested;
            let tau = total.clamp(-TORQUE_LIMIT, TORQUE_LIMIT);
            result.friction_requested[index] = requested;
            result.friction_applied[index] = tau - gravity[index];
            result.tau[index] = tau;
            result.clamped[index] |= total < -TORQUE_LIMIT || total > TORQUE_LIMIT;
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIELDS: [(&str, f64, f64); 6] = [
        ("kd", 0.0, 30.0),
        ("velocity_ff", 0.0, 1.0),
        ("coulomb", 0.0, 2.0),
        ("viscous", 0.0, 10.0),
        ("velocity_scale", 0.001, 0.2),
        ("max_friction", 0.0, 2.0),
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
    fn total_torque_cap_reports_actual_delivered_friction() {
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
        assert_bits(result.tau, [8.0, -8.0, -6.75, 6.75, 8.0, -8.0, -0.0]);
        assert_bits(
            result.friction_applied,
            [0.25, -0.25, 1.0, -1.0, 0.0, 0.0, 0.0],
        );
        assert_eq!(
            result.clamped,
            [true, true, false, false, true, true, false]
        );
        for (index, gravity) in gravity.into_iter().enumerate() {
            assert_eq!(result.friction_applied[index], result.tau[index] - gravity);
            assert!(result.tau[index].abs() <= 8.0);
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
    fn all_tuning_ranges_accept_endpoints_and_reject_values_outside() {
        for index in 0..7 {
            for (field, (name, min, max)) in FIELDS.into_iter().enumerate() {
                for endpoint in [min, max] {
                    let mut tuning = ServoTuning::default();
                    *field_mut(&mut tuning.joints[index], field) = endpoint;
                    assert!(
                        tuning.validate().is_ok(),
                        "J{} {name}={endpoint}",
                        index + 1
                    );
                }
                for outside in [min - 1e-9, max + 1e-9] {
                    let mut tuning = ServoTuning::default();
                    *field_mut(&mut tuning.joints[index], field) = outside;
                    let error = tuning.validate().unwrap_err();
                    assert!(error.starts_with(&format!("J{} ", index + 1)));
                    assert!(error.contains(name));
                    assert!(tuning.apply([0.0; 7], [0.0; 7]).is_err());
                }
            }
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
                    assert!(tuning.apply([0.0; 7], [0.0; 7]).is_err());
                }
            }
        }
    }

    #[test]
    fn sampled_inputs_reject_nonfinite_values_and_out_of_range_values() {
        let tuning = ServoTuning::default();
        for index in 0..7 {
            for invalid in [
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
                0.2 + 2.0 * VELOCITY_TOLERANCE,
                -0.2 - 2.0 * VELOCITY_TOLERANCE,
            ] {
                let mut dq = [0.0; 7];
                dq[index] = invalid;
                let error = tuning.apply(dq, [0.0; 7]).unwrap_err();
                assert!(error.starts_with(&format!("J{} ", index + 1)));
            }
            for invalid in [
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
                8.0 + 1e-12,
                -8.0 - 1e-12,
            ] {
                let mut gravity = [0.0; 7];
                gravity[index] = invalid;
                let error = tuning.apply([0.0; 7], gravity).unwrap_err();
                assert!(error.starts_with(&format!("J{} ", index + 1)));
            }
        }
        for endpoint in [-0.2 - VELOCITY_TOLERANCE, 0.2 + VELOCITY_TOLERANCE] {
            let result = tuning.apply([endpoint; 7], [8.0; 7]).unwrap();
            assert_bits(result.dq_feedforward, [endpoint; 7]);
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
            for (name, _, _) in FIELDS {
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
