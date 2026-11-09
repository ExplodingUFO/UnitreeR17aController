//! Fixed R1-7A local kinematics. SI units; geometric twists are [linear, angular]
//! at the TCP, expressed in base axes. This module has no control-loop state.
use nalgebra::{
    Isometry3, Quaternion, SMatrix, SVector, Translation3, Unit, UnitQuaternion, Vector3,
};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

type Jacobian = SMatrix<f64, 6, 7>;
type Twist = SVector<f64, 6>;
// Proportional feedback approaches a fixed local target smoothly rather than
// demanding the entire remaining displacement within one control interval.
const POSE_GAIN: f64 = 4.0;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    pub translation: [f64; 3],
    /// Quaternion components in x, y, z, w order.
    pub rotation: [f64; 4],
}

pub type TcpTransform = Pose;

impl Pose {
    #[cfg(test)]
    pub fn identity() -> Self {
        Self {
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Frame {
    Base,
    Tool,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MotionLimits {
    pub joint_velocity: [f64; 7],
    pub joint_acceleration: [f64; 7],
    pub max_reference_error: f64,
    pub max_joint_step: f64,
    pub linear_speed: f64,
    pub angular_speed: f64,
}

#[derive(Clone, Debug)]
pub enum LocalTarget {
    Velocity { twist: [f64; 6], frame: Frame },
    Pose(Pose),
}

#[derive(Clone, Debug)]
pub struct StepInput {
    pub reference: [f64; 7],
    pub measured: [f64; 7],
    pub previous_velocity: [f64; 7],
    pub target: LocalTarget,
    pub dt: f64,
    pub tcp: TcpTransform,
    pub limits: MotionLimits,
}

#[derive(Clone, Debug, Serialize)]
pub struct StepCandidate {
    pub q: [f64; 7],
    pub dq: [f64; 7],
    /// Achieved projection along the requested twist, including all scaling.
    pub speed_scale: f64,
}

#[derive(Clone, Debug)]
struct Joint {
    name: String,
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
    lower: f64,
    upper: f64,
    velocity: f64,
}

#[derive(Clone, Debug)]
pub struct Kinematics {
    joints: [Joint; 7],
    tip: Isometry3<f64>,
}

impl Kinematics {
    pub fn load(side: &str) -> Result<Self, String> {
        let xml = match side {
            "right" => include_str!("../../resources/models/right/robot.urdf"),
            "left" => include_str!("../../resources/models/left/robot.urdf"),
            _ => return Err("side must be right or left".into()),
        };
        Self::from_urdf(xml)
    }

    fn from_urdf(xml: &str) -> Result<Self, String> {
        // Plain embedded URDF only: never expand entities, xacro, URLs or commands.
        let upper = xml.to_ascii_uppercase();
        if xml.len() > 1_000_000 || upper.contains("<!DOCTYPE") || upper.contains("<!ENTITY") {
            return Err("unsupported URDF declarations or size".into());
        }
        let robot = urdf_rs::read_from_string(xml).map_err(|e| format!("URDF: {e}"))?;
        if robot.joints.len() > 32 || robot.links.len() != robot.joints.len() + 1 {
            return Err("expected a single fixed seven-axis chain".into());
        }
        let links: HashSet<_> = robot.links.iter().map(|l| l.name.as_str()).collect();
        if links.len() != robot.links.len()
            || !links.contains("base_link")
            || !links.contains("Link7")
        {
            return Err("URDF link names do not identify base_link and Link7".into());
        }
        let mut names = HashSet::new();
        let mut children = HashSet::new();
        let mut parents = HashSet::new();
        for joint in &robot.joints {
            if !names.insert(&joint.name)
                || !parents.insert(&joint.parent.link)
                || !children.insert(&joint.child.link)
                || !links.contains(joint.parent.link.as_str())
                || !links.contains(joint.child.link.as_str())
                || joint.mimic.is_some()
            {
                return Err("duplicate, branched, missing or mimic URDF joint".into());
            }
        }
        if children.contains(&"base_link".to_string()) {
            return Err("base_link must be the chain root".into());
        }
        let mut link = "base_link";
        let mut visited = HashSet::new();
        let mut pending = Isometry3::identity();
        let mut joints = Vec::with_capacity(7);
        while link != "Link7" {
            let joint = robot
                .joints
                .iter()
                .find(|j| j.parent.link == link)
                .ok_or_else(|| format!("broken URDF chain at {link}"))?;
            if !visited.insert(&joint.name) {
                return Err("cyclic URDF chain".into());
            }
            let p = &joint.origin;
            let xyz = Vector3::new(p.xyz[0], p.xyz[1], p.xyz[2]);
            if !xyz.iter().chain(p.rpy.iter()).all(|v| v.is_finite()) {
                return Err("non-finite URDF origin".into());
            }
            pending *= Isometry3::from_parts(
                Translation3::from(xyz),
                UnitQuaternion::from_euler_angles(p.rpy[0], p.rpy[1], p.rpy[2]),
            );
            match joint.joint_type {
                urdf_rs::JointType::Fixed => {}
                urdf_rs::JointType::Revolute => {
                    let axis =
                        Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]);
                    let norm = axis.norm();
                    let l = &joint.limit;
                    if !norm.is_finite()
                        || norm < 1e-12
                        || ![l.lower, l.upper, l.velocity].iter().all(|v| v.is_finite())
                        || l.lower >= l.upper
                        || l.velocity <= 0.0
                    {
                        return Err(format!("invalid axis or limits for {}", joint.name));
                    }
                    joints.push(Joint {
                        name: joint.name.clone(),
                        origin: pending,
                        axis: Unit::new_normalize(axis),
                        lower: l.lower,
                        upper: l.upper,
                        velocity: l.velocity,
                    });
                    pending = Isometry3::identity();
                }
                _ => return Err("only fixed and revolute URDF joints are supported".into()),
            }
            link = &joint.child.link;
        }
        if visited.len() != robot.joints.len() || joints.len() != 7 {
            return Err("expected exactly seven revolute joints on base_link to Link7".into());
        }
        Ok(Self {
            joints: joints.try_into().map_err(|_| "invalid joint count")?,
            tip: pending,
        })
    }

    pub fn joint_names(&self) -> [String; 7] {
        std::array::from_fn(|i| self.joints[i].name.clone())
    }

    pub fn position_limits(&self) -> ([f64; 7], [f64; 7]) {
        (
            std::array::from_fn(|i| self.joints[i].lower),
            std::array::from_fn(|i| self.joints[i].upper),
        )
    }

    fn validate_q(&self, q: &[f64; 7]) -> Result<(), String> {
        for (value, joint) in q.iter().zip(&self.joints) {
            if !value.is_finite() || *value < joint.lower || *value > joint.upper {
                return Err(format!(
                    "{} position is non-finite or outside URDF limits",
                    joint.name
                ));
            }
        }
        Ok(())
    }

    fn evaluate(&self, q: &[f64; 7], tcp: &TcpTransform) -> Result<(Pose, Jacobian), String> {
        self.validate_q(q)?;
        let tcp = to_isometry(tcp)?;
        let mut transform = Isometry3::identity();
        let mut origins = [Vector3::zeros(); 7];
        let mut axes = [Vector3::zeros(); 7];
        for (i, joint) in self.joints.iter().enumerate() {
            transform *= joint.origin;
            origins[i] = transform.translation.vector;
            axes[i] = transform.rotation * joint.axis.as_ref();
            transform *= Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&joint.axis, q[i]),
            );
        }
        transform = transform * self.tip * tcp;
        let p = transform.translation.vector;
        let mut jacobian = Jacobian::zeros();
        for i in 0..7 {
            let linear = axes[i].cross(&(p - origins[i]));
            for row in 0..3 {
                jacobian[(row, i)] = linear[row];
                jacobian[(row + 3, i)] = axes[i][row];
            }
        }
        let pose = from_isometry(&transform);
        if !jacobian.iter().all(|v| v.is_finite())
            || !pose.translation.iter().all(|v| v.is_finite())
        {
            return Err("non-finite FK or Jacobian".into());
        }
        Ok((pose, jacobian))
    }

    pub fn fk(&self, q: &[f64; 7], tcp: &TcpTransform) -> Result<Pose, String> {
        self.evaluate(q, tcp).map(|(pose, _)| pose)
    }

    #[cfg(test)]
    pub fn jacobian(&self, q: &[f64; 7], tcp: &TcpTransform) -> Result<SMatrix<f64, 6, 7>, String> {
        self.evaluate(q, tcp).map(|(_, jacobian)| jacobian)
    }

    /// A single local weighted DLS step, with common (direction-preserving) scaling.
    /// Invalid dynamics, blocked constraints and unproductive numerical solutions
    /// return errors for the caller to stop/resynchronize. No hidden IK iteration.
    pub fn step(&self, input: StepInput) -> Result<StepCandidate, String> {
        let StepInput {
            reference,
            measured,
            previous_velocity,
            target,
            dt,
            tcp,
            limits,
        } = input;
        if !dt.is_finite() || !(1e-6..=0.1).contains(&dt) {
            return Err("dt must be between 1 microsecond and 100 milliseconds".into());
        }
        self.validate_q(&reference)?;
        self.validate_q(&measured)?;
        let scalars = [
            limits.max_reference_error,
            limits.max_joint_step,
            limits.linear_speed,
            limits.angular_speed,
        ];
        if !scalars
            .iter()
            .chain(limits.joint_velocity.iter())
            .chain(limits.joint_acceleration.iter())
            .all(|v| v.is_finite() && *v >= 1e-9 && *v <= 1e3)
        {
            return Err("motion limits must be finite positive values in [1e-9, 1000]".into());
        }
        let max_step = limits.max_joint_step.min(0.1); // Never leave the local configuration neighborhood.
        let speeds: [f64; 7] =
            std::array::from_fn(|i| limits.joint_velocity[i].min(self.joints[i].velocity));
        for i in 0..7 {
            if !previous_velocity[i].is_finite() || previous_velocity[i].abs() > speeds[i] + 1e-12 {
                return Err("previous velocity exceeds a finite joint velocity limit".into());
            }
            if (reference[i] - measured[i]).abs() > limits.max_reference_error {
                return Err("reference is too far from measured joints".into());
            }
        }
        let (current, jacobian) = self.evaluate(&reference, &tcp)?;
        let current_iso = to_isometry(&current)?;
        let pose_target = match &target {
            LocalTarget::Pose(p) => Some(*p),
            _ => None,
        };
        let desired = match target {
            LocalTarget::Velocity { twist, frame } => {
                if !twist.iter().all(|v| v.is_finite()) {
                    return Err("non-finite target twist".into());
                }
                let mut value = Twist::from_row_slice(&twist);
                if frame == Frame::Tool {
                    let linear = current_iso.rotation * Vector3::new(twist[0], twist[1], twist[2]);
                    let angular = current_iso.rotation * Vector3::new(twist[3], twist[4], twist[5]);
                    value.fixed_rows_mut::<3>(0).copy_from(&linear);
                    value.fixed_rows_mut::<3>(3).copy_from(&angular);
                }
                value
            }
            LocalTarget::Pose(pose) => {
                to_isometry(&pose)?;
                let error = Twist::from_row_slice(&pose_error(&current, &pose));
                if error.fixed_rows::<3>(0).norm() > 0.1 || error.fixed_rows::<3>(3).norm() > 0.35 {
                    return Err("pose target exceeds local range (0.1 m / 0.35 rad)".into());
                }
                error * POSE_GAIN
            }
        };
        if !desired.iter().all(|v| v.is_finite()) || !desired.norm().is_finite() {
            return Err("non-finite target velocity".into());
        }
        // One common task scale preserves the linear/angular ratio.
        let task_scale = (limits.linear_speed / desired.fixed_rows::<3>(0).norm().max(1e-300))
            .min(limits.angular_speed / desired.fixed_rows::<3>(3).norm().max(1e-300))
            .min(1.0);
        let desired_bounded = desired * task_scale;
        let task_weights = Twist::from_row_slice(&[
            1.0 / limits.linear_speed,
            1.0 / limits.linear_speed,
            1.0 / limits.linear_speed,
            1.0 / limits.angular_speed,
            1.0 / limits.angular_speed,
            1.0 / limits.angular_speed,
        ]);
        let mut a = jacobian;
        for row in 0..6 {
            for col in 0..7 {
                a[(row, col)] *= task_weights[row] * speeds[col];
            }
        }
        let b = desired_bounded.component_mul(&task_weights);
        if !a.iter().chain(b.iter()).all(|v| v.is_finite()) {
            return Err("invalid weighted system".into());
        }
        let svd = a
            .try_svd(false, false, 1e-12, 64)
            .ok_or("bounded SVD did not converge")?;
        let sigma = svd.singular_values.min();
        if !sigma.is_finite() {
            return Err("non-finite singular value".into());
        }
        let damping = 0.002 + 0.08 * (1.0 - (sigma / 0.05).min(1.0)).powi(2);
        let system = a * a.transpose() + SMatrix::<f64, 6, 6>::identity() * damping.powi(2);
        let y = system
            .cholesky()
            .ok_or("DLS factorization failed")?
            .solve(&b);
        if !y.iter().all(|v| v.is_finite()) || (system * y - b).norm() > 1e-7 * (1.0 + b.norm()) {
            return Err("DLS solve has an invalid residual".into());
        }
        let normalized = a.transpose() * y;
        let raw: SVector<f64, 7> = SVector::from_fn(|i, _| speeds[i] * normalized[i]);
        if !raw.iter().all(|v| v.is_finite()) {
            return Err("non-finite DLS result".into());
        }
        // alpha is an interval, not just an upper bound: acceleration can impose
        // a lower bound when already moving. Never scale below that bound.
        let mut alpha_min: f64 = 0.0;
        let mut alpha_max: f64 = 1.0;
        let raw_tcp = jacobian * raw;
        for (speed, bound) in [
            (raw_tcp.fixed_rows::<3>(0).norm(), limits.linear_speed),
            (raw_tcp.fixed_rows::<3>(3).norm(), limits.angular_speed),
        ] {
            if !speed.is_finite() {
                return Err("non-finite candidate TCP velocity".into());
            }
            if speed > bound {
                alpha_max = alpha_max.min(0.999 * bound / speed);
            }
        }
        for i in 0..7 {
            let joint = &self.joints[i];
            let acceleration = limits.joint_acceleration[i];
            let lower = joint.lower.max(measured[i] - limits.max_reference_error);
            let upper = joint.upper.min(measured[i] + limits.max_reference_error);
            let mut low = (-speeds[i])
                .max(-max_step / dt)
                .max((lower - reference[i]) / dt)
                .max(previous_velocity[i] - acceleration * dt);
            let mut high = speeds[i]
                .min(max_step / dt)
                .min((upper - reference[i]) / dt)
                .min(previous_velocity[i] + acceleration * dt);
            // Conservative one-step braking envelope: distance after this step
            // must still accommodate v^2/(2a). This is a model constraint only.
            let brake_speed = |distance: f64| {
                let at = acceleration * dt;
                let twice_ad = 2.0 * acceleration * distance.max(0.0);
                twice_ad / ((at * at + twice_ad).sqrt() + at)
            };
            low = low.max(-brake_speed(reference[i] - lower));
            high = high.min(brake_speed(upper - reference[i]));
            if low > high + 1e-12 {
                return Err("joint constraints require stopping/resynchronization".into());
            }
            if raw[i].abs() < 1e-15 {
                if low > 1e-12 || high < -1e-12 {
                    return Err("acceleration cannot follow the local direction".into());
                }
            } else {
                let x = low / raw[i];
                let z = high / raw[i];
                alpha_min = alpha_min.max(x.min(z));
                alpha_max = alpha_max.min(x.max(z));
            }
        }
        if alpha_max < alpha_min || alpha_max < 0.0 {
            return Err("no common scale satisfies joint dynamics".into());
        }
        let alpha = alpha_max.clamp(0.0, 1.0);
        let dq: [f64; 7] = std::array::from_fn(|i| raw[i] * alpha);
        let q: [f64; 7] = std::array::from_fn(|i| reference[i] + dq[i] * dt);
        self.validate_q(&q)?;
        for i in 0..7 {
            if !dq[i].is_finite()
                || dq[i].abs() > speeds[i] + 1e-10
                || (dq[i] - previous_velocity[i]).abs() > limits.joint_acceleration[i] * dt + 1e-10
                || (q[i] - reference[i]).abs() > max_step + 1e-12
                || (q[i] - measured[i]).abs() > limits.max_reference_error + 1e-12
            {
                return Err("final candidate violates joint constraints".into());
            }
        }
        if b.norm() < 1e-12 {
            return Ok(StepCandidate {
                q,
                dq,
                speed_scale: if desired.norm() == 0.0 { 1.0 } else { 0.0 },
            });
        }
        if alpha < 1e-9 || dq.iter().all(|v| v.abs() < 1e-12) {
            return Err("local motion blocked by limits or singularity".into());
        }
        // Inspect the actual FK increment after constraints, not the unconstrained
        // DLS answer. Rejected candidates never leave this stateless module.
        let next = self.fk(&q, &tcp)?;
        let achieved = Twist::from_row_slice(&pose_error(&current, &next)) / dt;
        if achieved.fixed_rows::<3>(0).norm() > limits.linear_speed + 1e-10
            || achieved.fixed_rows::<3>(3).norm() > limits.angular_speed + 1e-10
        {
            return Err("final FK increment exceeds TCP speed limits".into());
        }
        let achieved_weighted = achieved.component_mul(&task_weights);
        let requested_scaled = b * alpha;
        if (achieved_weighted - requested_scaled).norm() > 0.5 * requested_scaled.norm()
            || achieved_weighted.dot(&b) <= 0.0
        {
            return Err(
                "local direction cannot make useful progress near this configuration".into(),
            );
        }
        if let Some(goal) = pose_target {
            let before = Twist::from_row_slice(&pose_error(&current, &goal))
                .component_mul(&task_weights)
                .norm();
            let after = Twist::from_row_slice(&pose_error(&next, &goal))
                .component_mul(&task_weights)
                .norm();
            if !after.is_finite() || after >= before {
                return Err("local pose error does not decrease".into());
            }
        }
        let projection = achieved_weighted.dot(&b) / b.norm_squared();
        let speed_scale = (projection * task_scale).clamp(0.0, 1.0);
        if !speed_scale.is_finite() {
            return Err("invalid achieved speed scale".into());
        }
        Ok(StepCandidate { q, dq, speed_scale })
    }
}

fn to_isometry(pose: &Pose) -> Result<Isometry3<f64>, String> {
    if !pose
        .translation
        .iter()
        .chain(pose.rotation.iter())
        .all(|v| v.is_finite())
    {
        return Err("pose contains non-finite values".into());
    }
    let [x, y, z, w] = pose.rotation;
    let quaternion = Quaternion::new(w, x, y, z);
    let norm = quaternion.norm();
    if !norm.is_finite() || (norm - 1.0).abs() > 1e-6 {
        return Err("pose rotation must be a unit xyzw quaternion".into());
    }
    Ok(Isometry3::from_parts(
        Translation3::from(Vector3::from_row_slice(&pose.translation)),
        UnitQuaternion::new_normalize(quaternion),
    ))
}

fn from_isometry(transform: &Isometry3<f64>) -> Pose {
    let q = transform.rotation.quaternion();
    Pose {
        translation: transform.translation.vector.into(),
        rotation: [q.i, q.j, q.k, q.w],
    }
}

/// Translation is at the TCP; rotation keeps TCP position fixed. Base rotation
/// pre-multiplies R, tool rotation post-multiplies R. Translation uses the old R.
pub fn offset_pose(pose: &Pose, delta: [f64; 6], frame: Frame) -> Result<Pose, String> {
    let transform = to_isometry(pose)?;
    if !delta.iter().all(|v| v.is_finite()) {
        return Err("non-finite pose offset".into());
    }
    let translation = Vector3::new(delta[0], delta[1], delta[2]);
    let angular = Vector3::new(delta[3], delta[4], delta[5]);
    if !translation.norm().is_finite() || !angular.norm().is_finite() {
        return Err("pose offset overflow".into());
    }
    let rotation = UnitQuaternion::from_scaled_axis(angular);
    let (p, r) = match frame {
        Frame::Base => (
            transform.translation.vector + translation,
            rotation * transform.rotation,
        ),
        Frame::Tool => (
            transform.translation.vector + transform.rotation * translation,
            transform.rotation * rotation,
        ),
    };
    let result = from_isometry(&Isometry3::from_parts(Translation3::from(p), r));
    to_isometry(&result)?;
    Ok(result)
}

/// Base-expressed [position error, shortest rotation vector]. Invalid inputs
/// yield +infinity components (the fixed public signature has no Result).
/// Callers accepting external poses must validate them before motion planning.
pub fn pose_error(current: &Pose, target: &Pose) -> [f64; 6] {
    let (Ok(current), Ok(target)) = (to_isometry(current), to_isometry(target)) else {
        return [f64::INFINITY; 6];
    };
    let p = target.translation.vector - current.translation.vector;
    let r = (target.rotation * current.rotation.inverse()).scaled_axis();
    [p.x, p.y, p.z, r.x, r.y, r.z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn limits() -> MotionLimits {
        MotionLimits {
            joint_velocity: [0.4; 7],
            joint_acceleration: [1.5; 7],
            max_reference_error: 0.08,
            max_joint_step: 0.01,
            linear_speed: 0.03,
            angular_speed: 0.2,
        }
    }

    fn input(q: [f64; 7], target: LocalTarget) -> StepInput {
        StepInput {
            reference: q,
            measured: q,
            previous_velocity: [0.0; 7],
            target,
            dt: 0.02,
            tcp: Pose::identity(),
            limits: limits(),
        }
    }

    fn velocity(twist: [f64; 6], frame: Frame) -> LocalTarget {
        LocalTarget::Velocity { twist, frame }
    }

    fn interior() -> [f64; 7] {
        [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3]
    }

    fn check_bounds(k: &Kinematics, i: &StepInput, c: &StepCandidate) {
        let (lower, upper) = k.position_limits();
        assert!(c.speed_scale.is_finite() && (0.0..=1.0).contains(&c.speed_scale));
        let old = k.fk(&i.reference, &i.tcp).unwrap();
        let new = k.fk(&c.q, &i.tcp).unwrap();
        let effective = Twist::from(pose_error(&old, &new)) / i.dt;
        assert!(effective.fixed_rows::<3>(0).norm() <= i.limits.linear_speed + 1e-10);
        assert!(effective.fixed_rows::<3>(3).norm() <= i.limits.angular_speed + 1e-10);
        for j in 0..7 {
            assert!(c.q[j].is_finite() && c.dq[j].is_finite());
            assert!(c.q[j] >= lower[j] && c.q[j] <= upper[j]);
            assert!((c.q[j] - i.reference[j] - c.dq[j] * i.dt).abs() < 1e-14);
            assert!((c.q[j] - i.reference[j]).abs() <= i.limits.max_joint_step + 1e-12);
            assert!(c.dq[j].abs() <= i.limits.joint_velocity[j] + 1e-12);
            assert!(
                (c.dq[j] - i.previous_velocity[j]).abs()
                    <= i.limits.joint_acceleration[j] * i.dt + 1e-10
            );
            assert!((c.q[j] - i.measured[j]).abs() <= i.limits.max_reference_error + 1e-12);
        }
    }

    #[test]
    fn independent_homogeneous_matrix_benchmarks() {
        #[derive(Deserialize)]
        struct Sample {
            side: String,
            q: [f64; 7],
            tcp: Pose,
            matrix: [[f64; 4]; 4],
        }
        #[derive(Deserialize)]
        struct Reference {
            samples: Vec<Sample>,
        }
        let fixtures: Reference =
            serde_json::from_str(include_str!("../../resources/models/fk-reference.json")).unwrap();
        assert_eq!(fixtures.samples.len(), 6);
        for sample in fixtures.samples {
            let k = Kinematics::load(&sample.side).unwrap();
            let actual = to_isometry(&k.fk(&sample.q, &sample.tcp).unwrap())
                .unwrap()
                .to_homogeneous();
            for row in 0..4 {
                for col in 0..4 {
                    assert!(
                        (actual[(row, col)] - sample.matrix[row][col]).abs() < 2e-12,
                        "{} q={:?}, matrix ({row}, {col})",
                        sample.side,
                        sample.q
                    );
                }
            }
        }
    }

    #[test]
    fn tcp_jacobian_matches_central_differences_both_sides_and_step_sizes() {
        let tcp = offset_pose(
            &Pose::identity(),
            [0.03, -0.02, 0.08, 0.2, -0.1, 0.3],
            Frame::Base,
        )
        .unwrap();
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            for q in [interior(), [-0.5, 1.5, 0.2, 2.1, 0.7, -0.3, 0.8]] {
                let jacobian = k.jacobian(&q, &tcp).unwrap();
                for h in [1e-4, 1e-6] {
                    for col in 0..7 {
                        let mut plus = q;
                        plus[col] += h;
                        let mut minus = q;
                        minus[col] -= h;
                        let plus = k.fk(&plus, &tcp).unwrap();
                        let minus = k.fk(&minus, &tcp).unwrap();
                        let difference = pose_error(&minus, &plus);
                        for row in 0..6 {
                            assert!(
                                (jacobian[(row, col)] - difference[row] / (2.0 * h)).abs() < 2e-8,
                                "{side} ({row}, {col}), h={h}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn base_and_tool_offsets_use_tcp_axes_and_keep_rotation_center() {
        let pose = Pose {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, (FRAC_PI_2 / 2.0).sin(), (FRAC_PI_2 / 2.0).cos()],
        };
        let base = offset_pose(&pose, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base).unwrap();
        let tool = offset_pose(&pose, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Tool).unwrap();
        for i in 0..3 {
            assert!((base.translation[i] - [1.1, 2.0, 3.0][i]).abs() < 1e-12);
            assert!((tool.translation[i] - [1.0, 2.1, 3.0][i]).abs() < 1e-12);
        }
        let base = offset_pose(&pose, [0.0, 0.0, 0.0, FRAC_PI_2, 0.0, 0.0], Frame::Base).unwrap();
        let tool = offset_pose(&pose, [0.0, 0.0, 0.0, FRAC_PI_2, 0.0, 0.0], Frame::Tool).unwrap();
        assert_eq!(base.translation, pose.translation);
        assert_eq!(tool.translation, pose.translation);
        let base_r = to_isometry(&base).unwrap().rotation;
        let tool_r = to_isometry(&tool).unwrap().rotation;
        assert!((base_r * Vector3::x() - Vector3::z()).norm() < 1e-12);
        assert!((tool_r * Vector3::x() - Vector3::y()).norm() < 1e-12);
        assert!((base_r * Vector3::z() + Vector3::y()).norm() < 1e-12);
        assert!((tool_r * Vector3::z() - Vector3::x()).norm() < 1e-12);
    }

    #[test]
    fn serialization_and_shortest_rotation_error_are_xyzw() {
        assert_eq!(serde_json::to_string(&Frame::Tool).unwrap(), "\"tool\"");
        assert_eq!(serde_json::to_string(&Frame::Base).unwrap(), "\"base\"");
        assert_eq!(
            serde_json::to_value(Pose::identity()).unwrap()["rotation"],
            serde_json::json!([0.0, 0.0, 0.0, 1.0])
        );
        let pose = offset_pose(
            &Pose::identity(),
            [0.0, 0.0, 0.0, 0.2, -0.3, 0.4],
            Frame::Base,
        )
        .unwrap();
        let negated = Pose {
            rotation: pose.rotation.map(|v| -v),
            ..pose
        };
        assert!(pose_error(&pose, &negated).iter().all(|v| v.abs() < 1e-12));
        let actual = pose_error(&Pose::identity(), &pose);
        for i in 0..6 {
            assert!((actual[i] - [0.0, 0.0, 0.0, 0.2, -0.3, 0.4][i]).abs() < 1e-12);
        }
        let mut invalid = pose;
        invalid.rotation = [0.0; 4];
        assert!(offset_pose(&invalid, [0.0; 6], Frame::Base).is_err());
        assert!(pose_error(&invalid, &pose).iter().all(|v| v.is_infinite()));
        let tiny = offset_pose(&pose, [0.0, 0.0, 0.0, 1e-10, 0.0, 0.0], Frame::Base).unwrap();
        assert!((pose_error(&pose, &tiny)[3] - 1e-10).abs() < 1e-14);
    }

    fn planar_urdf() -> String {
        let mut xml = String::from("<robot name=\"planar\"><link name=\"base_link\"/>");
        for n in 1..=7 {
            xml += &format!("<link name=\"Link{n}\"/>");
        }
        // Deliberately reversed XML joint order; all seven axes still rotate z.
        for n in (1..=7).rev() {
            let parent = if n == 1 {
                "base_link".into()
            } else {
                format!("Link{}", n - 1)
            };
            xml += &format!(
                "<joint name=\"J{n}\" type=\"revolute\"><parent link=\"{parent}\"/><child link=\"Link{n}\"/><origin xyz=\"1 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/><limit lower=\"-4\" upper=\"4\" velocity=\"1\" effort=\"1\"/></joint>"
            );
        }
        xml + "</robot>"
    }

    #[test]
    fn analytic_planar_chain_preserves_fixed_origin_and_graph_order() {
        let xml = planar_urdf().replace("<parent link=\"base_link\"/>", "<parent link=\"mount\"/>")
            .replace("</robot>", &format!("<link name=\"mount\"/><joint name=\"mount_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"mount\"/><origin xyz=\"0.2 0 0\" rpy=\"0 0 {FRAC_PI_2}\"/></joint></robot>"));
        let k = Kinematics::from_urdf(&xml).unwrap();
        assert_eq!(
            k.joint_names(),
            std::array::from_fn(|i| format!("J{}", i + 1))
        );
        let pose = k
            .fk(
                &[FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                &Pose::identity(),
            )
            .unwrap();
        assert!((pose.translation[0] + 5.8).abs() < 1e-12);
        assert!((pose.translation[1] - 1.0).abs() < 1e-12);
        assert!(pose.translation[2].abs() < 1e-12);
        assert!(
            (to_isometry(&pose).unwrap().rotation * Vector3::x() + Vector3::x()).norm() < 1e-12
        );
    }

    #[test]
    fn malformed_or_non_serial_urdf_is_rejected() {
        let xml = planar_urdf();
        for bad in [
            xml.replace("xyz=\"0 0 1\"", "xyz=\"0 0 0\""),
            xml.replace("lower=\"-4\"", "lower=\"5\""),
            xml.replace("velocity=\"1\"", "velocity=\"NaN\""),
            xml.replace("xyz=\"1 0 0\"", "xyz=\"NaN 0 0\""),
            xml.replace("type=\"revolute\"", "type=\"prismatic\""),
            xml.replace("<axis", "<mimic joint=\"J1\"/><axis"),
            xml.replace("<parent link=\"Link1\"/>", "<parent link=\"base_link\"/>"),
            xml.replace("<child link=\"Link7\"/>", "<child link=\"base_link\"/>"),
            format!("<!DOCTYPE robot [<!ENTITY x SYSTEM 'file:///etc/passwd'>]>{xml}"),
        ] {
            assert!(Kinematics::from_urdf(&bad).is_err());
        }
        assert!(Kinematics::load("other").is_err());
        let right = Kinematics::load("right").unwrap();
        let left = Kinematics::load("left").unwrap();
        assert_eq!(right.position_limits().0[5], -0.785);
        assert_eq!(left.position_limits().0[5], -0.524);
        assert_ne!(
            right.fk(&interior(), &Pose::identity()).unwrap(),
            left.fk(&interior(), &Pose::identity()).unwrap()
        );
    }

    #[test]
    fn continuous_step_and_pose_step_share_direction_and_enforce_all_bounds() {
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let q = interior();
            let current = k.fk(&q, &Pose::identity()).unwrap();
            let v = [0.006, -0.003, 0.002, 0.01, -0.005, 0.004];
            let i = input(q, velocity(v, Frame::Base));
            let continuous = k.step(i.clone()).unwrap();
            check_bounds(&k, &i, &continuous);
            assert!(continuous.speed_scale > 0.0);
            let target = offset_pose(&current, v.map(|x| x / POSE_GAIN), Frame::Base).unwrap();
            let p = StepInput {
                target: LocalTarget::Pose(target),
                ..i.clone()
            };
            let pose = k.step(p.clone()).unwrap();
            check_bounds(&k, &p, &pose);
            for j in 0..7 {
                assert!((continuous.q[j] - pose.q[j]).abs() < 1e-12);
            }
            let next = k.fk(&pose.q, &Pose::identity()).unwrap();
            assert!(
                Twist::from(pose_error(&next, &target)).norm()
                    < Twist::from(pose_error(&current, &target)).norm()
            );
        }
    }

    #[test]
    fn nearby_pose_sequence_converges_without_a_configuration_jump() {
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let q = interior();
            let start = k.fk(&q, &Pose::identity()).unwrap();
            let goal = offset_pose(
                &start,
                [0.005, -0.003, 0.002, 0.03, -0.02, 0.01],
                Frame::Base,
            )
            .unwrap();
            let mut i = input(q, LocalTarget::Pose(goal));
            let mut reached = false;
            for tick in 0..500 {
                let pose = k.fk(&i.reference, &i.tcp).unwrap();
                let error = Twist::from(pose_error(&pose, &goal));
                if error.fixed_rows::<3>(0).norm() < 1e-6 && error.fixed_rows::<3>(3).norm() < 1e-5
                {
                    reached = true;
                    break;
                }
                let c = k
                    .step(i.clone())
                    .unwrap_or_else(|e| panic!("{side} tick {tick}: {e}"));
                check_bounds(&k, &i, &c);
                i.reference = c.q;
                i.measured = c.q;
                i.previous_velocity = c.dq;
            }
            assert!(reached, "{side} did not reach local target");
        }
    }

    #[test]
    fn tool_velocity_is_rotated_at_actual_tcp_without_point_shift() {
        let k = Kinematics::load("right").unwrap();
        let tcp = offset_pose(
            &Pose::identity(),
            [0.04, 0.01, 0.07, 0.3, -0.2, 0.1],
            Frame::Tool,
        )
        .unwrap();
        let q = interior();
        let rotation = to_isometry(&k.fk(&q, &tcp).unwrap()).unwrap().rotation;
        let local = [0.005, -0.003, 0.002, 0.01, 0.02, -0.01];
        let linear = rotation * Vector3::new(local[0], local[1], local[2]);
        let angular = rotation * Vector3::new(local[3], local[4], local[5]);
        let base = [
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
        ];
        let tool_input = StepInput {
            tcp,
            ..input(q, velocity(local, Frame::Tool))
        };
        let base_input = StepInput {
            tcp,
            ..input(q, velocity(base, Frame::Base))
        };
        let a = k.step(tool_input).unwrap();
        let b = k.step(base_input).unwrap();
        for j in 0..7 {
            assert!((a.q[j] - b.q[j]).abs() < 1e-12);
        }
    }

    #[test]
    fn finite_dynamics_across_start_reverse_and_stop() {
        let k = Kinematics::load("right").unwrap();
        let mut i = input(
            interior(),
            velocity([0.01, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base),
        );
        let mut accepted = 0;
        for sign in [1.0, -1.0] {
            i.target = velocity([sign * 0.01, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base);
            for _ in 0..12 {
                match k.step(i.clone()) {
                    Ok(c) => {
                        check_bounds(&k, &i, &c);
                        i.reference = c.q;
                        i.measured = c.q;
                        i.previous_velocity = c.dq;
                        accepted += 1;
                    }
                    // Common scaling may reject a reversal that cannot satisfy
                    // acceleration; control layer must then stop/resynchronize.
                    Err(_) => break,
                }
            }
        }
        assert!(accepted > 0);
        let stop = StepInput {
            target: velocity([0.0; 6], Frame::Base),
            ..i.clone()
        };
        if let Ok(c) = k.step(stop.clone()) {
            check_bounds(&k, &stop, &c);
        }
    }

    #[test]
    fn joint_position_velocity_acceleration_reference_and_increment_constraints() {
        let coaxial = planar_urdf().replace("xyz=\"1 0 0\"", "xyz=\"0 0 0\"");
        let k = Kinematics::from_urdf(&coaxial).unwrap();
        let mut i = input(
            [0.0; 7],
            velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.1], Frame::Base),
        );
        i.limits.max_joint_step = 1e-5;
        i.limits.max_reference_error = 2e-5;
        i.limits.joint_velocity = [0.01; 7];
        i.limits.joint_acceleration = [0.001; 7];
        let c = k.step(i.clone()).unwrap();
        check_bounds(&k, &i, &c);
        assert!(c.speed_scale < 1.0);
        let mut at_limit = input(
            [4.0; 7],
            velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.1], Frame::Base),
        );
        at_limit.limits.joint_acceleration = [100.0; 7];
        assert!(k.step(at_limit).is_err());
        // Every official lower/upper joint boundary must reject or stay within
        // all constraints; no failed branch may return an invalid candidate.
        for side in ["right", "left"] {
            let official = Kinematics::load(side).unwrap();
            let (lower, upper) = official.position_limits();
            for joint in 0..7 {
                for bound in [lower[joint], upper[joint]] {
                    let mut q = interior();
                    q[joint] = bound;
                    for sign in [-1.0, 1.0] {
                        let i = input(
                            q,
                            velocity([sign * 0.01, 0.0, 0.0, 0.0, sign * 0.02, 0.0], Frame::Base),
                        );
                        if let Ok(c) = official.step(i.clone()) {
                            check_bounds(&official, &i, &c);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn individual_dynamic_constraints_bind_the_final_transmitted_velocity() {
        let coaxial = planar_urdf().replace("xyz=\"1 0 0\"", "xyz=\"0 0 0\"");
        let k = Kinematics::from_urdf(&coaxial).unwrap();
        let base = input(
            [0.0; 7],
            velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.1], Frame::Base),
        );
        let mut increment = base.clone();
        increment.limits.max_joint_step = 1e-5;
        let c = k.step(increment.clone()).unwrap();
        check_bounds(&k, &increment, &c);
        assert!(c.q.iter().any(|q| (*q - 1e-5).abs() < 1e-12));
        let mut acceleration = base.clone();
        acceleration.limits.joint_acceleration = [0.001; 7];
        let c = k.step(acceleration.clone()).unwrap();
        check_bounds(&k, &acceleration, &c);
        assert!(c.dq.iter().any(|v| (*v - 2e-5).abs() < 1e-12));
        let mut speed = base.clone();
        speed.limits.joint_velocity = [0.01; 7];
        let c = k.step(speed.clone()).unwrap();
        check_bounds(&k, &speed, &c);
        assert!(c.dq.iter().any(|v| (*v - 0.01).abs() < 1e-12));
        let mut urdf_speed = base.clone();
        urdf_speed.target = velocity([0.0, 0.0, 0.0, 0.0, 0.0, 100.0], Frame::Base);
        urdf_speed.limits.joint_velocity = [100.0; 7];
        urdf_speed.limits.joint_acceleration = [100.0; 7];
        urdf_speed.limits.angular_speed = 10.0;
        urdf_speed.limits.max_joint_step = 0.1;
        let c = k.step(urdf_speed.clone()).unwrap();
        check_bounds(&k, &urdf_speed, &c);
        assert!(c.dq.iter().all(|v| v.abs() <= 1.0 + 1e-12));
        assert!(c.dq.iter().any(|v| (*v - 1.0).abs() < 1e-12));
        let mut reference = base.clone();
        reference.measured = [-reference.limits.max_reference_error; 7];
        assert!(k.step(reference).is_err());
        let mut inward = base.clone();
        inward.reference = [4.0; 7];
        inward.measured = [4.0; 7];
        inward.target = velocity([0.0, 0.0, 0.0, 0.0, 0.0, -0.1], Frame::Base);
        let c = k.step(inward.clone()).unwrap();
        check_bounds(&k, &inward, &c);
        assert!(c.q.iter().all(|q| *q < 4.0));
        let impossible_reversal = StepInput {
            previous_velocity: [0.4; 7],
            target: velocity([0.0, 0.0, 0.0, 0.0, 0.0, -0.1], Frame::Base),
            ..base
        };
        assert!(k.step(impossible_reversal).is_err());
    }

    #[test]
    fn singular_unproductive_direction_errors_and_feasible_direction_scales() {
        let k = Kinematics::from_urdf(&planar_urdf()).unwrap();
        // Straight planar chain cannot translate along z at any joint speed.
        assert!(
            k.step(input(
                [0.0; 7],
                velocity([0.0, 0.0, 0.01, 0.0, 0.0, 0.0], Frame::Base)
            ))
            .is_err()
        );
        // A rank-deficient chain can still perform a useful in-plane local turn.
        let i = input(
            [0.0; 7],
            velocity([0.0, 0.005, 0.0, 0.0, 0.0, 0.0], Frame::Base),
        );
        let c = k.step(i.clone()).unwrap();
        check_bounds(&k, &i, &c);
        assert!(c.speed_scale > 0.0 && c.speed_scale <= 1.0);
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let i = input(
                [0.0; 7],
                velocity([0.0, 0.005, 0.0, 0.0, 0.0, 0.0], Frame::Base),
            );
            if let Ok(c) = k.step(i.clone()) {
                check_bounds(&k, &i, &c);
            }
        }
    }

    #[test]
    fn invalid_inputs_fail_without_nan_or_unbounded_iteration() {
        let k = Kinematics::load("right").unwrap();
        let i = input(
            interior(),
            velocity([0.001, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base),
        );
        for dt in [0.0, -0.01, 0.100001, f64::INFINITY, f64::NAN] {
            assert!(k.step(StepInput { dt, ..i.clone() }).is_err());
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
            let mut q = interior();
            q[2] = bad;
            assert!(k.fk(&q, &Pose::identity()).is_err());
            assert!(
                k.step(StepInput {
                    reference: q,
                    ..i.clone()
                })
                .is_err()
            );
            assert!(
                k.step(StepInput {
                    measured: q,
                    ..i.clone()
                })
                .is_err()
            );
            assert!(
                k.step(StepInput {
                    previous_velocity: [bad; 7],
                    ..i.clone()
                })
                .is_err()
            );
            assert!(
                k.step(StepInput {
                    target: velocity([bad; 6], Frame::Tool),
                    ..i.clone()
                })
                .is_err()
            );
            let invalid = Pose {
                translation: [bad; 3],
                ..Pose::identity()
            };
            assert!(
                k.step(StepInput {
                    tcp: invalid,
                    ..i.clone()
                })
                .is_err()
            );
            assert!(
                k.step(StepInput {
                    target: LocalTarget::Pose(invalid),
                    ..i.clone()
                })
                .is_err()
            );
        }
        let mut stale = i.clone();
        stale.measured[0] += 0.081;
        assert!(k.step(stale).is_err());
        let mut invalid = i.clone();
        invalid.limits.joint_velocity[0] = 0.0;
        assert!(k.step(invalid).is_err());
        let mut invalid = i.clone();
        invalid.limits.joint_acceleration[2] = f64::NAN;
        assert!(k.step(invalid).is_err());
        let goal = offset_pose(
            &k.fk(&interior(), &Pose::identity()).unwrap(),
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            Frame::Base,
        )
        .unwrap();
        assert!(
            k.step(StepInput {
                target: LocalTarget::Pose(goal),
                ..i.clone()
            })
            .is_err()
        );
        assert!(
            k.step(StepInput {
                tcp: Pose {
                    rotation: [0.0; 4],
                    ..Pose::identity()
                },
                ..i.clone()
            })
            .is_err()
        );
        assert!(
            k.step(StepInput {
                previous_velocity: [0.5; 7],
                ..i
            })
            .is_err()
        );
    }
}
