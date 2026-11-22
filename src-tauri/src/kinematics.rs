//! Fixed R1-7A local kinematics. SI units; geometric twists are [linear, angular]
//! at the TCP, expressed in base axes. This module has no control-loop state.
use crate::trajectory;
use nalgebra::{
    Isometry3, Matrix3, Quaternion, SMatrix, SVector, Translation3, Unit, UnitQuaternion, Vector3,
};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

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
    pub smoothing: Option<SmoothingState>,
    pub planning: Option<CartesianPlanning>,
}

#[derive(Clone, Copy, Debug)]
pub struct CartesianPlanning {
    /// Actual accepted start for joint-position bounds, including outside axes.
    pub origin: [f64; 7],
    /// Optional posture on the previewed IK branch at the current TCP progress.
    pub guide: Option<JointGuide>,
}

#[derive(Clone, Copy, Debug)]
pub struct JointGuide {
    pub axis: usize,
    pub position: [f64; 7],
    pub tangent: [f64; 7],
}

#[derive(Clone, Copy, Debug)]
pub struct CartesianPath {
    pub origin: Pose,
    pub axis: usize,
    pub frame: Frame,
}

#[derive(Clone, Copy, Debug)]
pub struct HoldPose {
    pub joints: [f64; 7],
    pub bandwidth: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SmoothingState {
    pub hold: Option<HoldPose>,
    pub acceleration: [f64; 7],
    pub jerk: [f64; 7],
    pub path: Option<CartesianPath>,
    pub stopping: bool,
}

impl CartesianPath {
    pub(crate) fn axis_in_base(self) -> Result<Vector3<f64>, String> {
        if self.axis >= 6 {
            return Err("invalid Cartesian path axis".into());
        }
        let axis = Vector3::ith(self.axis % 3, 1.0);
        Ok(if self.frame == Frame::Tool {
            to_isometry(&self.origin)?.rotation * axis
        } else {
            axis
        })
    }
    fn correct(
        self,
        current: &Pose,
        desired: Twist,
    ) -> Result<(Twist, SMatrix<f64, 6, 6>), String> {
        let axis = self.axis_in_base()?;
        let perpendicular = Matrix3::identity() - axis * axis.transpose();
        let error = Twist::from_row_slice(&pose_error(current, &self.origin));
        let mut result = desired;
        let mut priority = SMatrix::<f64, 6, 6>::identity();
        for block in 0..2 {
            let commanded = (block == 0) == (self.axis < 3);
            let project = if commanded {
                perpendicular
            } else {
                Matrix3::identity()
            };
            let offset = block * 3;
            let requested = desired.fixed_rows::<3>(offset).into_owned();
            let correction = error.fixed_rows::<3>(offset).into_owned();
            result.fixed_rows_mut::<3>(offset).copy_from(
                &((Matrix3::identity() - project) * requested + project * correction * 20.0),
            );
            priority
                .fixed_view_mut::<3, 3>(offset, offset)
                .copy_from(&(Matrix3::identity() + 100.0 * project));
        }
        Ok((result, priority))
    }
    #[cfg(test)]
    pub(crate) fn check(self, candidate: &Pose) -> Result<(), String> {
        let error = Twist::from_row_slice(&pose_error(&self.origin, candidate));
        let axis = self.axis_in_base()?;
        let orthogonal = Matrix3::identity() - axis * axis.transpose();
        let linear = error.fixed_rows::<3>(0).into_owned();
        let angular = error.fixed_rows::<3>(3).into_owned();
        let linear_error = if self.axis < 3 {
            (orthogonal * linear).norm()
        } else {
            linear.norm()
        };
        let angular_error = if self.axis >= 3 {
            (orthogonal * angular).norm()
        } else {
            angular.norm()
        };
        if linear_error > 0.001 || angular_error > std::f64::consts::PI / 180.0 {
            return Err(format!(
                "Cartesian path deviation: position={linear_error}m orientation={angular_error}rad"
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct StepCandidate {
    pub q: [f64; 7],
    pub dq: [f64; 7],
    pub ddq: [f64; 7],
    /// Achieved projection along the requested twist, including all scaling.
    pub speed_scale: f64,
}

#[derive(Clone, Debug)]
struct Joint {
    name: String,
    origin: Isometry3<f64>,
    axis: Unit<Vector3<f64>>,
    link_mass: f64,
    link_com: Vector3<f64>,
    #[cfg_attr(not(test), allow(dead_code))]
    lower: f64,
    #[cfg_attr(not(test), allow(dead_code))]
    upper: f64,
    velocity: f64,
}

#[derive(Clone, Debug)]
pub struct Kinematics {
    joints: [Joint; 7],
    tip: Isometry3<f64>,
    chain_length: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct ContinuationBounds {
    pub lower: [f64; 7],
    pub upper: [f64; 7],
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
        let link_bodies: HashMap<_, _> = robot
            .links
            .iter()
            .map(|link| (link.name.as_str(), link))
            .collect();
        let mut link = "base_link";
        let mut visited = HashSet::new();
        let mut pending = Isometry3::identity();
        let mut joints = Vec::with_capacity(7);
        let mut chain_length = 0.0;
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
            chain_length += xyz.norm();
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
                    let body = link_bodies
                        .get(joint.child.link.as_str())
                        .ok_or_else(|| format!("missing child link for {}", joint.name))?;
                    let mass = body.inertial.mass.value;
                    let com = body.inertial.origin.xyz;
                    if !mass.is_finite() || mass < 0.0 || !com.iter().all(|v| v.is_finite()) {
                        return Err(format!("invalid inertial for {}", joint.name));
                    }
                    joints.push(Joint {
                        name: joint.name.clone(),
                        origin: pending,
                        axis: Unit::new_normalize(axis),
                        link_mass: mass,
                        link_com: Vector3::new(com[0], com[1], com[2]),
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
        if visited.len() != robot.joints.len() || joints.len() != 7 || !chain_length.is_finite() {
            return Err("expected exactly seven revolute joints on base_link to Link7".into());
        }
        Ok(Self {
            joints: joints.try_into().map_err(|_| "invalid joint count")?,
            tip: pending,
            chain_length,
        })
    }

    pub fn joint_names(&self) -> [String; 7] {
        std::array::from_fn(|i| self.joints[i].name.clone())
    }

    /// URDF joint velocity ratings, independent of editable motion preferences.
    pub fn speed_limits(&self) -> [f64; 7] {
        std::array::from_fn(|i| self.joints[i].velocity)
    }

    /// Conservative base-centred geometric bound including fixed links and TCP.
    /// Local IK, rather than this sphere, determines available directional travel.
    pub fn reach_radius(&self, tcp: &TcpTransform) -> f64 {
        self.chain_length + Vector3::from(tcp.translation).norm()
    }

    /// Planning bounds retain the starting configuration when nominal URDF
    /// zeros differ, while permitting no additional travel outside the model.
    pub(crate) fn continuation_bounds(
        &self,
        origin: &[f64; 7],
    ) -> Result<ContinuationBounds, String> {
        self.validate_q(origin)?;
        Ok(ContinuationBounds {
            lower: std::array::from_fn(|i| self.joints[i].lower.min(origin[i])),
            upper: std::array::from_fn(|i| self.joints[i].upper.max(origin[i])),
        })
    }

    /// Exact instantaneous linear/angular norm bounds over the joint-speed box.
    /// These describe the Jacobian at this pose, not independent Cartesian goals.
    pub fn cartesian_speed_limits(
        &self,
        q: &[f64; 7],
        tcp: &TcpTransform,
        joint_limits: [f64; 7],
    ) -> Result<(f64, f64), String> {
        if joint_limits.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err("joint speed limits must be finite positive values".into());
        }
        let (_, jacobian) = self.evaluate(q, tcp)?;
        let speeds: [f64; 7] =
            std::array::from_fn(|i| joint_limits[i].min(self.joints[i].velocity));
        let mut linear: f64 = 0.0;
        let mut angular: f64 = 0.0;
        // A convex norm on a box attains its maximum at a vertex.
        for signs in 0..128 {
            let velocity = SVector::<f64, 7>::from_fn(|i, _| {
                if signs & (1 << i) == 0 {
                    -speeds[i]
                } else {
                    speeds[i]
                }
            });
            let twist = jacobian * velocity;
            linear = linear.max(twist.fixed_rows::<3>(0).norm());
            angular = angular.max(twist.fixed_rows::<3>(3).norm());
        }
        if !linear.is_finite() || !angular.is_finite() {
            return Err("non-finite Cartesian speed bounds".into());
        }
        Ok((linear, angular))
    }

    /// Geometry-only continuation near `seed`, with no operating speed or servo
    /// derivatives. The trust region prevents IK branch jumps. Nominal URDF
    /// position zeros are not installed-arm hardware travel gates.
    pub(crate) fn continue_pose(
        &self,
        seed: &[f64; 7],
        tcp: &TcpTransform,
        target: &Pose,
        max_joint_step: f64,
        bounds: ContinuationBounds,
    ) -> Result<[f64; 7], String> {
        to_isometry(target)?;
        if !max_joint_step.is_finite() || max_joint_step <= 0.0 {
            return Err("invalid IK continuation trust region".into());
        }
        let length = self.reach_radius(tcp).max(f64::MIN_POSITIVE);
        if !length.is_finite() {
            return Err("non-finite model reach".into());
        }
        let initial = self.fk(seed, tcp)?;
        let distance = Vector3::from(target.translation) - Vector3::from(initial.translation);
        let tolerance = (distance.norm() / length * 1e-6).clamp(64.0 * f64::EPSILON, 1e-10);
        let mut q = *seed;
        for _ in 0..48 {
            let (pose, mut jacobian) = self.evaluate(&q, tcp)?;
            let mut error = Twist::from(pose_error(&pose, target));
            for row in 0..3 {
                error[row] /= length;
                jacobian.row_mut(row).scale_mut(1.0 / length);
            }
            if error.norm() <= tolerance {
                return Ok(q);
            }
            let mut improved = None;
            for damping in [1e-6_f64, 1e-4, 1e-2] {
                let h = jacobian.transpose() * jacobian
                    + SMatrix::<f64, 7, 7>::identity() * damping.powi(2);
                let gradient = -jacobian.transpose() * error;
                let window = trajectory::VelocityWindow {
                    low: std::array::from_fn(|i| {
                        (seed[i] - max_joint_step).max(bounds.lower[i]) - q[i]
                    }),
                    high: std::array::from_fn(|i| {
                        (seed[i] + max_joint_step).min(bounds.upper[i]) - q[i]
                    }),
                };
                let Ok(step) = trajectory::bounded_quadratic(h, gradient, window, [0.0; 7]) else {
                    continue;
                };
                let mut amount: f64 = 1.0;
                for _ in 0..12 {
                    let next = std::array::from_fn(|i| {
                        (q[i] + step[i] * amount).clamp(bounds.lower[i], bounds.upper[i])
                    });
                    let next_pose = self.fk(&next, tcp)?;
                    let mut next_error = Twist::from(pose_error(&next_pose, target));
                    next_error.fixed_rows_mut::<3>(0).scale_mut(1.0 / length);
                    if next_error.norm() < error.norm() * (1.0 - 1e-8) {
                        improved = Some(next);
                        break;
                    }
                    amount *= 0.5;
                }
                if improved.is_some() {
                    break;
                }
            }
            q = improved.ok_or("local geometry continuation reached its boundary")?;
        }
        Err("local geometry continuation did not converge".into())
    }

    #[cfg(test)]
    pub fn position_limits(&self) -> ([f64; 7], [f64; 7]) {
        (
            std::array::from_fn(|i| self.joints[i].lower),
            std::array::from_fn(|i| self.joints[i].upper),
        )
    }

    fn validate_q(&self, q: &[f64; 7]) -> Result<(), String> {
        // URDF limits are model metadata; the installed arm's physical stops
        // bound travel, so only reject values that cannot be computed.
        if q.iter().any(|value| !value.is_finite()) {
            return Err("joint position is non-finite".into());
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

    /// Local joint displacement for timing a Cartesian segment. This is a
    /// Jacobian estimate, not an absolute IK target or a reachability check.
    pub fn local_displacement(
        &self,
        q: &[f64; 7],
        tcp: &TcpTransform,
        target: &Pose,
    ) -> Result<[f64; 7], String> {
        let (pose, mut jacobian) = self.evaluate(q, tcp)?;
        let mut error = Twist::from_row_slice(&pose_error(&pose, target));
        let length = self.reach_radius(tcp);
        for row in 0..3 {
            jacobian.row_mut(row).scale_mut(1.0 / length);
            error[row] /= length;
        }
        let system = jacobian * jacobian.transpose() + SMatrix::<f64, 6, 6>::identity() * 1e-12;
        let displacement = jacobian.transpose()
            * system
                .cholesky()
                .ok_or("Cartesian timing factorization failed")?
                .solve(&error);
        let bounds = self.continuation_bounds(q)?;
        Ok(std::array::from_fn(|i| {
            // A local linearization can diverge near a singularity. Its timing
            // estimate uses the model's joint span; this does not clip the target.
            let span = bounds.upper[i] - bounds.lower[i];
            displacement[i].clamp(-span, span)
        }))
    }

    /// Static gravity torque in URDF joint coordinates, using the same link
    /// inertials as the official R1-7A model. Positive values are the torque
    /// the actuator must apply to hold the commanded configuration.
    pub fn gravity(&self, q: &[f64; 7]) -> Result<[f64; 7], String> {
        self.validate_q(q)?;
        let mut transform = Isometry3::identity();
        let mut origins = [Vector3::zeros(); 7];
        let mut axes = [Vector3::zeros(); 7];
        let mut coms = [Vector3::zeros(); 7];
        for (i, joint) in self.joints.iter().enumerate() {
            transform *= joint.origin;
            origins[i] = transform.translation.vector;
            axes[i] = transform.rotation * joint.axis.as_ref();
            transform *= Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&joint.axis, q[i]),
            );
            coms[i] = transform.translation.vector + transform.rotation * joint.link_com;
        }
        let mut torque = [0.0; 7];
        for i in 0..7 {
            let mut gravity_moment = 0.0;
            for (j, com) in coms.iter().enumerate().skip(i) {
                let force = Vector3::new(0.0, 0.0, -9.81 * self.joints[j].link_mass);
                gravity_moment += axes[i].dot(&(*com - origins[i]).cross(&force));
            }
            torque[i] = -gravity_moment;
        }
        if !torque.iter().all(|value| value.is_finite()) {
            return Err("non-finite gravity torque".into());
        }
        Ok(torque)
    }

    #[cfg(test)]
    pub fn jacobian(&self, q: &[f64; 7], tcp: &TcpTransform) -> Result<SMatrix<f64, 6, 7>, String> {
        self.evaluate(q, tcp).map(|(_, jacobian)| jacobian)
    }

    /// A single local weighted DLS step, with common (direction-preserving) scaling.
    /// Invalid dynamics, blocked constraints and unproductive numerical solutions
    /// return errors for the caller to stop/resynchronize. No hidden IK iteration.
    pub fn step(&self, input: StepInput) -> Result<StepCandidate, String> {
        let smoothed_input = input.clone();
        let StepInput {
            reference,
            measured,
            previous_velocity,
            target,
            dt,
            tcp,
            limits,
            smoothing,
            planning: _,
        } = input;
        if !dt.is_finite() || dt <= 0.0 {
            return Err("dt must be finite and positive".into());
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
            .all(|v| v.is_finite() && *v > 0.0)
        {
            return Err("motion limits must be finite positive values".into());
        }
        let max_step = limits.max_joint_step;
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
        let mut desired = match target {
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
                error * POSE_GAIN
            }
        };
        if let Some(state) = smoothing
            && !state.stopping
            && let Some(path) = state.path
        {
            desired = path.correct(&current, desired)?.0;
        }
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
        if smoothing.is_some() {
            let mut candidate = self.smoothed_candidate(
                &smoothed_input,
                current,
                jacobian,
                desired_bounded,
                raw,
                damping,
            )?;
            candidate.speed_scale *= task_scale;
            return Ok(candidate);
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
            // The Jacobian bounds instantaneous speed; the finite FK step can
            // be slightly faster due to curvature. Reserve the same headroom
            // before reaching the cap, rather than only after exceeding it.
            if speed > 0.0 {
                alpha_max = alpha_max.min(0.999 * bound / speed);
            }
        }
        for i in 0..7 {
            let acceleration = limits.joint_acceleration[i];
            let lower = measured[i] - limits.max_reference_error;
            let upper = measured[i] + limits.max_reference_error;
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
                ddq: std::array::from_fn(|i| (dq[i] - previous_velocity[i]) / dt),
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
        Ok(StepCandidate {
            q,
            dq,
            ddq: std::array::from_fn(|i| (dq[i] - previous_velocity[i]) / dt),
            speed_scale,
        })
    }

    fn braking_reference(
        &self,
        input: &StepInput,
        current: Pose,
        jacobian: Jacobian,
        desired: Twist,
        raw: SVector<f64, 7>,
        damping: f64,
    ) -> Result<(Twist, SVector<f64, 7>), String> {
        let state = input.smoothing.unwrap();
        let limits = input.limits;
        let dt = input.dt;
        let speeds: [f64; 7] =
            std::array::from_fn(|i| limits.joint_velocity[i].min(self.joints[i].velocity));
        let velocity = SVector::<f64, 7>::from_row_slice(&input.previous_velocity);
        let acceleration = SVector::<f64, 7>::from_row_slice(&state.acceleration);
        let task_velocity = jacobian * velocity;
        let ahead = std::array::from_fn(|i| input.reference[i] + velocity[i] * dt * 0.5);
        let behind = std::array::from_fn(|i| input.reference[i] - velocity[i] * dt * 0.5);
        let curvature = (self.evaluate(&ahead, &input.tcp)?.1
            - self.evaluate(&behind, &input.tcp)?.1)
            * velocity
            / dt;
        let task_acceleration = jacobian * acceleration + curvature;
        // Critically damped task jerk, using the current velocity and acceleration.
        // Bandwidth follows the configured jerk and joint motion, rather than an
        // instantaneous position-error velocity that a slow jerk ramp cannot follow.
        let rate_scale = (0..7)
            .map(|i| velocity[i].abs().max(raw[i].abs().min(speeds[i])))
            .fold(f64::EPSILON, f64::max);
        let omega = state
            .hold
            .map_or_else(
                || (state.jerk.into_iter().fold(f64::INFINITY, f64::min) / rate_scale).sqrt(),
                |hold| hold.bandwidth,
            )
            .min(0.25 / dt);
        let mut requested = if let LocalTarget::Pose(goal) = &input.target {
            Twist::from_row_slice(&pose_error(&current, goal)) * (omega / 3.0)
        } else {
            desired
        };
        if let Some(path) = state.path {
            let axis = path.axis_in_base()?;
            let perpendicular = Matrix3::identity() - axis * axis.transpose();
            let error = Twist::from_row_slice(&pose_error(&current, &path.origin));
            for block in 0..2 {
                let project = if (block == 0) == (path.axis < 3) {
                    perpendicular
                } else {
                    Matrix3::identity()
                };
                let offset = block * 3;
                let value = requested.fixed_rows::<3>(offset).into_owned();
                requested.fixed_rows_mut::<3>(offset).copy_from(
                    &((Matrix3::identity() - project) * value
                        + project * error.fixed_rows::<3>(offset).into_owned() * (omega / 3.0)),
                );
            }
        }
        let scale = (limits.linear_speed / requested.fixed_rows::<3>(0).norm().max(1e-300))
            .min(limits.angular_speed / requested.fixed_rows::<3>(3).norm().max(1e-300))
            .min(1.0);
        requested *= scale;
        let task_jerk =
            (requested - task_velocity) * (3.0 * omega * omega) - task_acceleration * (3.0 * omega);
        let next_acceleration = task_acceleration + task_jerk * dt;
        let desired = task_velocity + (next_acceleration - curvature) * dt;
        let task_weights = SMatrix::<f64, 6, 6>::from_diagonal(&Twist::from_fn(|i, _| {
            1.0 / if i < 3 {
                limits.linear_speed
            } else {
                limits.angular_speed
            }
        }));
        let normalized = task_weights * jacobian;
        let goal_system = (normalized * normalized.transpose()
            + SMatrix::<f64, 6, 6>::identity() * damping.powi(2))
        .cholesky()
        .ok_or("Cartesian goal factorization failed")?;
        let mut preferred = normalized.transpose() * goal_system.solve(&(task_weights * requested));
        if let Some(hold) = state.hold {
            let goal = SVector::<f64, 7>::from_fn(|i, _| {
                (hold.joints[i] - input.reference[i]) * (omega / 3.0)
            });
            let delta = goal - preferred;
            preferred += delta - normalized.transpose() * goal_system.solve(&(normalized * delta));
        }
        let posture = velocity
            + (acceleration
                + ((preferred - velocity) * (3.0 * omega * omega) - acceleration * (3.0 * omega))
                    * dt)
                * dt;
        Ok((desired, posture))
    }

    fn smoothed_candidate(
        &self,
        input: &StepInput,
        current: Pose,
        jacobian: Jacobian,
        desired: Twist,
        raw: SVector<f64, 7>,
        damping: f64,
    ) -> Result<StepCandidate, String> {
        let state = input.smoothing.unwrap();
        let limits = input.limits;
        let dt = input.dt;
        let speeds = std::array::from_fn(|i| limits.joint_velocity[i].min(self.joints[i].velocity));
        let guide = input
            .planning
            .filter(|_| !state.stopping)
            .and_then(|p| p.guide);
        let (desired, posture) = if state.stopping {
            self.braking_reference(input, current, jacobian, desired, raw, damping)?
        } else {
            let posture = if let Some(guide) = guide {
                if guide.axis >= 3 {
                    return Err("invalid planning guide axis".into());
                }
                SVector::<f64, 7>::from_fn(|i, _| {
                    (guide.tangent[i] * desired[guide.axis]
                        + 4.0 * (guide.position[i] - input.reference[i]))
                        .clamp(-speeds[i], speeds[i])
                })
            } else {
                SVector::<f64, 7>::zeros()
            };
            (desired, posture)
        };
        let mut weights = SMatrix::<f64, 6, 6>::identity();
        for i in 0..6 {
            weights[(i, i)] = 1.0
                / if i < 3 {
                    limits.linear_speed
                } else {
                    limits.angular_speed
                };
        }
        let task_weights = weights;
        if let Some(path) = state.path {
            weights = path.correct(&current, desired)?.1 * weights;
        }
        let a = weights * jacobian;
        let b = weights * desired;
        let task_hessian = a.transpose() * a;
        // Bound conditioning after path priorities are applied. Penalize joint
        // speed itself, so an unobservable null-space velocity also stops.
        // Continuity comes from hard derivative bounds, not a velocity-memory
        // objective that could preserve redundant motion indefinitely.
        let regularizer = SMatrix::<f64, 7, 7>::from_diagonal(&SVector::from_fn(|i, _| {
            if state.stopping {
                (1.0 / speeds[i].powi(2)).max(task_hessian.trace() * 32.0 * f64::EPSILON)
            } else {
                (damping.powi(2) / speeds[i].powi(2)).max(task_hessian.trace() * 1e-8)
            }
        }));
        let h = task_hessian + regularizer;
        let g = -a.transpose() * b - regularizer * posture;
        let mut window = trajectory::derivative_window(
            input.previous_velocity,
            state.acceleration,
            speeds,
            limits.joint_acceleration,
            state.jerk,
            dt,
        )?;
        if let Some(planning) = input.planning {
            let bounds = self.continuation_bounds(&planning.origin)?;
            trajectory::bound_position_window(
                &mut window,
                input.reference,
                input.previous_velocity,
                limits.joint_acceleration,
                state.jerk,
                bounds.lower,
                bounds.upper,
                dt,
            )?;
        }
        for i in 0..7 {
            window.low[i] = window.low[i]
                .max(-limits.max_joint_step / dt)
                .max((input.measured[i] - limits.max_reference_error - input.reference[i]) / dt);
            window.high[i] = window.high[i]
                .min(limits.max_joint_step / dt)
                .min((input.measured[i] + limits.max_reference_error - input.reference[i]) / dt);
            if window.low[i] > window.high[i] {
                return Err("Cartesian derivative and reference bounds conflict".into());
            }
        }
        if !state.stopping {
            let unconstrained = h
                .cholesky()
                .ok_or("Cartesian objective factorization failed")?
                .solve(&(-g));
            trajectory::approach_velocity_target(
                &mut window,
                input.previous_velocity,
                std::array::from_fn(|i| unconstrained[i]),
                speeds,
                state.jerk,
                dt,
            )?;
        }
        let dq = trajectory::bounded_quadratic(h, g, window, input.previous_velocity)?;
        let (q, ddq) = trajectory::integrate(input.reference, input.previous_velocity, dq, dt);
        for i in 0..7 {
            if !q[i].is_finite()
                || !ddq[i].is_finite()
                || dq[i].abs() > speeds[i] + 1e-10
                || ddq[i].abs() > limits.joint_acceleration[i] + 1e-9
                || (ddq[i] - state.acceleration[i]).abs() > state.jerk[i] * dt + 1e-8
                || (q[i] - input.reference[i]).abs() > limits.max_joint_step + 1e-12
                || (q[i] - input.measured[i]).abs() > limits.max_reference_error + 1e-12
            {
                return Err("smoothed output violates final joint derivatives".into());
            }
        }
        let next = self.fk(&q, &input.tcp)?;
        // Path accuracy is maintained by the correction objective above. Small
        // finite-step residuals are not an additional motion-failure gate.
        let achieved = Twist::from_row_slice(&pose_error(&current, &next)) / dt;
        // Cartesian speed is the task objective. A finite-step FK residual or
        // transient while joint derivatives settle is not a second hard speed
        // gate; the QP already enforces model joint speeds and derivatives.
        // Path-correction priorities belong to the objective, not the progress
        // metric: a transverse correction can briefly oppose its old velocity
        // while the commanded direction still makes useful progress.
        let requested = task_weights * desired;
        let projection = if requested.norm_squared() < 1e-20 {
            1.0
        } else {
            (task_weights * achieved).dot(&requested) / requested.norm_squared()
        };
        if !projection.is_finite() {
            return Err("invalid smoothed task projection".into());
        }
        Ok(StepCandidate {
            q,
            dq,
            ddq,
            speed_scale: projection.clamp(0.0, 1.0),
        })
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
            smoothing: None,
            planning: None,
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

    // Offline probes: match arm::advance's 100 Hz configuration, not the
    // older, more permissive test fixture above. No device/bus is constructed.
    fn rotation_diagnostic_limits() -> MotionLimits {
        MotionLimits {
            joint_velocity: [0.20; 7],
            joint_acceleration: [0.60; 7],
            max_reference_error: 0.60,
            max_joint_step: 0.01,
            linear_speed: 0.015,
            angular_speed: 0.15,
        }
    }

    #[test]
    fn rotation_jog_preserves_selected_tcp_and_axis() {
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        let k = Kinematics::load("right").unwrap();
        for z_offset in [0.0, 0.1] {
            let tcp = Pose {
                translation: [0.0, 0.0, z_offset],
                ..Pose::identity()
            };
            for frame in [Frame::Base, Frame::Tool] {
                for axis in 0..3 {
                    let mut twist = [0.0; 6];
                    twist[axis + 3] = 5.0_f64.to_radians();
                    let mut sample = StepInput {
                        dt: 0.01,
                        tcp,
                        limits: rotation_diagnostic_limits(),
                        ..input(q, velocity(twist, frame))
                    };
                    let start_tcp = k.fk(&q, &tcp).unwrap();
                    let start_link = k.fk(&q, &Pose::identity()).unwrap();
                    let desired_axis = if frame == Frame::Base {
                        Vector3::ith(axis, 1.0)
                    } else {
                        to_isometry(&start_tcp).unwrap().rotation * Vector3::ith(axis, 1.0)
                    };
                    for tick in 0..200 {
                        let c = k.step(sample.clone()).unwrap_or_else(|e| {
                            panic!("offset={z_offset} {frame:?} axis={axis} tick={tick}: {e}")
                        });
                        check_bounds(&k, &sample, &c);
                        sample.reference = c.q;
                        sample.measured = c.q;
                        sample.previous_velocity = c.dq;
                    }
                    let end_tcp = k.fk(&sample.reference, &tcp).unwrap();
                    let end_link = k.fk(&sample.reference, &Pose::identity()).unwrap();
                    let tcp_delta = Twist::from(pose_error(&start_tcp, &end_tcp));
                    let link_delta = Twist::from(pose_error(&start_link, &end_link));
                    let angular = tcp_delta.fixed_rows::<3>(3).into_owned();
                    let axis_error = angular
                        .normalize()
                        .dot(&desired_axis)
                        .clamp(-1.0, 1.0)
                        .acos()
                        .to_degrees();
                    let tcp_drift = tcp_delta.fixed_rows::<3>(0).norm() * 1000.0;
                    let link_motion = link_delta.fixed_rows::<3>(0).norm() * 1000.0;
                    // Removing the tool displacement from each TCP pose must
                    // account for Link7 motion about the fixed selected point.
                    let start_r = to_isometry(&start_tcp).unwrap().rotation;
                    let end_r = to_isometry(&end_tcp).unwrap().rotation;
                    let tool_vector = Vector3::new(0.0, 0.0, z_offset);
                    let expected_link_delta = start_r * tool_vector - end_r * tool_vector;
                    assert!((link_delta.fixed_rows::<3>(0) - expected_link_delta).norm() < 0.00005);
                    assert!(
                        tcp_drift < 0.05,
                        "{frame:?} axis={axis}: TCP moved {tcp_drift} mm"
                    );
                    assert!(
                        axis_error < 0.1,
                        "{frame:?} axis={axis}: wrong rotation axis {axis_error} deg"
                    );
                    if z_offset == 0.1 && frame == Frame::Tool && axis < 2 {
                        assert!(
                            link_motion > 10.0,
                            "Link7 must translate to keep offset TCP fixed"
                        );
                        assert!(
                            sample.previous_velocity[..6].iter().any(|v| v.abs() > 0.01),
                            "compensation must involve more than J7"
                        );
                    }
                    eprintln!(
                        "rotation-tcp100 offset_m={z_offset} {frame:?} axis={axis} tcp_drift_mm={tcp_drift:.6} link7_motion_mm={link_motion:.6} axis_error_deg={axis_error:.6} angle_rad={:.6} final_dq={:?}",
                        angular.norm(),
                        sample.previous_velocity
                    );
                }
            }
        }
    }

    #[test]
    fn rotation_jog_at_maximum_speed_does_not_interrupt() {
        let k = Kinematics::load("right").unwrap();
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        let mut sample = StepInput {
            dt: 0.01,
            limits: rotation_diagnostic_limits(),
            ..input(q, velocity([0.0, 0.0, 0.0, 0.15, 0.0, 0.0], Frame::Base))
        };
        for tick in 0..200 {
            let candidate = k
                .step(sample.clone())
                .unwrap_or_else(|e| panic!("tick={tick}: {e}"));
            check_bounds(&k, &sample, &candidate);
            sample.reference = candidate.q;
            sample.measured = candidate.q;
            sample.previous_velocity = candidate.dq;
        }
    }

    #[test]
    fn jerk_bounded_rotation_preserves_offset_tcp_through_start_and_braking() {
        let k = Kinematics::load("right").unwrap();
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        let tcp = Pose {
            translation: [0.0, 0.0, 0.1],
            ..Pose::identity()
        };
        let origin = k.fk(&q, &tcp).unwrap();
        let path = CartesianPath {
            origin,
            axis: 3,
            frame: Frame::Base,
        };
        let mut sample = StepInput {
            dt: 0.01,
            tcp,
            limits: rotation_diagnostic_limits(),
            smoothing: Some(SmoothingState {
                hold: None,
                acceleration: [0.0; 7],
                jerk: [trajectory::JOINT_JERK; 7],
                path: Some(path),
                stopping: false,
            }),
            ..input(
                q,
                velocity([0.0, 0.0, 0.0, 0.0872664626, 0.0, 0.0], Frame::Base),
            )
        };
        for tick in 0..350 {
            if tick == 200 {
                sample.target = velocity([0.0; 6], Frame::Base);
                sample.smoothing.as_mut().unwrap().stopping = true;
            }
            let c = k
                .step(sample.clone())
                .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
            check_bounds(&k, &sample, &c);
            for i in 0..7 {
                let old = sample.smoothing.as_ref().unwrap().acceleration[i];
                assert!((c.ddq[i] - old).abs() <= trajectory::JOINT_JERK * sample.dt + 1e-8);
                assert!(
                    (c.dq[i] - sample.previous_velocity[i] - c.ddq[i] * sample.dt).abs() < 1e-12
                );
            }
            let next = k.fk(&c.q, &tcp).unwrap();
            let error = pose_error(&origin, &next);
            assert!(error[..3].iter().map(|x| x * x).sum::<f64>().sqrt() <= 0.00005);
            sample.reference = c.q;
            sample.measured = c.q;
            sample.previous_velocity = c.dq;
            sample.smoothing.as_mut().unwrap().acceleration = c.ddq;
        }
        assert!(
            sample.previous_velocity.iter().all(|v| v.abs() < 1e-5),
            "{:?}",
            sample.previous_velocity
        );
    }

    #[test]
    fn smoothed_every_cartesian_axis_and_frame_preserves_path_under_jitter() {
        for side in ["left", "right"] {
            let k = Kinematics::load(side).unwrap();
            for q in [
                interior(),
                [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133],
            ] {
                for frame in [Frame::Base, Frame::Tool] {
                    for axis in 0..6 {
                        let tcp = Pose {
                            translation: [0.0, 0.0, 0.1],
                            ..Pose::identity()
                        };
                        let origin = k.fk(&q, &tcp).unwrap();
                        let path = CartesianPath {
                            origin,
                            axis,
                            frame,
                        };
                        let mut twist = [0.0; 6];
                        twist[axis] = if axis < 3 { 0.015 } else { 0.15 };
                        let mut sample = StepInput {
                            tcp,
                            limits: rotation_diagnostic_limits(),
                            smoothing: Some(SmoothingState {
                                hold: None,
                                acceleration: [0.0; 7],
                                jerk: [trajectory::JOINT_JERK; 7],
                                path: Some(path),
                                stopping: false,
                            }),
                            ..input(q, velocity(twist, frame))
                        };
                        for (tick, dt) in [0.040, 0.002, 0.007, 0.003]
                            .into_iter()
                            .cycle()
                            .take(400)
                            .enumerate()
                        {
                            sample.dt = dt;
                            if tick == 160 {
                                sample.target = velocity([0.0; 6], frame);
                                sample.smoothing.as_mut().unwrap().stopping = true;
                            }
                            let c = k.step(sample.clone()).unwrap_or_else(|e| {
                                panic!("{side} q={q:?} {frame:?} axis={axis} tick={tick}: {e}")
                            });
                            check_bounds(&k, &sample, &c);
                            for i in 0..7 {
                                assert!(
                                    (c.ddq[i] - sample.smoothing.as_ref().unwrap().acceleration[i])
                                        .abs()
                                        <= trajectory::JOINT_JERK * dt + 1e-8
                                );
                            }
                            path.check(&k.fk(&c.q, &tcp).unwrap()).unwrap_or_else(|e| {
                                panic!("{side} q={q:?} {frame:?} axis={axis} tick={tick}: {e}; deviation={:?}",
                                    pose_error(&origin, &k.fk(&c.q, &tcp).unwrap()))
                            });
                            sample.reference = c.q;
                            sample.measured = c.q;
                            sample.previous_velocity = c.dq;
                            sample.smoothing.as_mut().unwrap().acceleration = c.ddq;
                        }
                        assert!(
                            sample.previous_velocity.iter().all(|v| v.abs() < 1e-5),
                            "{side} {frame:?} {axis}: {:?}",
                            sample.previous_velocity
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn held_low_speed_cartesian_jogs_have_a_steady_reference_after_ramp() {
        for side in ["left", "right"] {
            let k = Kinematics::load(side).unwrap();
            for frame in [Frame::Base, Frame::Tool] {
                for axis in 0..6 {
                    let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
                    let tcp = Pose {
                        translation: [0.0, 0.0, 0.1],
                        ..Pose::identity()
                    };
                    let origin = k.fk(&q, &tcp).unwrap();
                    let path = CartesianPath {
                        origin,
                        axis,
                        frame,
                    };
                    let direction = path.axis_in_base().unwrap();
                    let requested = if axis < 3 { 0.0005 } else { 0.005 };
                    let mut twist = [0.0; 6];
                    twist[axis] = requested;
                    let mut sample = StepInput {
                        tcp,
                        limits: rotation_diagnostic_limits(),
                        smoothing: Some(SmoothingState {
                            hold: None,
                            acceleration: [0.0; 7],
                            jerk: [trajectory::JOINT_JERK; 7],
                            path: Some(path),
                            stopping: false,
                        }),
                        ..input(q, velocity(twist, frame))
                    };
                    let mut steady = Vec::new();
                    for tick in 0..360 {
                        sample.dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                        let old = k.fk(&sample.reference, &tcp).unwrap();
                        let c = k.step(sample.clone()).unwrap_or_else(|e| {
                            panic!("{side} {frame:?} axis={axis} tick={tick}: {e}")
                        });
                        check_bounds(&k, &sample, &c);
                        let next = k.fk(&c.q, &tcp).unwrap();
                        if tick >= 160 {
                            let effective =
                                Twist::from_row_slice(&pose_error(&old, &next)) / sample.dt;
                            steady.push(
                                effective
                                    .fixed_rows::<3>(if axis < 3 { 0 } else { 3 })
                                    .dot(&direction),
                            );
                        }
                        sample.reference = c.q;
                        sample.measured = c.q;
                        sample.previous_velocity = c.dq;
                        sample.smoothing.as_mut().unwrap().acceleration = c.ddq;
                    }
                    let mean = steady.iter().sum::<f64>() / steady.len() as f64;
                    let min = steady.iter().copied().fold(f64::INFINITY, f64::min);
                    let max = steady.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    assert!(
                        mean > requested * 0.98
                            && mean < requested * 1.02
                            && (max - min) / mean < 0.03,
                        "{side} {frame:?} axis={axis} mean={mean:e} spread={:e}",
                        (max - min) / mean
                    );
                }
            }
        }
    }

    fn check_bounds(k: &Kinematics, i: &StepInput, c: &StepCandidate) {
        assert!(c.speed_scale.is_finite() && (0.0..=1.0).contains(&c.speed_scale));
        let old = k.fk(&i.reference, &i.tcp).unwrap();
        let new = k.fk(&c.q, &i.tcp).unwrap();
        let effective = Twist::from(pose_error(&old, &new)) / i.dt;
        if i.smoothing.is_none() {
            assert!(effective.fixed_rows::<3>(0).norm() <= i.limits.linear_speed + 1e-10);
            assert!(effective.fixed_rows::<3>(3).norm() <= i.limits.angular_speed + 1e-10);
        } else {
            // During derivative-limited transients the requested Cartesian
            // speed is an objective. The integrated motion remains bounded by
            // joint speeds and the actual chain length throughout the step.
            let joint_speed: f64 = k
                .speed_limits()
                .iter()
                .zip(i.limits.joint_velocity)
                .map(|(model, requested)| model.min(requested))
                .sum();
            assert!(
                effective.fixed_rows::<3>(0).norm() <= k.reach_radius(&i.tcp) * joint_speed + 1e-10
            );
            assert!(effective.fixed_rows::<3>(3).norm() <= joint_speed + 1e-10);
        }
        for j in 0..7 {
            assert!(c.q[j].is_finite() && c.dq[j].is_finite());
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
    fn model_speed_and_reach_bounds_use_urdf_transforms_and_tcp() {
        for side in ["right", "left"] {
            let model = Kinematics::load(side).unwrap();
            assert_eq!(
                model.speed_limits(),
                [18.7, 18.7, 33.4, 33.4, 33.4, 37.7, 37.7]
            );
            for tcp in [
                Pose::identity(),
                Pose {
                    translation: [0.2, -0.1, 0.3],
                    ..Pose::identity()
                },
            ] {
                assert!(
                    model
                        .fk(&interior(), &tcp)
                        .unwrap()
                        .translation
                        .iter()
                        .map(|v| v * v)
                        .sum::<f64>()
                        .sqrt()
                        <= model.reach_radius(&tcp)
                );
                let small = model
                    .cartesian_speed_limits(&interior(), &tcp, [0.1; 7])
                    .unwrap();
                let large = model
                    .cartesian_speed_limits(&interior(), &tcp, [1.0; 7])
                    .unwrap();
                assert!(small.0 > 0.0 && small.1 > 0.0);
                assert!((large.0 - 10.0 * small.0).abs() < 1e-12);
                assert!((large.1 - 10.0 * small.1).abs() < 1e-12);
                assert!(large.0 > 0.015 && large.1 > 0.15);
                assert_eq!(
                    model
                        .cartesian_speed_limits(&interior(), &tcp, [1000.0; 7])
                        .unwrap(),
                    model
                        .cartesian_speed_limits(&interior(), &tcp, model.speed_limits())
                        .unwrap()
                );
            }
            for limit in [0.0, -1.0, f64::INFINITY, f64::NAN] {
                assert!(
                    model
                        .cartesian_speed_limits(&interior(), &Pose::identity(), [limit; 7])
                        .is_err()
                );
            }
        }
        let planar = Kinematics::from_urdf(&planar_urdf()).unwrap();
        assert_eq!(planar.reach_radius(&Pose::identity()), 7.0);
        assert_eq!(
            planar.reach_radius(&Pose {
                translation: [0.0, 0.0, 2.0],
                ..Pose::identity()
            }),
            9.0
        );
        assert_eq!(
            planar
                .cartesian_speed_limits(&[0.0; 7], &Pose::identity(), [1.0; 7])
                .unwrap(),
            (21.0, 7.0)
        );
    }

    #[test]
    fn geometry_continuation_has_no_one_metre_radius_or_four_metre_loop_budget() {
        use crate::calibration_path::{PreviewRequest, preview};
        let mut robot =
            urdf_rs::read_from_string(include_str!("../../resources/models/right/robot.urdf"))
                .unwrap();
        for joint in &mut robot.joints {
            for value in &mut joint.origin.xyz.0 {
                *value *= 20.0;
            }
        }
        let large = Kinematics::from_urdf(&urdf_rs::write_to_string(&robot).unwrap()).unwrap();
        let ordinary = Kinematics::load("right").unwrap();
        assert!(
            (large.reach_radius(&Pose::identity()) / ordinary.reach_radius(&Pose::identity())
                - 20.0)
                .abs()
                < 1e-12
        );
        let result = preview(
            &large,
            "right",
            interior(),
            Pose::identity(),
            PreviewRequest::default(),
        )
        .unwrap();
        assert!(result.ranges[0].negative < -4.0);
        assert!(result.program.amplitude > 4.0);
        assert_eq!(result.points.last().unwrap().joints, interior());
        for pair in result.points.windows(2) {
            assert!((0..7).all(|axis| (pair[1].joints[axis] - pair[0].joints[axis]).abs() <= 0.01 + 1e-12));
        }
    }

    #[test]
    fn finite_longer_periods_and_distant_pose_goals_still_generate_local_bounded_steps() {
        let model = Kinematics::load("right").unwrap();
        for dt in [1e-7, 0.051, 0.101, 0.2] {
            let sample = StepInput {
                dt,
                ..input(
                    interior(),
                    velocity([0.001, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base),
                )
            };
            let candidate = model.step(sample.clone()).unwrap();
            check_bounds(&model, &sample, &candidate);
        }
        let mut sample = input(interior(), velocity([0.0; 6], Frame::Base));
        let pose = model.fk(&sample.reference, &sample.tcp).unwrap();
        sample.target = LocalTarget::Pose(
            offset_pose(&pose, [0.3, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base).unwrap(),
        );
        let candidate = model.step(sample.clone()).unwrap();
        check_bounds(&model, &sample, &candidate);
        assert!(candidate.q != sample.reference);
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
    fn joint_velocity_acceleration_reference_and_increment_constraints() {
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
        // Official lower/upper values are model metadata; candidates at those
        // values must still satisfy every dynamic constraint.
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
    fn finite_poses_outside_urdf_limits_still_support_fk_and_local_steps() {
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let (lower, _) = k.position_limits();
            let mut q = interior();
            // Observed installation offsets: J2 and J4 sit slightly below the
            // official model's zero/lower bound.
            q[1] = lower[1] - 0.035;
            q[3] = lower[3] - 0.022;
            let pose = k.fk(&q, &Pose::identity()).unwrap();
            assert!(pose.translation.iter().all(|value| value.is_finite()));
            let jacobian = k.jacobian(&q, &Pose::identity()).unwrap();
            assert!(jacobian.iter().all(|value| value.is_finite()));
            let joint_velocity = [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let twist: [f64; 6] = std::array::from_fn(|row| {
                (0..7)
                    .map(|column| jacobian[(row, column)] * joint_velocity[column])
                    .sum()
            });
            let i = input(q, velocity(twist, Frame::Base));
            let candidate = k.step(i.clone()).unwrap();
            check_bounds(&k, &i, &candidate);
            assert!(candidate.q[0] > q[0]);
        }
    }

    #[test]
    fn observed_installation_pose_supports_every_cartesian_jog_axis() {
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            for frame in [Frame::Base, Frame::Tool] {
                for axis in 0..6 {
                    let mut twist = [0.0; 6];
                    twist[axis] = if axis < 3 { 0.01 } else { 0.05 };
                    let sample = input(q, velocity(twist, frame));
                    let candidate = k
                        .step(sample.clone())
                        .unwrap_or_else(|error| panic!("{side} {frame:?} axis {axis}: {error}"));
                    check_bounds(&k, &sample, &candidate);
                    assert!(
                        candidate.speed_scale > 0.05,
                        "{side} {frame:?} axis {axis}: scale {}",
                        candidate.speed_scale
                    );
                    assert!(
                        candidate
                            .q
                            .iter()
                            .zip(q)
                            .any(|(after, before)| (after - before).abs() > 1e-8)
                    );
                }
            }
        }
    }

    #[test]
    fn gravity_feedforward_has_expected_holding_sign_for_the_shoulder_joints() {
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let torque = k.gravity(&q).unwrap();
            assert!(
                torque.iter().all(|value| value.is_finite()),
                "{side}: {torque:?}"
            );
            assert!(torque[1] > 0.1, "{side}: {torque:?}");
            assert!(torque[3] > 0.1, "{side}: {torque:?}");
            assert!(
                torque.iter().all(|value| value.abs() < 30.0),
                "{side}: {torque:?}"
            );
        }
    }

    #[test]
    fn observed_installation_pose_sustains_continuous_cartesian_jog() {
        let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
        let mut limits = limits();
        limits.joint_velocity = [0.2; 7];
        limits.joint_acceleration = [0.6; 7];
        limits.max_reference_error = 0.6;
        limits.linear_speed = 0.015;
        limits.angular_speed = 0.15;
        for side in ["right", "left"] {
            let k = Kinematics::load(side).unwrap();
            let tcp = Pose::identity();
            for frame in [Frame::Base, Frame::Tool] {
                for axis in 0..6 {
                    let mut reference = q;
                    let mut measured = q;
                    let mut previous_velocity = [0.0; 7];
                    let initial = k.fk(&q, &tcp).unwrap();
                    let mut twist = [0.0; 6];
                    twist[axis] = if axis < 3 { 0.01 } else { 0.1 };
                    for _ in 0..100 {
                        let candidate = k
                            .step(StepInput {
                                reference,
                                measured,
                                previous_velocity,
                                target: velocity(twist, frame),
                                dt: 0.02,
                                tcp,
                                limits,
                                smoothing: None,
                                planning: None,
                            })
                            .unwrap_or_else(|error| {
                                panic!("{side} {frame:?} axis {axis}: {error}")
                            });
                        measured = reference;
                        reference = candidate.q;
                        previous_velocity = candidate.dq;
                    }
                    let final_pose = k.fk(&reference, &tcp).unwrap();
                    let error = pose_error(&initial, &final_pose);
                    assert!(
                        error[..3].iter().map(|v| v * v).sum::<f64>().sqrt() > 1e-3
                            || error[3..].iter().map(|v| v * v).sum::<f64>().sqrt() > 1e-3,
                        "{side} {frame:?} axis {axis} did not move"
                    );
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
        for dt in [0.0, -0.01, f64::INFINITY, f64::NAN] {
            assert!(k.step(StepInput { dt, ..i.clone() }).is_err());
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
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
        assert!(
            k.step(StepInput {
                previous_velocity: [f64::MAX; 7],
                ..i.clone()
            })
            .is_err()
        );
        assert!(
            k.step(StepInput {
                target: velocity([f64::MAX; 6], Frame::Tool),
                ..i.clone()
            })
            .is_err()
        );
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
        let candidate = k
            .step(StepInput {
                target: LocalTarget::Pose(goal),
                ..i.clone()
            })
            .unwrap();
        check_bounds(&k, &i, &candidate);
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
