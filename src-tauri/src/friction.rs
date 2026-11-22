//! Pure, empirical identification from feedback torque minus gravity at actual q.
//!
//! This residual can contain gravity/model error as well as friction. There is
//! deliberately no directional offset: it would be indistinguishable from Fc.
//! A shared nonnegative inertia can explain acceleration. Fits use the runtime's
//! tanh smoothing. Compensation budgets are metadata, never measurement limits.

use crate::tuning::FrictionCurve;
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use std::{cell::RefCell, collections::HashMap, rc::Rc};

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    /// rad/s, retaining the measured direction.
    pub velocity: f64,
    /// rad/s².
    pub acceleration: f64,
    /// Feedback torque minus modeled gravity at measured q, in Nm.
    pub effort: f64,
    /// Positive relative least-squares weight.
    pub weight: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Identification {
    pub axis: usize,
    pub source: String,
    /// Number of finite, nonzero-velocity, positive-weight observations.
    pub samples: usize,
    pub positive: Option<DirectionFit>,
    pub negative: Option<DirectionFit>,
    /// Shared nuisance inertia, in kg m².
    pub inertia: f64,
    /// Actual compensation budget supplied by the caller, in Nm.
    #[serde(default)]
    pub max_friction: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct DirectionFit {
    pub curve: FrictionCurve,
    pub samples: usize,
    /// Signed range of the observations, in rad/s.
    pub velocity_min: f64,
    pub velocity_max: f64,
    /// Weighted residual RMSE including the nuisance inertia, in Nm.
    pub rmse: f64,
    pub points: Vec<FitPoint>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FitPoint {
    pub velocity: f64,
    /// Weighted mean of real measured effort residuals, retaining their sign.
    pub effort: f64,
    /// Mean prediction at those samples: unclipped fitted friction plus inertia * a.
    pub predicted: f64,
    pub samples: usize,
}

#[derive(Clone, Copy, PartialEq)]
enum Shape {
    Basic,
    Stribeck(f64),
}

#[derive(Clone, Copy)]
struct Model {
    shapes: [Option<Shape>; 2],
    inertia: bool,
}

struct Row {
    sample: Sample,
    direction: usize,
    weight: f64,
    smooth: f64,
    velocity: f64,
    acceleration: f64,
    effort: f64,
}

struct Data {
    rows: Vec<Row>,
    effective_count: f64,
    velocity_unit: f64,
    effort_unit: f64,
    velocity_scale: f64,
    statistics: [RefCell<HashMap<u64, Rc<Statistics>>>; 2],
}

struct Fit {
    model: Model,
    statistics: [Option<Rc<Statistics>>; 2],
    columns: [Vec<usize>; 2],
    inertia_column: Option<usize>,
    coefficients: Vec<f64>,
    score: f64,
}

/// Identify each observable direction without I/O or any motion policy.
///
/// The caller supplies an explicit budget or the measured residual's absolute
/// peak as metadata only, including zero. Nonfinite data and
/// exact zero velocities are not fit; there is no speed floor or outlier gate.
/// Unobservable Stribeck decay falls back to Fc == Fs, and an unobservable
/// direction is None. Search bounds come only from observed speeds. BIC and a
/// normalized Jacobian check avoid fitting an unsupported static-friction tail.
pub fn identify(
    axis: usize,
    samples: &[Sample],
    velocity_scale: f64,
    max_friction: f64,
) -> Identification {
    let valid: Vec<_> = samples
        .iter()
        .copied()
        .filter(|s| {
            s.velocity.is_finite()
                && s.velocity != 0.0
                && s.acceleration.is_finite()
                && s.effort.is_finite()
                && s.weight.is_finite()
                && s.weight > 0.0
        })
        .collect();
    let mut result = Identification {
        axis,
        source: "feedback_effort_residual".into(),
        samples: valid.len(),
        positive: None,
        negative: None,
        inertia: 0.0,
        max_friction: if max_friction.is_finite() && max_friction >= 0.0 {
            max_friction
        } else {
            0.0
        },
    };
    if valid.is_empty() || !velocity_scale.is_finite() || velocity_scale <= 0.0 {
        return result;
    }

    // Normalize units and weights before forming a small least-squares system.
    let peak = |f: fn(&Sample) -> f64| valid.iter().map(f).fold(0.0, f64::max);
    let velocity_unit = peak(|s| s.velocity.abs());
    let acceleration_unit = peak(|s| s.acceleration.abs());
    let effort_peak = peak(|s| s.effort.abs());
    // All-zero residuals need a unit for normalization, not an invented signal.
    let effort_unit = if effort_peak > 0.0 { effort_peak } else { 1.0 };
    let weight_unit = peak(|s| s.weight);
    let rows = valid
        .into_iter()
        .map(|sample| Row {
            direction: usize::from(sample.velocity < 0.0),
            weight: sample.weight / weight_unit,
            smooth: (sample.velocity / velocity_scale).tanh(),
            velocity: sample.velocity / velocity_unit,
            acceleration: if acceleration_unit > 0.0 {
                sample.acceleration / acceleration_unit
            } else {
                0.0
            },
            effort: sample.effort / effort_unit,
            sample,
        })
        .collect();
    let mut data = Data {
        rows,
        effective_count: 0.0,
        velocity_unit,
        effort_unit,
        velocity_scale,
        statistics: std::array::from_fn(|_| RefCell::new(HashMap::new())),
    };

    let mut shapes = [None; 2];
    let mut searches: [Vec<f64>; 2] = std::array::from_fn(|_| Vec::new());
    for direction in 0..2 {
        let rows: Vec<_> = data
            .rows
            .iter()
            .filter(|r| r.direction == direction)
            .collect();
        let mut speeds: Vec<_> = rows.iter().map(|r| r.sample.velocity.abs()).collect();
        speeds.sort_by(f64::total_cmp);
        speeds.dedup();
        if speeds.len() < 2 || effective_count(rows.iter().map(|r| r.weight)) <= 3.0 {
            continue;
        }
        let basis = DMatrix::from_fn(rows.len(), 2, |i, j| {
            if j == 0 {
                rows[i].smooth
            } else {
                rows[i].velocity
            }
        });
        let weights: Vec<_> = rows.iter().map(|r| r.weight).collect();
        if normalized_gram(&basis, &weights).is_none() {
            continue;
        }
        shapes[direction] = Some(Shape::Basic);
        if speeds.len() >= 4 && effective_count(weights.iter().copied()) > 5.0 {
            let lo = speeds[0].ln();
            let hi = speeds[speeds.len() - 1].ln();
            // Both ends must have actual support; endpoint scales themselves
            // do not identify a decay within the measured range.
            searches[direction] = (1..16)
                .map(|i| (lo + (hi - lo) * i as f64 / 16.0).exp())
                .filter(|v| v.is_finite() && *v > speeds[0] && *v < speeds[speeds.len() - 1])
                .collect();
        }
    }
    // Do not let an unexcited direction contaminate the shared inertia.
    data.rows.retain(|r| shapes[r.direction].is_some());
    if data.rows.is_empty() {
        return result;
    }
    data.effective_count = effective_count(data.rows.iter().map(|r| r.weight));

    let mut best: Option<Fit> = None;
    for inertia in [false, true] {
        if inertia && acceleration_unit == 0.0 {
            continue;
        }
        let Some(mut fit) = fit_model(&data, Model { shapes, inertia }) else {
            continue;
        };
        // Coordinate search of the two decay scales keeps their common
        // acceleration term coupled, rather than fitting directions in isolation.
        for _ in 0..3 {
            let old_score = fit.score;
            for (direction, search) in searches.iter().enumerate() {
                if search.is_empty() {
                    continue;
                }
                let mut candidates = search.clone();
                if let Some(Shape::Stribeck(vs)) = fit.model.shapes[direction] {
                    candidates.push(vs);
                    candidates.sort_by(f64::total_cmp);
                    candidates.dedup();
                }
                for vs in candidates.iter().copied() {
                    let mut model = fit.model;
                    model.shapes[direction] = Some(Shape::Stribeck(vs));
                    if let Some(candidate) = fit_model(&data, model)
                        && candidate.score < fit.score
                    {
                        fit = candidate;
                    }
                }
                if let Some(Shape::Stribeck(vs)) = fit.model.shapes[direction] {
                    let lo = candidates.iter().copied().rfind(|v| *v < vs);
                    let hi = candidates.iter().copied().find(|v| *v > vs);
                    if let (Some(lo), Some(hi)) = (lo, hi) {
                        // Local log-scale refinement; no torque trial grid.
                        let (mut lo, mut hi) = (lo.ln(), hi.ln());
                        let ratio = (5.0_f64.sqrt() - 1.0) / 2.0;
                        let model = fit.model;
                        let mut evaluate = |log_vs: f64| {
                            let mut model = model;
                            model.shapes[direction] = Some(Shape::Stribeck(log_vs.exp()));
                            let Some(candidate) = fit_model(&data, model) else {
                                return f64::INFINITY;
                            };
                            let score = candidate.score;
                            if score < fit.score {
                                fit = candidate;
                            }
                            score
                        };
                        let mut left = hi - ratio * (hi - lo);
                        let mut right = lo + ratio * (hi - lo);
                        let mut left_score = evaluate(left);
                        let mut right_score = evaluate(right);
                        for _ in 0..20 {
                            if left_score <= right_score {
                                hi = right;
                                right = left;
                                right_score = left_score;
                                left = hi - ratio * (hi - lo);
                                left_score = evaluate(left);
                            } else {
                                lo = left;
                                left = right;
                                left_score = right_score;
                                right = lo + ratio * (hi - lo);
                                right_score = evaluate(right);
                            }
                        }
                    }
                }
            }
            if fit.score >= old_score - f64::EPSILON.sqrt() {
                break;
            }
        }
        if best.as_ref().is_none_or(|b| fit.score < b.score) {
            best = Some(fit);
        }
    }
    if let Some(fit) = best {
        let inertia = fit.inertia_column.map_or(0.0, |i| {
            unscale(fit.coefficients[i], effort_unit, acceleration_unit)
        });
        if inertia.is_finite() {
            result.inertia = inertia;
            result.positive = direction_fit(&data, &fit, 0);
            result.negative = direction_fit(&data, &fit, 1);
        }
    }
    result
}

fn effective_count(weights: impl Iterator<Item = f64>) -> f64 {
    let (sum, squared) = weights.fold((0.0, 0.0), |(s, q), w| (s + w, q + w * w));
    if squared > 0.0 {
        sum * sum / squared
    } else {
        0.0
    }
}

/// Complete fixed-scale normal equations over real rows, including every
/// friction/acceleration/decay cross-product. No representative rows or means.
struct Statistics {
    gram: DMatrix<f64>,
    rhs: DVector<f64>,
}

fn features(row: &Row, shape: Shape) -> ([f64; 5], usize) {
    match shape {
        Shape::Basic => ([row.smooth, row.velocity, row.acceleration, 0.0, 0.0], 3),
        Shape::Stribeck(vs) => {
            let squared = (row.sample.velocity.abs() / vs).powi(2);
            let decay = (-squared).exp();
            let derivative = if squared.is_finite() {
                row.smooth * 2.0 * squared * decay
            } else {
                0.0
            };
            (
                [
                    row.smooth * (1.0 - decay),
                    row.smooth * decay,
                    row.velocity,
                    row.acceleration,
                    derivative,
                ],
                5,
            )
        }
    }
}

impl Data {
    fn statistics(&self, direction: usize, shape: Shape) -> Rc<Statistics> {
        let key = match shape {
            Shape::Basic => 0,
            Shape::Stribeck(vs) => vs.to_bits(),
        };
        if let Some(statistics) = self.statistics[direction].borrow().get(&key) {
            return Rc::clone(statistics);
        }
        let p = if shape == Shape::Basic { 3 } else { 5 };
        let mut gram = DMatrix::zeros(p, p);
        let mut rhs = DVector::zeros(p);
        for row in self.rows.iter().filter(|r| r.direction == direction) {
            let (values, _) = features(row, shape);
            for j in 0..p {
                let weighted = row.weight * values[j];
                rhs[j] += weighted * row.effort;
                for k in 0..=j {
                    gram[(j, k)] += weighted * values[k];
                }
            }
        }
        for j in 0..p {
            for k in 0..j {
                gram[(k, j)] = gram[(j, k)];
            }
        }
        let statistics = Rc::new(Statistics { gram, rhs });
        self.statistics[direction]
            .borrow_mut()
            .insert(key, Rc::clone(&statistics));
        statistics
    }
}

/// Column normalization makes the rank decision independent of SI units.
/// sqrt(epsilon) is a numerical observability tolerance, not a speed gate.
fn normalized_gram(basis: &DMatrix<f64>, weights: &[f64]) -> Option<(DMatrix<f64>, Vec<f64>)> {
    if basis.nrows() < basis.ncols() {
        return None;
    }
    let gram = DMatrix::from_fn(basis.ncols(), basis.ncols(), |j, k| {
        (0..basis.nrows())
            .map(|i| weights[i] * basis[(i, j)] * basis[(i, k)])
            .sum()
    });
    normalize_normal(gram)
}

fn normalize_normal(mut gram: DMatrix<f64>) -> Option<(DMatrix<f64>, Vec<f64>)> {
    let p = gram.ncols();
    if p == 0 {
        return None;
    }
    let norms: Vec<_> = (0..p).map(|j| gram[(j, j)].sqrt()).collect();
    if norms.iter().any(|n| !n.is_finite() || *n == 0.0) {
        return None;
    }
    for j in 0..p {
        for k in 0..p {
            gram[(j, k)] = (gram[(j, k)] / norms[j]) / norms[k];
        }
    }
    if gram.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let eigenvalues = gram.clone().symmetric_eigen().eigenvalues;
    if eigenvalues.min() <= eigenvalues.max() * f64::EPSILON.sqrt() {
        return None;
    }
    Some((gram, norms))
}

/// At most seven unknowns: active-set NNLS on complete normal equations.
fn nnls(gram: DMatrix<f64>, mut rhs: DVector<f64>) -> Option<Vec<f64>> {
    let (gram, norms) = normalize_normal(gram)?;
    let p = gram.ncols();
    for j in 0..p {
        rhs[j] /= norms[j];
    }
    let mut x = DVector::zeros(p);
    let mut passive = vec![false; p];
    let tolerance = 64.0 * f64::EPSILON * rhs.norm().max(1.0);
    for _ in 0..p * p * 4 {
        let gradient = &rhs - &gram * &x;
        let next = (0..p)
            .filter(|j| !passive[*j] && gradient[*j] > tolerance)
            .max_by(|a, b| gradient[*a].total_cmp(&gradient[*b]));
        let Some(next) = next else {
            break;
        };
        passive[next] = true;
        for _ in 0..p + 1 {
            let active: Vec<_> = (0..p).filter(|j| passive[*j]).collect();
            let sub = DMatrix::from_fn(active.len(), active.len(), |j, k| {
                gram[(active[j], active[k])]
            });
            let solved = sub
                .cholesky()?
                .solve(&DVector::from_fn(active.len(), |j, _| rhs[active[j]]));
            let mut z = DVector::zeros(p);
            for (j, column) in active.iter().copied().enumerate() {
                z[column] = solved[j];
            }
            if active.iter().all(|j| z[*j] > 0.0) {
                x = z;
                break;
            }
            let alpha = active
                .iter()
                .filter(|j| z[**j] <= 0.0)
                .map(|j| x[*j] / (x[*j] - z[*j]))
                .fold(1.0, f64::min);
            x += alpha * (z - &x);
            for j in active {
                if x[j] <= tolerance {
                    x[j] = 0.0;
                    passive[j] = false;
                }
            }
        }
    }
    let result: Vec<_> = x.iter().zip(norms).map(|(x, n)| x.max(0.0) / n).collect();
    result.iter().all(|v| v.is_finite()).then_some(result)
}

fn fit_model(data: &Data, model: Model) -> Option<Fit> {
    let mut columns: [Vec<usize>; 2] = std::array::from_fn(|_| Vec::new());
    let mut p = 0;
    let mut degrees = 0;
    for (direction, shape) in model.shapes.into_iter().enumerate() {
        let n = match shape {
            None => 0,
            Some(Shape::Basic) => 2,
            Some(Shape::Stribeck(_)) => 3,
        };
        columns[direction] = (p..p + n).collect();
        p += n;
        degrees += n + usize::from(matches!(shape, Some(Shape::Stribeck(_))));
    }
    let inertia_column = model.inertia.then_some(p);
    p += usize::from(model.inertia);
    degrees += usize::from(model.inertia);
    let statistics = std::array::from_fn(|direction| {
        model.shapes[direction].map(|shape| data.statistics(direction, shape))
    });
    let mut fit = Fit {
        model,
        statistics,
        columns,
        inertia_column,
        coefficients: vec![0.0; p],
        score: 0.0,
    };
    let (gram, rhs) = fit.normal_equations(false);
    fit.coefficients = nnls(gram, rhs)?;
    let weights: Vec<_> = data.rows.iter().map(|r| r.weight).collect();
    let n = data.effective_count;
    if n <= degrees as f64 + 1.0 {
        return None;
    }
    let variance = fit.loss(data) / weights.iter().sum::<f64>();
    // Numerical precision is the only noise floor. It also makes exact Basic
    // data prefer the simpler model instead of roundoff-sized extra curvature.
    fit.score = n * variance.max(f64::EPSILON).ln() + degrees as f64 * n.ln();
    // The decay scale and common inertia must also be distinguishable from
    // one another, not just from the linear friction coefficients.
    if (model.inertia
        || model
            .shapes
            .iter()
            .any(|s| matches!(s, Some(Shape::Stribeck(_)))))
        && !jointly_observable(&fit)
    {
        return None;
    }
    Some(fit)
}

impl Fit {
    /// Assemble complete per-direction statistics into the shared model.
    /// The optional decay derivatives use the same real rows as the linear fit.
    fn normal_equations(&self, with_decay: bool) -> (DMatrix<f64>, DVector<f64>) {
        let nonlinear = if with_decay {
            self.model
                .shapes
                .iter()
                .filter(|s| matches!(s, Some(Shape::Stribeck(_))))
                .count()
        } else {
            0
        };
        let p = self.coefficients.len();
        let mut gram = DMatrix::zeros(p + nonlinear, p + nonlinear);
        let mut rhs = DVector::zeros(p + nonlinear);
        let mut decay_column = p;
        for direction in 0..2 {
            let Some(statistics) = &self.statistics[direction] else {
                continue;
            };
            let columns = &self.columns[direction];
            let mut mapping: Vec<_> = columns
                .iter()
                .copied()
                .enumerate()
                .map(|(local, global)| (local, global, 1.0))
                .collect();
            if let Some(global) = self.inertia_column {
                mapping.push((columns.len(), global, 1.0));
            }
            if with_decay && matches!(self.model.shapes[direction], Some(Shape::Stribeck(_))) {
                let difference = self.coefficients[columns[1]] - self.coefficients[columns[0]];
                mapping.push((columns.len() + 1, decay_column, difference));
                decay_column += 1;
            }
            for &(local, global, scale) in &mapping {
                rhs[global] += statistics.rhs[local] * scale;
                for &(other_local, other_global, other_scale) in &mapping {
                    gram[(global, other_global)] +=
                        statistics.gram[(local, other_local)] * scale * other_scale;
                }
            }
        }
        (gram, rhs)
    }

    fn prediction(&self, row: &Row) -> f64 {
        let Some(shape) = self.model.shapes[row.direction] else {
            return 0.0;
        };
        let (values, _) = features(row, shape);
        let friction: f64 = self.columns[row.direction]
            .iter()
            .enumerate()
            .map(|(local, column)| values[local] * self.coefficients[*column])
            .sum();
        friction
            + self
                .inertia_column
                .map_or(0.0, |j| self.coefficients[j] * row.acceleration)
    }

    fn loss(&self, data: &Data) -> f64 {
        // Sum actual residuals, avoiding cancellation in y'y - 2x'X'y + x'Gx.
        data.rows
            .iter()
            .map(|row| row.weight * (self.prediction(row) - row.effort).powi(2))
            .sum()
    }
}

fn jointly_observable(fit: &Fit) -> bool {
    normalize_normal(fit.normal_equations(true).0).is_some()
}

fn direction_observable(_data: &Data, fit: &Fit, direction: usize) -> bool {
    let Some(statistics) = &fit.statistics[direction] else {
        return false;
    };
    let columns = &fit.columns[direction];
    let mut mapping: Vec<_> = (0..columns.len()).map(|j| (j, 1.0)).collect();
    if matches!(fit.model.shapes[direction], Some(Shape::Stribeck(_))) {
        mapping.push((
            columns.len() + 1,
            fit.coefficients[columns[1]] - fit.coefficients[columns[0]],
        ));
    }
    let gram = DMatrix::from_fn(mapping.len(), mapping.len(), |j, k| {
        statistics.gram[(mapping[j].0, mapping[k].0)] * mapping[j].1 * mapping[k].1
    });
    normalize_normal(gram).is_some()
}

fn unscale(coefficient: f64, unit: f64, divisor: f64) -> f64 {
    let product = coefficient * unit;
    if product.is_finite() {
        product / divisor
    } else {
        (coefficient / divisor) * unit
    }
}

fn direction_fit(data: &Data, fit: &Fit, direction: usize) -> Option<DirectionFit> {
    let shape = fit.model.shapes[direction]?;
    if !direction_observable(data, fit, direction) {
        return None;
    }
    let columns = &fit.columns[direction];
    let (static_column, viscous_column, vs) = match shape {
        Shape::Basic => (columns[0], columns[1], data.velocity_scale),
        Shape::Stribeck(vs) => (columns[1], columns[2], vs),
    };
    let curve = FrictionCurve {
        coulomb: fit.coefficients[columns[0]] * data.effort_unit,
        static_friction: fit.coefficients[static_column] * data.effort_unit,
        viscous: unscale(
            fit.coefficients[viscous_column],
            data.effort_unit,
            data.velocity_unit,
        ),
        stribeck_velocity: vs,
    };
    if [curve.coulomb, curve.static_friction, curve.viscous, vs]
        .iter()
        .any(|v| !v.is_finite())
    {
        return None;
    }
    let predict = |row: &Row| {
        curve.evaluate(row.sample.velocity, data.velocity_scale) / data.effort_unit
            + fit
                .inertia_column
                .map_or(0.0, |j| fit.coefficients[j] * row.acceleration)
    };
    let mut indices: Vec<_> = data
        .rows
        .iter()
        .enumerate()
        .filter(|(_, r)| r.direction == direction)
        .map(|(i, _)| i)
        .collect();
    indices.sort_by(|a, b| {
        data.rows[*a]
            .sample
            .velocity
            .total_cmp(&data.rows[*b].sample.velocity)
    });
    let mut sum_weight = 0.0;
    let mut loss = 0.0;
    for i in &indices {
        let row = &data.rows[*i];
        sum_weight += row.weight;
        loss += row.weight * (predict(row) - row.effort).powi(2);
    }
    let rmse = (loss / sum_weight).sqrt() * data.effort_unit;
    if !rmse.is_finite() {
        return None;
    }
    let mut points = Vec::new();
    let mut start = 0;
    while start < indices.len() {
        let mut end = (start + indices.len().div_ceil(24)).min(indices.len());
        // Repeated measured speeds belong in the same bin.
        while end < indices.len()
            && data.rows[indices[end]].sample.velocity
                == data.rows[indices[end - 1]].sample.velocity
        {
            end += 1;
        }
        let (mut velocity, mut effort, mut predicted, mut weight) = (0.0, 0.0, 0.0, 0.0);
        for i in &indices[start..end] {
            let row = &data.rows[*i];
            let prediction = predict(row) * data.effort_unit;
            if !prediction.is_finite() {
                return None;
            }
            if row.weight == 0.0 {
                continue;
            }
            weight += row.weight;
            let fraction = row.weight / weight;
            velocity = velocity * (1.0 - fraction) + row.sample.velocity * fraction;
            effort = effort * (1.0 - fraction) + row.sample.effort * fraction;
            predicted = predicted * (1.0 - fraction) + prediction * fraction;
        }
        if weight > 0.0 {
            if [velocity, effort, predicted].iter().any(|v| !v.is_finite()) {
                return None;
            }
            points.push(FitPoint {
                velocity,
                effort,
                predicted,
                samples: end - start,
            });
        }
        start = end;
    }
    Some(DirectionFit {
        curve,
        samples: indices.len(),
        velocity_min: data.rows[*indices.first()?].sample.velocity,
        velocity_max: data.rows[*indices.last()?].sample.velocity,
        rmse,
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCALE: f64 = 0.017;

    fn curves() -> [FrictionCurve; 2] {
        [
            FrictionCurve {
                coulomb: 0.19,
                static_friction: 0.51,
                viscous: 0.27,
                stribeck_velocity: 0.045,
            },
            FrictionCurve {
                coulomb: 0.32,
                static_friction: 0.62,
                viscous: 0.13,
                stribeck_velocity: 0.075,
            },
        ]
    }

    fn synthetic(curves: [FrictionCurve; 2], inertia: f64, noise: f64) -> Vec<Sample> {
        let mut samples = Vec::new();
        for (direction, curve) in curves.into_iter().enumerate() {
            for i in 0..16 {
                let velocity = 0.003
                    * (0.5_f64 / 0.003).powf(i as f64 / 15.0)
                    * if direction == 0 { 1.0 } else { -1.0 };
                for (j, acceleration) in [-0.7, -0.2, 0.0, 0.3, 0.8].into_iter().enumerate() {
                    // Separate analytic reference, including runtime smoothing.
                    let ratio = velocity.abs() / curve.stribeck_velocity;
                    let friction = (curve.coulomb
                        + (curve.static_friction - curve.coulomb) * (-ratio * ratio).exp())
                        * (velocity / SCALE).tanh()
                        + curve.viscous * velocity;
                    samples.push(Sample {
                        velocity,
                        acceleration,
                        effort: friction
                            + inertia * acceleration
                            + noise * ((i * 17 + j * 7 + direction * 3) as f64).sin(),
                        weight: 0.5 + 0.1 * j as f64,
                    });
                }
            }
        }
        samples
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            actual.is_finite() && (actual - expected).abs() <= tolerance,
            "expected {expected}, got {actual}, tolerance {tolerance}"
        );
    }

    #[test]
    fn recovers_asymmetric_multispeed_curves_and_shared_inertia_with_noise() {
        let samples = synthetic(curves(), 0.08, 0.0008);
        let fit = identify(5, &samples, SCALE, 2.0);
        assert_eq!(fit.axis, 5);
        assert_eq!(fit.source, "feedback_effort_residual");
        assert_eq!(fit.samples, samples.len());
        assert_eq!(fit.max_friction, 2.0);
        close(fit.inertia, 0.08, 0.001);
        for (actual, expected) in [
            fit.positive.as_ref().unwrap(),
            fit.negative.as_ref().unwrap(),
        ]
        .into_iter()
        .zip(curves())
        {
            close(actual.curve.coulomb, expected.coulomb, 0.006);
            close(
                actual.curve.static_friction,
                expected.static_friction,
                0.008,
            );
            close(actual.curve.viscous, expected.viscous, 0.012);
            close(
                actual.curve.stribeck_velocity,
                expected.stribeck_velocity,
                0.002,
            );
            assert_eq!(actual.samples, samples.len() / 2);
            assert!(actual.rmse < 0.001);
            assert!(actual.velocity_min < actual.velocity_max);
            assert_eq!(
                actual.points.iter().map(|p| p.samples).sum::<usize>(),
                actual.samples
            );
        }
        let negative = fit.negative.as_ref().unwrap();
        close(negative.velocity_min, -0.5, 1e-15);
        close(negative.velocity_max, -0.003, 1e-15);
        assert!(
            negative
                .points
                .iter()
                .all(|p| p.velocity < 0.0 && p.effort < 0.0)
        );
        // Bins contain real observations, not synthetic torque at a bin center.
        for point in &fit.positive.as_ref().unwrap().points {
            let rows: Vec<_> = samples
                .iter()
                .filter(|s| (s.velocity - point.velocity).abs() < 1e-15)
                .collect();
            assert_eq!(point.samples, rows.len());
            let weight: f64 = rows.iter().map(|s| s.weight).sum();
            let effort: f64 = rows.iter().map(|s| s.effort * s.weight).sum::<f64>() / weight;
            close(point.effort, effort, 1e-14);
            let predicted = rows
                .iter()
                .map(|s| {
                    (fit.positive
                        .as_ref()
                        .unwrap()
                        .curve
                        .evaluate(s.velocity, SCALE)
                        + fit.inertia * s.acceleration)
                        * s.weight
                })
                .sum::<f64>()
                / weight;
            close(point.predicted, predicted, 1e-12);
        }
        serde_json::from_str::<Identification>(&serde_json::to_string(&fit).unwrap()).unwrap();
    }

    #[test]
    fn basic_friction_and_missing_direction_use_observable_fallback() {
        let curve = FrictionCurve {
            coulomb: 0.3,
            static_friction: 0.3,
            viscous: 0.2,
            stribeck_velocity: 0.04,
        };
        let mut samples = synthetic([curve; 2], 0.0, 0.0);
        samples.retain(|s| s.velocity > 0.0);
        let fit = identify(0, &samples, SCALE, 1.0);
        assert!(fit.negative.is_none());
        close(fit.inertia, 0.0, 1e-12);
        let positive = fit.positive.unwrap();
        close(positive.curve.coulomb, curve.coulomb, 1e-10);
        close(positive.curve.viscous, curve.viscous, 1e-10);
        assert_eq!(
            positive.curve.coulomb.to_bits(),
            positive.curve.static_friction.to_bits()
        );
        assert_eq!(positive.curve.stribeck_velocity, SCALE);
        assert!(positive.rmse < 1e-12);
    }

    #[test]
    fn collinear_acceleration_cannot_steal_viscous_friction() {
        for bidirectional in [false, true] {
            let samples: Vec<_> = (0..1_000)
                .map(|i| {
                    let sign = if bidirectional && i >= 500 { -1.0 } else { 1.0 };
                    let velocity = sign * (0.003 + 0.497 * (i % 500) as f64 / 499.0);
                    Sample {
                        velocity,
                        acceleration: velocity,
                        effort: 0.3 * (velocity / SCALE).tanh() + 0.2 * velocity,
                        weight: 1.0,
                    }
                })
                .collect();
            let fit = identify(0, &samples, SCALE, 0.02);
            assert_eq!(
                fit.inertia, 0.0,
                "a=v makes inertia and viscous friction collinear"
            );
            assert!(fit.positive.is_some());
            assert_eq!(fit.negative.is_some(), bidirectional);
            for report in [fit.positive, fit.negative].into_iter().flatten() {
                close(report.curve.coulomb, 0.3, 1e-10);
                close(report.curve.static_friction, 0.3, 1e-10);
                close(report.curve.viscous, 0.2, 1e-10);
                assert!(report.rmse < 1e-12);
            }
        }
    }

    #[test]
    fn increasing_friction_with_fs_below_fc_is_identifiable() {
        let mut known = curves();
        known[0].static_friction = 0.07;
        known[1].static_friction = 0.11;
        let fit = identify(1, &synthetic(known, 0.03, 0.0), SCALE, 2.0);
        close(fit.inertia, 0.03, 1e-6);
        for (fit, expected) in [fit.positive.unwrap(), fit.negative.unwrap()]
            .into_iter()
            .zip(known)
        {
            close(fit.curve.coulomb, expected.coulomb, 1e-5);
            close(fit.curve.static_friction, expected.static_friction, 1e-5);
            close(
                fit.curve.stribeck_velocity,
                expected.stribeck_velocity,
                1e-5,
            );
        }
    }

    #[test]
    fn compensation_budget_is_only_metadata_and_runtime_output_limit() {
        use crate::tuning::{JointTuning, ServoTuning, StribeckTuning};

        let samples = synthetic(curves(), 0.06, 0.0001);
        let peak = samples.iter().map(|s| s.effort.abs()).fold(0.0, f64::max);
        let reference = identify(2, &samples, SCALE, peak);
        assert!(reference.positive.is_some() && reference.negative.is_some());
        let mut reference_json = serde_json::to_value(&reference).unwrap();
        reference_json
            .as_object_mut()
            .unwrap()
            .remove("max_friction");
        for budget in [0.0, 0.02, 0.2, peak, f64::MAX] {
            let fit = identify(2, &samples, SCALE, budget);
            assert_eq!(fit.max_friction, budget);
            assert!(
                fit.positive.is_some() && fit.negative.is_some(),
                "budget {budget}"
            );
            let mut json = serde_json::to_value(&fit).unwrap();
            json.as_object_mut().unwrap().remove("max_friction");
            assert_eq!(
                json, reference_json,
                "budget {budget} changed the measured model"
            );
            let positive = fit.positive.unwrap();
            let negative = fit.negative.unwrap();
            let tuning = ServoTuning {
                friction_enabled: true,
                joints: [JointTuning {
                    velocity_scale: SCALE,
                    max_friction: budget,
                    stribeck: StribeckTuning {
                        enabled: true,
                        positive: positive.curve,
                        negative: negative.curve,
                    },
                    ..JointTuning::default()
                }; 7],
            };
            tuning.validate().unwrap();
            for velocity in [-0.1, 0.1] {
                let curve = if velocity > 0.0 {
                    positive.curve
                } else {
                    negative.curve
                };
                let requested = tuning
                    .apply([velocity; 7], [0.0; 7])
                    .unwrap()
                    .friction_requested[0];
                close(
                    requested,
                    curve.evaluate(velocity, SCALE).clamp(-budget, budget),
                    1e-14,
                );
            }
            if budget < 0.2 {
                assert!(negative.points.iter().any(|p| p.predicted.abs() > budget));
            }
        }
    }

    #[test]
    fn insufficient_speed_excitation_never_recommends_a_curve() {
        for speeds in [
            vec![0.1; 20],
            (0..20).map(|i| 0.2 + i as f64 * 1e-10).collect(),
            (1..21).map(|i| i as f64 * 1e-11).collect(),
        ] {
            let samples: Vec<_> = speeds
                .into_iter()
                .enumerate()
                .map(|(i, velocity)| Sample {
                    velocity,
                    acceleration: (i as f64).sin(),
                    effort: 0.2 + (i as f64).sin() * 0.01,
                    weight: 1.0,
                })
                .collect();
            let fit = identify(0, &samples, SCALE, 1.0);
            assert!(fit.positive.is_none() && fit.negative.is_none());
            assert_eq!(fit.inertia, 0.0);
        }
    }

    #[test]
    fn zero_measured_residual_stays_zero_with_zero_compensation_budget() {
        let mut samples = synthetic(curves(), 0.0, 0.0);
        for sample in &mut samples {
            sample.effort = 0.0;
        }
        let fit = identify(0, &samples, SCALE, 0.0);
        assert_eq!(fit.inertia, 0.0);
        for report in [fit.positive.unwrap(), fit.negative.unwrap()] {
            assert_eq!(report.curve.coulomb, 0.0);
            assert_eq!(report.curve.static_friction, 0.0);
            assert_eq!(report.curve.viscous, 0.0);
            assert_eq!(report.rmse, 0.0);
            assert!(
                report
                    .points
                    .iter()
                    .all(|p| p.effort == 0.0 && p.predicted == 0.0)
            );
        }
    }

    #[test]
    fn unobserved_stribeck_decay_does_not_create_an_extrapolated_peak() {
        let samples: Vec<_> = (0..40)
            .map(|i| {
                let velocity = 0.1 + 0.4 * i as f64 / 39.0;
                Sample {
                    velocity,
                    acceleration: 0.0,
                    effort: 0.3 * (velocity / SCALE).tanh()
                        + 0.2 * velocity
                        + 1e-6 * (i as f64).sin(),
                    weight: 1.0,
                }
            })
            .collect();
        let fit = identify(0, &samples, SCALE, 0.5).positive.unwrap();
        assert_eq!(fit.curve.coulomb, fit.curve.static_friction);
        close(fit.curve.coulomb, 0.3, 1e-5);
    }

    #[test]
    fn invalid_empty_and_legacy_results_are_finite_and_serializable() {
        let empty = identify(6, &[], SCALE, 0.7);
        assert_eq!(empty.samples, 0);
        assert_eq!(empty.max_friction, 0.7);
        assert!(empty.positive.is_none() && empty.negative.is_none());
        let mut json = serde_json::to_value(&empty).unwrap();
        json.as_object_mut().unwrap().remove("max_friction");
        let legacy: Identification = serde_json::from_value(json).unwrap();
        assert_eq!(legacy.max_friction, 0.0);
        let bad: Vec<_> = [0.0, -0.0, f64::NAN, f64::INFINITY]
            .into_iter()
            .map(|velocity| Sample {
                velocity,
                acceleration: 0.0,
                effort: 0.1,
                weight: 1.0,
            })
            .chain([
                Sample {
                    velocity: 0.1,
                    acceleration: f64::NAN,
                    effort: 0.2,
                    weight: 1.0,
                },
                Sample {
                    velocity: 0.1,
                    acceleration: 0.0,
                    effort: f64::INFINITY,
                    weight: 1.0,
                },
                Sample {
                    velocity: 0.1,
                    acceleration: 0.0,
                    effort: 0.2,
                    weight: -1.0,
                },
                Sample {
                    velocity: 0.1,
                    acceleration: 0.0,
                    effort: 0.2,
                    weight: 0.0,
                },
            ])
            .collect();
        assert_eq!(identify(0, &bad, SCALE, 1.0).samples, 0);
        for (scale, budget) in [
            (0.0, 1.0),
            (f64::NAN, 1.0),
            (SCALE, f64::INFINITY),
            (SCALE, -1.0),
        ] {
            let fit = identify(0, &bad, scale, budget);
            assert!(fit.inertia.is_finite() && fit.max_friction.is_finite());
            serde_json::to_string(&fit).unwrap();
        }
    }

    #[test]
    fn ten_thousand_noisy_samples_report_accuracy_and_elapsed_time() {
        let known = curves();
        let samples: Vec<_> = (0..10_000)
            .map(|i| {
                let direction = usize::from(i >= 5_000);
                let speed = 0.003 * (0.5_f64 / 0.003).powf((i % 5_000) as f64 / 4_999.0);
                let velocity = speed * if direction == 0 { 1.0 } else { -1.0 };
                let acceleration = 0.6 * (i as f64 * 0.137).sin() + 0.2 * (i as f64 * 0.023).cos();
                Sample {
                    velocity,
                    acceleration,
                    effort: known[direction].evaluate(velocity, SCALE)
                        + 0.08 * acceleration
                        + 0.002 * (i as f64 * 1.73).sin()
                        + 0.001 * (velocity * 11.0).sin(),
                    weight: 0.6 + 0.3 * (i as f64 * 0.031).cos().abs(),
                }
            })
            .collect();
        let auto_budget = samples.iter().map(|s| s.effort.abs()).fold(0.0, f64::max);
        for budget in [auto_budget, 0.27] {
            let started = std::time::Instant::now();
            let fit = identify(0, &samples, SCALE, budget);
            eprintln!(
                "friction identify: {} samples, budget {budget:.4}, elapsed {:?}, debug={}",
                samples.len(),
                started.elapsed(),
                cfg!(debug_assertions)
            );
            assert_eq!(fit.samples, samples.len());
            assert_eq!(fit.max_friction, budget);
            assert!(fit.inertia.is_finite());
            if budget == auto_budget {
                close(fit.inertia, 0.08, 0.003);
                for (report, expected) in [
                    fit.positive.as_ref().unwrap(),
                    fit.negative.as_ref().unwrap(),
                ]
                .into_iter()
                .zip(known)
                {
                    close(report.curve.coulomb, expected.coulomb, 0.012);
                    close(report.curve.static_friction, expected.static_friction, 0.02);
                    close(report.curve.viscous, expected.viscous, 0.025);
                    close(
                        report.curve.stribeck_velocity,
                        expected.stribeck_velocity,
                        0.004,
                    );
                    assert_eq!(report.samples, 5_000);
                    assert_eq!(
                        report.points.iter().map(|p| p.samples).sum::<usize>(),
                        5_000
                    );
                    assert!(report.rmse < 0.003);
                }
            }
            serde_json::to_string(&fit).unwrap();
        }
    }

    #[test]
    fn extreme_finite_si_values_never_leak_nonfinite_json_numbers() {
        let speeds = [
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            1e-150,
            1.0,
            1e150,
            f64::MAX,
        ];
        let samples: Vec<_> = speeds
            .into_iter()
            .enumerate()
            .flat_map(|(i, speed)| {
                [-1.0, 1.0].into_iter().map(move |sign| Sample {
                    velocity: sign * speed,
                    acceleration: if i % 2 == 0 { sign * f64::MAX } else { 0.0 },
                    effort: sign * f64::MAX,
                    weight: if i % 2 == 0 { f64::MIN_POSITIVE } else { 1.0 },
                })
            })
            .collect();
        for scale in [f64::from_bits(1), SCALE, f64::MAX] {
            let fit = identify(0, &samples, scale, f64::MAX);
            assert!(fit.inertia.is_finite());
            let mut json = serde_json::to_value(&fit).unwrap();
            json.as_object_mut().unwrap().remove("positive");
            json.as_object_mut().unwrap().remove("negative");
            assert!(!json.to_string().contains("null"));
            for report in [fit.positive, fit.negative].into_iter().flatten() {
                let json = serde_json::to_value(report).unwrap();
                assert!(!json.to_string().contains("null"));
            }
        }
    }
}
