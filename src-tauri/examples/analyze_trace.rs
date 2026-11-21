//! Offline analysis of exported TraceRecording JSON; no controller or serial I/O.
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, Write},
};

type Joints = [f64; 7];
const FRESH_US: u64 = 200_000;

#[derive(Clone, Deserialize, Serialize)]
struct ServoTuning {
    friction_enabled: bool,
    joints: [JointTuning; 7],
}
#[derive(Clone, Copy, Deserialize, Serialize)]
struct JointTuning {
    kd: f64,
    velocity_ff: f64,
    coulomb: f64,
    viscous: f64,
    velocity_scale: f64,
    max_friction: f64,
}
#[derive(Deserialize)]
struct ServoSample {
    revision: u64,
    dq_feedforward: Joints,
    tau_gravity: Joints,
    friction_requested: Joints,
    friction_applied: Joints,
    clamped: [bool; 7],
}
#[derive(Default, Serialize)]
struct TuningSummary {
    config: Option<ServoTuning>,
    control_samples: u64,
    transmit_samples_by_joint: [u64; 7],
    clamped_samples_by_joint: [u64; 7],
}

#[derive(Deserialize)]
struct Recording {
    trace: Capture,
    samples: Vec<Sample>,
}
#[derive(Deserialize, Serialize)]
struct Capture {
    id: String,
    started_at_ms: i64,
    duration_ms: u64,
    side: String,
    port: Option<String>,
    status: String,
    accepted: u64,
    written: u64,
    dropped: u64,
    errors: u64,
    pending: u64,
}
#[derive(Deserialize)]
struct Sample {
    at_us: u64,
    #[serde(flatten)]
    data: Source,
}
#[derive(Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
#[allow(
    clippy::large_enum_variant,
    reason = "Offline decoding consumes fixed seven-joint arrays once; no hot-path copies."
)]
enum Source {
    Control {
        #[serde(default)]
        servo_tuning: Option<ServoTuning>,
        #[serde(default)]
        servo_tuning_revision: u64,
        enabled: bool,
        q_ref: Joints,
        dq_ref: Joints,
        ddq_ref: Joints,
        jerk_ref: Joints,
        q: [Option<f64>; 7],
        dq: [Option<f64>; 7],
        age_us: [Option<u64>; 7],
    },
    Transmit {
        seq: u64,
        #[serde(default)]
        tuning: Option<ServoSample>,
        mode: [u8; 7],
        q_ref: Joints,
        dq_ref: Joints,
        tau_ff: Joints,
        q_wire: Joints,
        dq_wire: Joints,
        tau_wire: Joints,
        write_us: u64,
    },
    Feedback {
        joint: usize,
    },
}

fn parse(bytes: &[u8]) -> Result<Recording, String> {
    // Parse all numeric fields, including unused export fields: JSON NaN,
    // infinity and out-of-range numbers must fail before subset deserialization.
    let value: Value = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    serde_json::from_value(value).map_err(|e| e.to_string())
}

fn finite(values: impl IntoIterator<Item = f64>) -> Result<(), String> {
    if values.into_iter().all(f64::is_finite) {
        Ok(())
    } else {
        Err("nonfinite numeric value or derived error".into())
    }
}

fn quantiles(mut values: Vec<u64>) -> Value {
    values.sort_unstable();
    let at = |percent: usize| {
        (!values.is_empty()).then(|| values[(values.len() * percent).div_ceil(100) - 1])
    };
    json!({"count": values.len(), "p50": at(50), "p95": at(95), "max": values.last()})
}

fn summarize(mut recording: Recording) -> Result<Value, String> {
    recording.samples.sort_by_key(|s| s.at_us);
    let mut intervals: [Vec<u64>; 9] = std::array::from_fn(|_| Vec::new());
    let mut previous = [None; 9]; // Control, TX, feedback joints 0..6.
    let mut writes = Vec::new();
    let mut peaks = [[0.0_f64; 7]; 3];
    let mut error2 = [[0.0_f64; 7]; 2];
    let mut pairs = [0_u64; 7];
    let mut wire_error = [[0.0_f64; 7]; 3];
    let mut wire_samples = [0_u64; 7];
    let mut tuning_by_revision = BTreeMap::<u64, TuningSummary>::new();
    let mut clamped_samples = [0_u64; 7];
    let (mut tuned_batches, mut legacy_batches) = (0_u64, 0_u64);
    let (mut zero_seq, mut repeated, mut regressions) = (0_usize, 0, 0);
    let (mut previous_seq, mut skipped) = (None, 0_u64);
    for Sample { at_us, data } in recording.samples {
        let stream = match data {
            Source::Control {
                servo_tuning,
                servo_tuning_revision,
                enabled,
                q_ref,
                dq_ref,
                ddq_ref,
                jerk_ref,
                q,
                dq,
                age_us,
            } => {
                finite(
                    [q_ref, dq_ref, ddq_ref, jerk_ref]
                        .into_iter()
                        .flatten()
                        .chain(q.into_iter().flatten())
                        .chain(dq.into_iter().flatten()),
                )?;
                if let Some(config) = servo_tuning {
                    finite(config.joints.iter().flat_map(|j| {
                        [
                            j.kd,
                            j.velocity_ff,
                            j.coulomb,
                            j.viscous,
                            j.velocity_scale,
                            j.max_friction,
                        ]
                    }))?;
                    let summary = tuning_by_revision.entry(servo_tuning_revision).or_default();
                    summary.control_samples += 1;
                    summary.config = Some(config);
                }
                for (peak, reference) in peaks.iter_mut().zip([dq_ref, ddq_ref, jerk_ref]) {
                    for (max, value) in peak.iter_mut().zip(reference) {
                        *max = max.max(value.abs());
                    }
                }
                for (i, count) in pairs.iter_mut().enumerate() {
                    if enabled
                        && age_us[i].is_some_and(|age| age <= FRESH_US)
                        && let (Some(q), Some(dq)) = (q[i], dq[i])
                    {
                        *count += 1;
                        error2[0][i] += (q - q_ref[i]).powi(2);
                        error2[1][i] += (dq - dq_ref[i]).powi(2);
                    }
                }
                0
            }
            Source::Transmit {
                seq,
                tuning,
                mode,
                q_ref,
                dq_ref,
                tau_ff,
                q_wire,
                dq_wire,
                tau_wire,
                write_us,
            } => {
                finite(
                    [q_ref, dq_ref, tau_ff, q_wire, dq_wire, tau_wire]
                        .into_iter()
                        .flatten(),
                )?;
                if let Some(sample) = &tuning {
                    finite(
                        [
                            sample.dq_feedforward,
                            sample.tau_gravity,
                            sample.friction_requested,
                            sample.friction_applied,
                        ]
                        .into_iter()
                        .flatten(),
                    )?;
                }
                let dq_feedforward = tuning
                    .as_ref()
                    .map_or(dq_ref, |sample| sample.dq_feedforward);
                if mode.iter().any(|&m| m > 7) {
                    return Err("transmit mode must be 0..7".into());
                }
                writes.push(write_us);
                // Sequence zero has no control provenance. Disabled heartbeats
                // can also have nonzero sequences: only mode 1 is a servo target.
                if seq == 0 {
                    zero_seq += 1;
                } else {
                    if mode.contains(&1) {
                        if let Some(sample) = &tuning {
                            tuned_batches += 1;
                            let summary = tuning_by_revision.entry(sample.revision).or_default();
                            for (i, mode) in mode.iter().enumerate() {
                                if *mode == 1 {
                                    summary.transmit_samples_by_joint[i] += 1;
                                    let clamped = u64::from(sample.clamped[i]);
                                    summary.clamped_samples_by_joint[i] += clamped;
                                    clamped_samples[i] += clamped;
                                }
                            }
                        } else {
                            legacy_batches += 1;
                        }
                    }
                    for (count, mode) in wire_samples.iter_mut().zip(mode) {
                        *count += u64::from(mode == 1);
                    }
                    if let Some(last) = previous_seq {
                        repeated += usize::from(seq == last);
                        regressions += usize::from(seq < last);
                        skipped = skipped
                            .checked_add(seq.saturating_sub(last).saturating_sub(1))
                            .ok_or("sequence skip count overflow")?;
                    }
                    previous_seq = Some(seq);
                    for (errors, (ideal, wire)) in wire_error.iter_mut().zip(
                        [q_ref, dq_feedforward, tau_ff]
                            .into_iter()
                            .zip([q_wire, dq_wire, tau_wire]),
                    ) {
                        for (i, ((max, ideal), wire)) in
                            errors.iter_mut().zip(ideal).zip(wire).enumerate()
                        {
                            if mode[i] == 1 {
                                *max = max.max((ideal - wire).abs());
                            }
                        }
                    }
                }
                1
            }
            Source::Feedback { joint } => {
                if joint >= 7 {
                    return Err("feedback joint must be 0..6".into());
                }
                joint + 2
            }
        };
        if let Some(last) = previous[stream] {
            intervals[stream].push(at_us - last);
        }
        previous[stream] = Some(at_us);
    }
    finite(
        error2
            .into_iter()
            .flatten()
            .chain(wire_error.into_iter().flatten()),
    )?;
    let control_samples = intervals[0].len() + usize::from(previous[0].is_some());
    let intervals = intervals.map(quantiles);
    let peaks = peaks.map(|row| row.map(|n| (control_samples > 0).then_some(n)));
    let wire: [[Option<f64>; 7]; 3] =
        wire_error.map(|row| std::array::from_fn(|i| (wire_samples[i] > 0).then_some(row[i])));
    let tracking: Vec<_> = pairs
        .iter()
        .enumerate()
        .map(|(i, &n)| {
            json!({
                "joint": i, "pairs": n,
                "q_rad": (n > 0).then(|| (error2[0][i] / n as f64).sqrt()),
                "dq_rad_s": (n > 0).then(|| (error2[1][i] / n as f64).sqrt()),
            })
        })
        .collect();
    Ok(json!({
        "capture": recording.trace, "quantile_method": "nearest_rank",
        "intervals_us": {"control": intervals[0], "transmit": intervals[1],
            "feedback_by_joint": &intervals[2..]},
        "batch_write_us": quantiles(writes),
        "control_reference_abs_peaks": {"samples": control_samples,
            "dq_rad_s": peaks[0], "ddq_rad_s2": peaks[1], "jerk_rad_s3": peaks[2]},
        "tracking_rms": {"weighting": "uniform_per_control_sample",
            "requires_enabled": true, "requires_q_and_dq": true,
            "per_joint_age_limit_us": FRESH_US, "by_joint": tracking},
        "ideal_wire_max_abs_error": {"samples_by_joint": wire_samples,
            "exclude_seq_zero": true, "required_mode": 1,
            "q_rad": wire[0], "dq_rad_s": wire[1], "tau_nm": wire[2]},
        "transmit_sequences": {"repeated_nonzero": repeated, "skipped_nonzero": skipped,
            "regressions_nonzero": regressions, "zero_seq_excluded": zero_seq},
        "tuning": {"servo_batches_with_tuning": tuned_batches,
            "servo_batches_without_tuning": legacy_batches,
            "clamped_samples_by_joint": clamped_samples, "by_revision": tuning_by_revision},
    }))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args_os().skip(1);
    let path = args.next().ok_or("usage: analyze_trace <capture.json>")?;
    if args.next().is_some() {
        return Err("usage: analyze_trace <capture.json>".into());
    }
    let bytes = fs::read(&path)
        .map_err(|e| format!("cannot read {}: {e}", std::path::Path::new(&path).display()))?;
    let summary = parse(&bytes)
        .and_then(summarize)
        .map_err(|e| format!("invalid capture: {e}"))?;
    let mut stdout = io::stdout().lock();
    serde_json::to_writer_pretty(&mut stdout, &summary).map_err(|e| e.to_string())?;
    writeln!(stdout).map_err(|e| e.to_string())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("analyze_trace: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recording_value(samples: Vec<Value>) -> Value {
        let written = samples.len();
        json!({"trace": {"id": "offline-fixture", "started_at_ms": 123456789,
            "duration_ms": 1000, "side": "left", "port": "fake-port", "status": "writing",
            "accepted": written + 4, "written": written, "dropped": 2, "errors": 3, "pending": 4},
            "samples": samples})
    }
    fn recording(samples: Vec<Value>) -> Recording {
        parse(&serde_json::to_vec(&recording_value(samples)).unwrap()).unwrap()
    }
    fn control(
        at_us: u64,
        enabled: bool,
        age: Option<u64>,
        q: Option<f64>,
        dq: Option<f64>,
    ) -> Value {
        json!({"at_us": at_us, "source": "control", "enabled": enabled,
            "q_ref": ([1.0; 7]), "dq_ref": ([2.0; 7]), "ddq_ref": ([3.0; 7]), "jerk_ref": ([-4.0; 7]),
            "q": ([q; 7]), "dq": ([dq; 7]), "age_us": ([age; 7])})
    }
    fn transmit(at_us: u64, seq: u64) -> Value {
        json!({"at_us": at_us, "source": "transmit", "seq": seq, "mode": ([1; 7]), "write_us": at_us / 10,
            "q_ref": ([1.0; 7]), "dq_ref": ([2.0; 7]), "tau_ff": ([3.0; 7]),
            "q_wire": ([0.75; 7]), "dq_wire": ([1.5; 7]), "tau_wire": ([2.0; 7])})
    }

    fn profile() -> Value {
        json!({"friction_enabled": false, "joints": vec![json!({"kd": 10.0,
            "velocity_ff": 0.0, "coulomb": 0.0, "viscous": 0.0,
            "velocity_scale": 0.02, "max_friction": 0.5}); 7]})
    }

    fn servo_sample(revision: u64) -> Value {
        json!({"revision": revision, "dq_feedforward": ([0.0; 7]),
            "tau_gravity": ([2.75; 7]), "friction_requested": ([0.5; 7]),
            "friction_applied": ([0.25; 7]), "clamped": ([true; 7])})
    }

    #[test]
    fn zero_feedforward_weight_is_not_a_velocity_encoding_error() {
        let mut c = control(0, true, Some(0), Some(1.0), Some(2.0));
        c["servo_tuning"] = profile();
        c["servo_tuning_revision"] = json!(4);
        let mut tx = transmit(100, 7);
        tx["tuning"] = servo_sample(4);
        tx["dq_wire"] = json!(([0.0; 7]));
        let result = summarize(recording(vec![c, tx])).unwrap();
        assert_eq!(
            result["ideal_wire_max_abs_error"]["dq_rad_s"],
            json!(([0.0; 7]))
        );
        assert_eq!(
            result["control_reference_abs_peaks"]["dq_rad_s"],
            json!(([2.0; 7]))
        );
        assert_eq!(result["tracking_rms"]["by_joint"][0]["dq_rad_s"], 0.0);
        // Torque still compares the final total tau_ff, not gravity alone.
        assert_eq!(
            result["ideal_wire_max_abs_error"]["tau_nm"],
            json!(([1.0; 7]))
        );
        let revision = &result["tuning"]["by_revision"]["4"];
        assert_eq!(revision["config"], profile());
        assert_eq!(revision["control_samples"], 1);
        assert_eq!(revision["clamped_samples_by_joint"], json!(([1; 7])));
    }

    #[test]
    fn tuning_statistics_keep_the_servo_and_sequence_mask_with_legacy_fallback() {
        let mut mixed = transmit(100, 9);
        mixed["tuning"] = servo_sample(5);
        mixed["tuning"]["dq_feedforward"] = json!([0.5, 99.0, 99.0, 1.0, 99.0, 99.0, 99.0]);
        mixed["mode"] = json!([1, 0, 6, 1, 0, 6, 0]);
        let mut unsequenced = transmit(200, 0);
        unsequenced["tuning"] = servo_sample(6);
        let mut disabled = transmit(300, 10);
        disabled["tuning"] = servo_sample(6);
        disabled["mode"] = json!(([0; 7]));
        let mut clear = transmit(400, 11);
        clear["tuning"] = servo_sample(6);
        clear["mode"] = json!(([6; 7]));
        let mut legacy = transmit(500, 12);
        legacy["tuning"] = Value::Null;
        let result =
            summarize(recording(vec![mixed, unsequenced, disabled, clear, legacy])).unwrap();
        let tuning = &result["tuning"];
        assert_eq!(tuning["servo_batches_with_tuning"], 1);
        assert_eq!(tuning["servo_batches_without_tuning"], 1);
        assert_eq!(
            tuning["clamped_samples_by_joint"],
            json!([1, 0, 0, 1, 0, 0, 0])
        );
        assert!(tuning["by_revision"].get("6").is_none());
        assert!(tuning["by_revision"]["5"]["config"].is_null());
        assert_eq!(
            tuning["by_revision"]["5"]["transmit_samples_by_joint"],
            json!([1, 0, 0, 1, 0, 0, 0])
        );
        assert_eq!(
            result["ideal_wire_max_abs_error"]["dq_rad_s"],
            json!([1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        );
        assert_eq!(result["batch_write_us"]["count"], 5);
    }

    #[test]
    fn legacy_defaults_and_invalid_tuning_metadata_are_checked() {
        let capture = recording(vec![control(0, false, None, None, None), transmit(1, 1)]);
        assert!(matches!(
            &capture.samples[0].data,
            Source::Control {
                servo_tuning: None,
                servo_tuning_revision: 0,
                ..
            }
        ));
        assert!(matches!(
            &capture.samples[1].data,
            Source::Transmit { tuning: None, .. }
        ));
        let result = summarize(capture).unwrap();
        assert_eq!(result["tuning"]["by_revision"], json!({}));
        assert_eq!(
            result["ideal_wire_max_abs_error"]["dq_rad_s"],
            json!(([0.5; 7]))
        );
        let decode = |sample| serde_json::from_value::<Sample>(sample);
        let mut tx = transmit(0, 1);
        tx["tuning"] = servo_sample(1);
        for field in [
            "dq_feedforward",
            "tau_gravity",
            "friction_requested",
            "friction_applied",
            "clamped",
        ] {
            let mut invalid = tx.clone();
            invalid["tuning"][field].as_array_mut().unwrap().pop();
            assert!(decode(invalid).is_err(), "{field}");
        }
        let mut invalid = tx.clone();
        invalid["tuning"]["dq_feedforward"][0] = json!("NaN");
        assert!(decode(invalid).is_err());
        let bytes = serde_json::to_string(&recording_value(vec![tx])).unwrap();
        assert!(parse(bytes.as_bytes()).is_ok());
        for number in ["NaN", "Infinity", "1e999"] {
            assert!(parse(bytes.replacen("2.75", number, 1).as_bytes()).is_err());
        }
        let mut c = control(0, false, None, None, None);
        c["servo_tuning"] = profile();
        c["servo_tuning"]["joints"][0]
            .as_object_mut()
            .unwrap()
            .remove("kd");
        assert!(decode(c).is_err());
    }

    #[test]
    fn nearest_rank_quantiles_include_zero_intervals_and_empty_streams() {
        assert_eq!(
            quantiles(vec![]),
            json!({"count": 0, "p50": null, "p95": null, "max": null})
        );
        assert_eq!(
            quantiles(vec![10]),
            json!({"count": 1, "p50": 10, "p95": 10, "max": 10})
        );
        assert_eq!(
            quantiles(vec![100, 0, 20, 10]),
            json!({"count": 4, "p50": 10, "p95": 100, "max": 100})
        );
        assert_eq!(quantiles((1..=20).collect())["p95"], 19);
    }

    #[test]
    fn summary_filters_tracking_per_joint_and_retains_reference_and_timing_data() {
        let mut a = control(1_000, true, Some(FRESH_US), Some(3.0), Some(5.0));
        a["age_us"][1] = json!(FRESH_US + 1);
        a["age_us"][2] = Value::Null;
        a["q"][3] = Value::Null;
        a["dq"][4] = Value::Null;
        let mut disabled_tx = transmit(500, 0);
        disabled_tx["q_wire"] = json!(([1000.0; 7]));
        let result = summarize(recording(vec![
            transmit(4_000, 14),
            control(3_000, true, Some(0), Some(5.0), Some(6.0)),
            a,
            control(0, false, Some(0), Some(100.0), Some(100.0)),
            control(5_000, true, Some(FRESH_US + 1), Some(100.0), Some(100.0)),
            disabled_tx,
            transmit(1_000, 10),
            transmit(2_000, 10),
            json!({"at_us": 1_500, "source": "feedback", "joint": 2}),
            json!({"at_us": 1_500, "source": "feedback", "joint": 2}),
            json!({"at_us": 2_500, "source": "feedback", "joint": 2}),
        ]))
        .unwrap();
        assert_eq!(
            result["capture"],
            json!({"id": "offline-fixture", "started_at_ms": 123456789, "duration_ms": 1000,
                "side": "left", "port": "fake-port", "status": "writing", "accepted": 15,
                "written": 11, "dropped": 2, "errors": 3, "pending": 4})
        );
        assert_eq!(
            result["tracking_rms"]["weighting"],
            "uniform_per_control_sample"
        );
        assert_eq!(
            result["intervals_us"]["control"],
            json!({"count": 3, "p50": 2000, "p95": 2000, "max": 2000})
        );
        assert_eq!(result["intervals_us"]["transmit"]["p50"], 1000);
        assert_eq!(result["intervals_us"]["feedback_by_joint"][2]["p50"], 0);
        assert_eq!(result["batch_write_us"]["max"], 400);
        assert_eq!(
            result["control_reference_abs_peaks"]["jerk_rad_s3"],
            json!(([4.0; 7]))
        );
        let tracking = &result["tracking_rms"]["by_joint"];
        assert_eq!(tracking[0]["pairs"], 2);
        assert_eq!(tracking[0]["q_rad"], 10.0_f64.sqrt());
        assert_eq!(tracking[0]["dq_rad_s"], 12.5_f64.sqrt());
        for joint in 1..=4 {
            assert_eq!(tracking[joint]["pairs"], 1);
        }
        assert_eq!(
            result["ideal_wire_max_abs_error"]["q_rad"],
            json!(([0.25; 7]))
        );
        assert_eq!(
            result["ideal_wire_max_abs_error"]["dq_rad_s"],
            json!(([0.5; 7]))
        );
        assert_eq!(
            result["ideal_wire_max_abs_error"]["tau_nm"],
            json!(([1.0; 7]))
        );
        assert_eq!(
            result["transmit_sequences"],
            json!({"repeated_nonzero": 1,
            "skipped_nonzero": 3, "regressions_nonzero": 0, "zero_seq_excluded": 1})
        );
    }

    #[test]
    fn wire_errors_require_servo_mode_and_provenance_for_each_joint() {
        let mut mixed = transmit(100, 9);
        mixed["mode"] = json!([1, 0, 6, 1, 0, 6, 0]);
        mixed["q_wire"] = json!([0.75, 1000.0, 1000.0, 1.5, 1000.0, 1000.0, 1000.0]);
        let mut disabled = transmit(200, 10);
        disabled["mode"] = json!(([0; 7]));
        disabled["q_wire"] = json!(([1000.0; 7]));
        let mut clear = transmit(300, 11);
        clear["mode"] = json!(([6; 7]));
        clear["q_wire"] = json!(([1000.0; 7]));
        let mut unsequenced = transmit(400, 0);
        unsequenced["q_wire"] = json!(([1000.0; 7]));
        let result = summarize(recording(vec![mixed, disabled, clear, unsequenced])).unwrap();
        let wire = &result["ideal_wire_max_abs_error"];
        assert_eq!(wire["samples_by_joint"], json!([1, 0, 0, 1, 0, 0, 0]));
        assert_eq!(
            wire["q_rad"],
            json!([0.25, null, null, 0.5, null, null, null])
        );
        assert_eq!(
            wire["dq_rad_s"],
            json!([0.5, null, null, 0.5, null, null, null])
        );
        assert_eq!(
            wire["tau_nm"],
            json!([1.0, null, null, 1.0, null, null, null])
        );
        assert_eq!(result["intervals_us"]["transmit"]["count"], 3);
        assert_eq!(result["batch_write_us"]["count"], 4);
    }

    #[test]
    fn missing_fresh_pairs_are_null_and_invalid_data_is_rejected() {
        let result = summarize(recording(vec![control(
            0,
            false,
            Some(0),
            Some(1.0),
            Some(2.0),
        )]))
        .unwrap();
        assert!(result["tracking_rms"]["by_joint"][0]["q_rad"].is_null());
        assert!(result["ideal_wire_max_abs_error"]["q_rad"][0].is_null());
        assert!(parse(b"not JSON").is_err());
        assert!(parse(br#"{"unused":1e999}"#).is_err());
        assert!(parse(br#"{"trace":{},"samples":[]}"#).is_err());
        assert!(
            summarize(recording(vec![
                json!({"at_us":0,"source":"feedback","joint":7})
            ]))
            .is_err()
        );
        let overflow = control(0, true, Some(0), Some(1e308), Some(1e308));
        assert!(summarize(recording(vec![overflow])).is_err());
    }
}
