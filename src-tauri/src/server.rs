use crate::{
    arm::{ApiError, ArmHandle, CommandRequest, ControlRequest},
    journal::{Journal, LogPage, LogPageQuery, LogRecord, TraceSummary},
    protocol,
};
use axum::{
    Json, Router,
    extract::{
        DefaultBodyLimit, Path, Query, State, WebSocketUpgrade,
        rejection::{JsonRejection, QueryRejection},
        ws::{Message, WebSocket},
    },
    http::{HeaderMap, Method, StatusCode, header},
    middleware::{self, Next},
    response::{IntoResponse, Response},
    routing::{get, post},
};
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
    time::Duration,
};
use tokio::{
    net::TcpListener,
    sync::{Semaphore, oneshot},
    task::JoinHandle,
};
use tower_http::cors::CorsLayer;

const ORIGINS: &[&str] = &[
    "tauri://localhost",
    "http://tauri.localhost",
    "https://tauri.localhost",
    "http://127.0.0.1:1420",
    "http://localhost:1420",
];

#[derive(Clone)]
struct ApiState {
    arm: ArmHandle,
    journal: Journal,
    token: Arc<String>,
    sockets: Arc<Semaphore>,
}

pub struct ServerHandle {
    stop: Option<oneshot::Sender<()>>,
    task: JoinHandle<()>,
    address: SocketAddr,
}
impl ServerHandle {
    pub fn base_url(&self) -> String {
        format!("http://{}", self.address)
    }
    pub async fn shutdown(mut self) {
        if let Some(stop) = self.stop.take() {
            let _ = stop.send(());
        }
        if tokio::time::timeout(Duration::from_millis(500), &mut self.task)
            .await
            .is_err()
        {
            self.task.abort();
        }
    }
}

pub async fn launch(
    arm: ArmHandle,
    journal: Journal,
    token: String,
) -> Result<ServerHandle, String> {
    launch_at(
        arm,
        journal,
        token,
        SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 8177),
    )
    .await
}

async fn launch_at(
    arm: ArmHandle,
    journal: Journal,
    token: String,
    address: SocketAddr,
) -> Result<ServerHandle, String> {
    let listener = TcpListener::bind(address)
        .await
        .map_err(|e| format!("本机接口启动失败：{e}"))?;
    let address = listener.local_addr().map_err(|e| e.to_string())?;
    let state = ApiState {
        arm,
        journal: journal.clone(),
        token: Arc::new(token),
        sockets: Arc::new(Semaphore::new(16)),
    };
    let cors = CorsLayer::new()
        .allow_origin(
            ORIGINS
                .iter()
                .map(|origin| origin.parse::<axum::http::HeaderValue>().unwrap())
                .collect::<Vec<_>>(),
        )
        .allow_methods([Method::GET, Method::POST, Method::OPTIONS])
        .allow_headers([header::AUTHORIZATION, header::CONTENT_TYPE])
        .expose_headers([header::CONTENT_DISPOSITION]);
    let router = Router::new()
        .route("/v1/ports", get(ports))
        .route("/v1/state", get(snapshot))
        .route("/v1/control", post(control))
        .route("/v1/commands", post(command))
        .route("/v1/logs", get(logs))
        .route("/v1/logs/page", get(log_page))
        .route("/v1/calibrations", get(calibrations))
        .route("/v1/calibrations/{id}", get(calibration))
        .route("/v1/traces", post(trace_start).get(traces))
        .route("/v1/traces/stop", post(trace_stop))
        .route("/v1/traces/{id}", get(trace_read))
        .route("/v1/events", get(events))
        .with_state(state.clone())
        .layer(DefaultBodyLimit::max(16 * 1024))
        .layer(middleware::from_fn_with_state(state, authorize))
        .layer(cors);
    let (stop, stopped) = oneshot::channel();
    let task = tokio::spawn(async move {
        if let Err(error) = axum::serve(listener, router)
            .with_graceful_shutdown(async {
                let _ = stopped.await;
            })
            .await
        {
            journal.log("error", "server", "serve_failed", &error.to_string(), None);
        }
    });
    Ok(ServerHandle {
        stop: Some(stop),
        task,
        address,
    })
}

fn origin_allowed(headers: &HeaderMap) -> bool {
    headers.get(header::ORIGIN).is_none_or(|origin| {
        origin
            .to_str()
            .is_ok_and(|origin| ORIGINS.contains(&origin))
    })
}
fn valid_token(provided: &str, expected: &str) -> bool {
    provided.len() == expected.len()
        && provided
            .bytes()
            .zip(expected.bytes())
            .fold(0u8, |different, (a, b)| different | (a ^ b))
            == 0
}

async fn authorize(
    State(state): State<ApiState>,
    request: axum::extract::Request,
    next: Next,
) -> Response {
    if !origin_allowed(request.headers()) {
        return ApiError::new("forbidden_origin", "来源不受支持").into_response();
    }
    // Browser WebSockets authenticate in their first frame, never in the URL.
    if request.uri().path() == "/v1/events" {
        return next.run(request).await;
    }
    let token = request
        .headers()
        .get(header::AUTHORIZATION)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.strip_prefix("Bearer "));
    if !token.is_some_and(|token| valid_token(token, state.token.as_str())) {
        return ApiError::new("unauthorized", "认证失败").into_response();
    }
    next.run(request).await
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let status = match self.code.as_str() {
            "unauthorized" => StatusCode::UNAUTHORIZED,
            "forbidden_origin" => StatusCode::FORBIDDEN,
            "queue_full" | "too_many_connections" => StatusCode::TOO_MANY_REQUESTS,
            "closing" | "journal_unavailable" | "trace_unavailable" => {
                StatusCode::SERVICE_UNAVAILABLE
            }
            "timeout" => StatusCode::GATEWAY_TIMEOUT,
            "owned" | "not_owner" | "stale_context" | "motion_busy" | "trace_active" => {
                StatusCode::CONFLICT
            }
            "trace_capacity" => StatusCode::TOO_MANY_REQUESTS,
            "trace_not_found" | "calibration_not_found" => StatusCode::NOT_FOUND,
            _ => StatusCode::BAD_REQUEST,
        };
        (status, Json(self)).into_response()
    }
}

async fn ports() -> Result<Json<Vec<protocol::PortInfo>>, ApiError> {
    tokio::task::spawn_blocking(protocol::enumerate_ports)
        .await
        .map(Json)
        .map_err(|_| ApiError::new("ports_failed", "串口枚举失败"))
}
async fn snapshot(State(state): State<ApiState>) -> Json<crate::arm::Snapshot> {
    Json(state.arm.snapshot())
}
async fn control(
    State(state): State<ApiState>,
    Json(request): Json<ControlRequest>,
) -> Result<Json<crate::arm::ControlReply>, ApiError> {
    state.arm.control(request).await.map(Json)
}
async fn command(
    State(state): State<ApiState>,
    Json(request): Json<CommandRequest>,
) -> Result<(StatusCode, Json<serde_json::Value>), ApiError> {
    let request_id = request.request_id.clone();
    state.arm.command(request)?;
    Ok((
        StatusCode::ACCEPTED,
        Json(serde_json::json!({ "request_id": request_id, "status": "queued" })),
    ))
}
#[derive(Deserialize)]
struct LogQuery {
    limit: Option<u32>,
}
async fn logs(
    State(state): State<ApiState>,
    Query(query): Query<LogQuery>,
) -> Result<Json<Vec<LogRecord>>, ApiError> {
    let limit = query.limit.unwrap_or(50).clamp(1, 500);
    tokio::task::spawn_blocking(move || state.journal.recent(limit))
        .await
        .map_err(|_| ApiError::new("journal_unavailable", "日志查询失败"))?
        .map(Json)
        .map_err(|_| ApiError::new("journal_unavailable", "日志暂不可读"))
}

async fn log_page(
    State(state): State<ApiState>,
    query: Result<Query<LogPageQuery>, QueryRejection>,
) -> Result<Json<LogPage>, ApiError> {
    let Query(query) = query.map_err(|_| ApiError::new("invalid_log_query", "日志查询参数无效"))?;
    tokio::task::spawn_blocking(move || state.journal.page(query))
        .await
        .map_err(|_| ApiError::new("journal_unavailable", "日志查询失败"))?
        .map(Json)
        .map_err(|error| {
            if error == "invalid_log_query" {
                ApiError::new("invalid_log_query", "日志查询参数无效")
            } else {
                ApiError::new("journal_unavailable", "日志暂不可读")
            }
        })
}

#[derive(Default, Deserialize)]
struct CalibrationListQuery {
    limit: Option<u32>,
}

async fn calibrations(
    State(state): State<ApiState>,
    query: Result<Query<CalibrationListQuery>, QueryRejection>,
) -> Result<Response, ApiError> {
    let Query(query) =
        query.map_err(|_| ApiError::new("invalid_calibration", "校准查询参数无效"))?;
    let limit = query.limit.unwrap_or(20).clamp(1, 100) as usize;
    let body = tokio::task::spawn_blocking(move || {
        let reports = state.journal.calibrations(limit)?;
        serde_json::to_vec(&reports).map_err(|e| e.to_string())
    })
    .await
    .map_err(|_| ApiError::new("journal_unavailable", "校准记录读取失败"))?
    .map_err(|_| ApiError::new("journal_unavailable", "校准记录暂不可读"))?;
    Ok((
        [
            (header::CONTENT_TYPE, "application/json"),
            (header::CACHE_CONTROL, "no-store"),
        ],
        body,
    )
        .into_response())
}

#[derive(Default, Deserialize)]
struct CalibrationQuery {
    points: Option<usize>,
    download: Option<bool>,
}

async fn calibration(
    State(state): State<ApiState>,
    Path(id): Path<String>,
    query: Result<Query<CalibrationQuery>, QueryRejection>,
) -> Result<Response, ApiError> {
    let Query(query) =
        query.map_err(|_| ApiError::new("invalid_calibration", "校准查询参数无效"))?;
    tokio::task::spawn_blocking(move || {
        let report = state
            .journal
            .calibration(&id)
            .map_err(|_| ApiError::new("journal_unavailable", "校准记录暂不可读"))?
            .ok_or_else(|| ApiError::new("calibration_not_found", "校准记录不存在"))?;
        let download = query.download.unwrap_or(false);
        let report = if download {
            report
        } else {
            report.for_plot(query.points.unwrap_or(3000))
        };
        let body = serde_json::to_vec(&report)
            .map_err(|_| ApiError::new("journal_unavailable", "校准记录编码失败"))?;
        let mut response = (
            [
                (header::CONTENT_TYPE, "application/json"),
                (header::CACHE_CONTROL, "no-store"),
            ],
            body,
        )
            .into_response();
        if download {
            // Only the suggested filename is normalized; report IDs remain literal SQL values.
            let filename: String = report
                .id
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || matches!(c, '-' | '_') {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            response.headers_mut().insert(
                header::CONTENT_DISPOSITION,
                format!("attachment; filename=\"calibration-{filename}.json\"")
                    .parse()
                    .map_err(|_| ApiError::new("journal_unavailable", "校准附件编码失败"))?,
            );
        }
        Ok(response)
    })
    .await
    .map_err(|_| ApiError::new("journal_unavailable", "校准记录读取失败"))?
}

#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct TraceStartRequest {
    duration_ms: Option<u64>,
}

fn trace_error(code: String) -> ApiError {
    let message = match code.as_str() {
        "invalid_trace_duration" => "采样时长应为 1000–20000 毫秒",
        "invalid_trace_context" => "采样上下文无效",
        "trace_active" => "已有采样正在记录",
        "trace_capacity" => "采样写入积压，请稍后重试",
        "trace_not_found" => "采样不存在或已清理",
        "closing" => "应用正在关闭",
        _ => "采样暂不可读写",
    };
    ApiError::new(&code, message)
}

async fn trace_start(
    State(state): State<ApiState>,
    request: Result<Json<TraceStartRequest>, JsonRejection>,
) -> Result<Json<TraceSummary>, ApiError> {
    let Json(request) =
        request.map_err(|_| ApiError::new("invalid_trace_request", "采样请求 JSON 无效"))?;
    let duration_ms = request
        .duration_ms
        .unwrap_or(crate::trace::DEFAULT_DURATION_MS);
    if !(1000..=20_000).contains(&duration_ms) {
        return Err(trace_error("invalid_trace_duration".into()));
    }
    // Merely observe an existing connection. This never acquires a lease, sends
    // a control/command request, enables motors, or opens a serial device.
    let snapshot = state.arm.snapshot();
    if snapshot.port.is_none()
        || matches!(
            snapshot.phase,
            crate::arm::Phase::Disconnected
                | crate::arm::Phase::Connecting
                | crate::arm::Phase::Closing
        )
    {
        return Err(ApiError::new(
            "not_connected",
            "请先连接机械臂，再开始短时采样",
        ));
    }
    tokio::task::spawn_blocking(move || {
        state
            .journal
            .trace_start(duration_ms, &snapshot.side, snapshot.port.as_deref())
    })
    .await
    .map_err(|_| trace_error("trace_unavailable".into()))?
    .map(Json)
    .map_err(trace_error)
}

#[derive(Default, Deserialize)]
struct TraceListQuery {
    limit: Option<u32>,
}

async fn traces(
    State(state): State<ApiState>,
    query: Result<Query<TraceListQuery>, QueryRejection>,
) -> Result<Json<Vec<TraceSummary>>, ApiError> {
    let Query(query) =
        query.map_err(|_| ApiError::new("invalid_trace_request", "采样查询参数无效"))?;
    let limit = query.limit.unwrap_or(20).clamp(1, 10);
    tokio::task::spawn_blocking(move || state.journal.trace_list(limit))
        .await
        .map_err(|_| trace_error("trace_unavailable".into()))?
        .map(Json)
        .map_err(trace_error)
}

async fn trace_stop(State(state): State<ApiState>) -> Result<Json<Option<TraceSummary>>, ApiError> {
    tokio::task::spawn_blocking(move || state.journal.trace_stop())
        .await
        .map_err(|_| trace_error("trace_unavailable".into()))?
        .map(Json)
        .map_err(trace_error)
}

#[derive(Default, Deserialize)]
struct TraceReadQuery {
    download: Option<bool>,
    attachment: Option<bool>,
}

async fn trace_read(
    State(state): State<ApiState>,
    Path(id): Path<String>,
    query: Result<Query<TraceReadQuery>, QueryRejection>,
) -> Result<Response, ApiError> {
    let Query(query) =
        query.map_err(|_| ApiError::new("invalid_trace_request", "采样查询参数无效"))?;
    let download = query.download.unwrap_or(false) || query.attachment.unwrap_or(false);
    // Encoding a full capture is also blocking CPU work; keep it off Tokio threads.
    let (id, body) = tokio::task::spawn_blocking(move || {
        let recording = state.journal.trace_read(&id)?;
        let body = serde_json::to_vec(&recording).map_err(|_| "trace_unavailable".to_owned())?;
        Ok::<_, String>((recording.trace.id, body))
    })
    .await
    .map_err(|_| trace_error("trace_unavailable".into()))?
    .map_err(trace_error)?;
    let mut response = ([(header::CONTENT_TYPE, "application/json")], body).into_response();
    response
        .headers_mut()
        .insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    if download {
        response.headers_mut().insert(
            header::CONTENT_DISPOSITION,
            format!("attachment; filename=\"trace-{id}.json\"")
                .parse()
                .map_err(|_| trace_error("trace_unavailable".into()))?,
        );
    }
    Ok(response)
}

async fn events(State(state): State<ApiState>, ws: WebSocketUpgrade) -> Result<Response, ApiError> {
    let permit = state
        .sockets
        .clone()
        .try_acquire_owned()
        .map_err(|_| ApiError::new("too_many_connections", "订阅连接已满"))?;
    Ok(ws
        .max_message_size(8 * 1024)
        .max_frame_size(8 * 1024)
        .on_upgrade(move |socket| async move {
            let _permit = permit;
            stream(socket, state).await;
        }))
}

async fn stream(mut socket: WebSocket, state: ApiState) {
    let authenticated = match tokio::time::timeout(Duration::from_secs(2), socket.next()).await {
        Ok(Some(Ok(Message::Text(text)))) => serde_json::from_str::<serde_json::Value>(&text)
            .ok()
            .and_then(|value| {
                value
                    .get("token")
                    .and_then(|token| token.as_str())
                    .map(|token| valid_token(token, state.token.as_str()))
            })
            .unwrap_or(false),
        _ => false,
    };
    if !authenticated {
        let _ = socket.close().await;
        return;
    }
    let mut snapshots = state.arm.subscribe();
    loop {
        // Clone before awaiting the socket: slow subscribers never retain a watch read lock.
        let snapshot = snapshots.borrow_and_update().clone();
        let closing = snapshot.phase == crate::arm::Phase::Closing;
        let Ok(payload) =
            serde_json::to_string(&serde_json::json!({ "type": "state", "data": snapshot }))
        else {
            break;
        };
        if !matches!(
            tokio::time::timeout(
                Duration::from_millis(500),
                socket.send(Message::Text(payload.into()))
            )
            .await,
            Ok(Ok(()))
        ) {
            break;
        }
        if closing {
            break;
        }
        tokio::select! {
            changed = snapshots.changed() => { if changed.is_err() { break; } }
            message = socket.next() => {
                match message { Some(Ok(Message::Ping(bytes))) => { if socket.send(Message::Pong(bytes)).await.is_err() { break; } }, Some(Ok(Message::Pong(_))) => {}, _ => break }
            }
        }
    }
    let _ = tokio::time::timeout(Duration::from_millis(100), socket.close()).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration_report(sample_count: usize) -> crate::calibration::Report {
        crate::calibration::Report {
            id: "history-api".into(),
            request_id: "original-run".into(),
            created_at_ms: 1000,
            side: "right".into(),
            port: "simulation".into(),
            settings: crate::calibration::Settings {
                axis: Some(1),
                excursion: 0.1,
                speed: 0.02,
                max_friction: 0.1,
            },
            center_reference: [0.0; 7],
            center_measured: [0.0; 7],
            tcp: None,
            baseline: crate::tuning::ServoTuning::default(),
            recommended: None,
            trials: vec![],
            choices: vec![],
            sample_count,
            curves: Arc::new(
                (0..sample_count)
                    .map(|i| crate::calibration::Point {
                        t_ms: i as u32 * 20,
                        trial: usize::from(i >= sample_count / 2),
                        axis: 1,
                        reference: [(i % 32) as f64 / 1024.0; 7],
                        command: (i % 3 != 0).then_some([(i % 32) as f64 / 1024.0; 7]),
                        feedback: [((i % 32) as f64 - 1.0) / 1024.0; 7],
                        velocity: [1.0 / 128.0; 7],
                        feedback_velocity: [3.0 / 256.0; 7],
                        feedback_age_ms: [2; 7],
                        command_at_ms: (i % 3 != 0).then_some(i as u32 * 20),
                    })
                    .collect(),
            ),
            outcome: "cancelled".into(),
            improvement: None,
            message: "校准已取消".into(),
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn calibration_history_requires_auth_and_invalid_start_cannot_move_a_device() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            "calibration-api-test".into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let base = server.base_url();
        let client = reqwest::Client::new();
        for path in [
            "/v1/calibrations",
            "/v1/calibrations/missing",
            "/v1/calibrations/history-api?download=true&points=1",
        ] {
            assert_eq!(
                client
                    .get(format!("{base}{path}"))
                    .send()
                    .await
                    .unwrap()
                    .status(),
                StatusCode::UNAUTHORIZED
            );
            assert_eq!(
                client
                    .get(format!("{base}{path}"))
                    .bearer_auth("calibration-api-test")
                    .header(header::ORIGIN, "https://other.invalid")
                    .send()
                    .await
                    .unwrap()
                    .status(),
                StatusCode::FORBIDDEN
            );
        }
        let before = arm.snapshot();
        let control: serde_json::Value = client
            .post(format!("{base}/v1/control"))
            .bearer_auth("calibration-api-test")
            .json(&serde_json::json!({"action":"acquire","client_id":"calibration-api"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let response=client.post(format!("{base}/v1/commands")).bearer_auth("calibration-api-test").json(&serde_json::json!({
            "request_id":"invalid-calibration","client_id":"calibration-api","lease_id":control["lease_id"],"generation":control["generation"],
            "kind":"calibrate_start","settings":{"axis":9,"excursion":0.1,"speed":0.02,"max_friction":0.1}
        })).send().await.unwrap();
        assert_eq!(response.status(), StatusCode::ACCEPTED);
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if arm
                    .snapshot()
                    .results
                    .iter()
                    .any(|r| r.request_id == "invalid-calibration" && r.status == "rejected")
                {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        let report = calibration_report(0);
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if journal.record_calibration(&report) {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            loop {
                if journal
                    .calibration("history-api")
                    .is_ok_and(|r| r.is_some())
                {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .unwrap();
        let list: Vec<crate::calibration::Report> = client
            .get(format!("{base}/v1/calibrations?limit=20"))
            .bearer_auth("calibration-api-test")
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].id, "history-api");
        let detail: crate::calibration::Report = client
            .get(format!("{base}/v1/calibrations/history-api"))
            .bearer_auth("calibration-api-test")
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(detail.request_id, "original-run");
        assert_eq!(
            client
                .get(format!("{base}/v1/calibrations/missing"))
                .bearer_auth("calibration-api-test")
                .send()
                .await
                .unwrap()
                .status(),
            StatusCode::NOT_FOUND
        );
        assert_eq!(
            client
                .get(format!("{base}/v1/calibrations?limit=oops"))
                .bearer_auth("calibration-api-test")
                .send()
                .await
                .unwrap()
                .status(),
            StatusCode::BAD_REQUEST
        );
        let after = arm.snapshot();
        assert_eq!(after.phase, before.phase);
        assert_eq!(after.enabled, before.enabled);
        assert_eq!(after.port, before.port);
        assert!(after.action.is_none());
        assert!(after.calibration.is_none());
        assert!(arm.shutdown(Duration::from_millis(500)));
        server.shutdown().await;
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn calibration_plot_export_and_read_errors_do_not_issue_commands() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("events.db");
        let journal = Journal::start(path.clone()).unwrap();
        let report = calibration_report(6001);
        tokio::time::timeout(Duration::from_secs(8), async {
            while !journal.record_calibration(&report) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            while !journal
                .calibration(&report.id)
                .is_ok_and(|saved| saved.is_some())
            {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("calibration fixture did not commit");
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let token = "calibration-read-only";
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            token.into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let base = server.base_url();
        let client = reqwest::Client::new();
        let before = serde_json::to_value(arm.snapshot()).unwrap();
        let response = client
            .get(format!("{base}/v1/calibrations"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()[header::CACHE_CONTROL], "no-store");
        let summary = response.bytes().await.unwrap();
        let history: Vec<crate::calibration::Report> = serde_json::from_slice(&summary).unwrap();
        assert_eq!(history.len(), 1);
        assert!(history[0].curves.is_empty());
        assert_eq!(history[0].sample_count, report.sample_count);
        assert!(summary.len() * 50 < serde_json::to_vec(&report).unwrap().len());
        for (query, points, download) in [
            ("", 3000, false),
            ("?points=41", 41, false),
            ("?points=0", 0, false),
            ("?points=90001&download=false", 90001, false),
            ("?points=1&download=true", 0, true),
            ("?download=true", 0, true),
        ] {
            let response = client
                .get(format!("{base}/v1/calibrations/{}{query}", report.id))
                .bearer_auth(token)
                .header(header::ORIGIN, "http://localhost:1420")
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::OK, "{query}");
            assert_eq!(response.headers()[header::CONTENT_TYPE], "application/json");
            assert_eq!(response.headers()[header::CACHE_CONTROL], "no-store");
            if download {
                assert_eq!(
                    response.headers()[header::CONTENT_DISPOSITION],
                    "attachment; filename=\"calibration-history-api.json\""
                );
            } else {
                assert!(!response.headers().contains_key(header::CONTENT_DISPOSITION));
            }
            let saved: crate::calibration::Report = response.json().await.unwrap();
            let expected = report.for_plot(points);
            assert_eq!(saved.sample_count, 6001);
            assert_eq!(
                serde_json::to_value(saved.summary()).unwrap(),
                serde_json::to_value(expected.summary()).unwrap(),
                "{query}"
            );
            assert_eq!(saved.curves.len(), expected.curves.len(), "{query}");
            for (i, (saved, expected)) in
                saved.curves.iter().zip(expected.curves.iter()).enumerate()
            {
                assert_eq!(
                    serde_json::to_value(saved).unwrap(),
                    serde_json::to_value(expected).unwrap(),
                    "{query}: curve sample {i}"
                );
            }
        }
        for query in ["?points=oops", "?points=-1", "?download=oops"] {
            let response = client
                .get(format!("{base}/v1/calibrations/{}{query}", report.id))
                .bearer_auth(token)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::BAD_REQUEST);
            assert_eq!(
                response.json::<serde_json::Value>().await.unwrap()["code"],
                "invalid_calibration"
            );
        }
        let response = client
            .get(format!("{base}/v1/calibrations/{}", "x".repeat(101)))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::NOT_FOUND);
        assert_eq!(
            response.json::<serde_json::Value>().await.unwrap()["code"],
            "calibration_not_found"
        );

        let conn = rusqlite::Connection::open(path).unwrap();
        conn.execute(
            "UPDATE calibration_curves SET payloadJSON='broken json'",
            [],
        )
        .unwrap();
        let response = client
            .get(format!("{base}/v1/calibrations"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let history: Vec<crate::calibration::Report> = response.json().await.unwrap();
        assert!(history[0].curves.is_empty());
        assert_eq!(history[0].sample_count, 6001);
        for query in ["", "?download=true"] {
            let response = client
                .get(format!("{base}/v1/calibrations/{}{query}", report.id))
                .bearer_auth(token)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
            assert_eq!(
                response.json::<serde_json::Value>().await.unwrap()["code"],
                "journal_unavailable"
            );
        }
        conn.execute_batch("DROP TABLE calibrations").unwrap();
        for endpoint in ["calibrations", "calibrations/missing"] {
            let response = client
                .get(format!("{base}/v1/{endpoint}"))
                .bearer_auth(token)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
            assert_eq!(
                response.json::<serde_json::Value>().await.unwrap()["code"],
                "journal_unavailable"
            );
        }
        let after = serde_json::to_value(arm.snapshot()).unwrap();
        for key in [
            "generation",
            "phase",
            "side",
            "port",
            "enabled",
            "owner",
            "action",
            "calibration",
            "last_result",
            "results",
        ] {
            assert_eq!(before[key], after[key], "calibration read changed {key}");
        }
        assert!(arm.shutdown(Duration::from_millis(500)));
        server.shutdown().await;
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    use tokio_tungstenite::{connect_async, tungstenite::Message as ClientMessage};

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn log_page_http_auth_query_snapshot_and_legacy_reads_do_not_issue_commands() {
        fn page_url(base: &str, query: &[(&str, &str)]) -> reqwest::Url {
            let mut url = reqwest::Url::parse(&format!("{base}/v1/logs/page")).unwrap();
            url.query_pairs_mut().extend_pairs(query.iter().copied());
            url
        }

        async fn page(
            client: &reqwest::Client,
            base: &str,
            token: &str,
            query: &[(&str, &str)],
        ) -> LogPage {
            let response = client
                .get(page_url(base, query))
                .bearer_auth(token)
                .header(header::ORIGIN, "http://localhost:1420")
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::OK);
            assert_eq!(
                response.headers()[header::ACCESS_CONTROL_ALLOW_ORIGIN],
                "http://localhost:1420"
            );
            response.json().await.unwrap()
        }

        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        // Retry only rejected fixture submissions, preserving production try_lock behavior.
        tokio::time::timeout(Duration::from_secs(8), async {
            for i in 0..123 {
                loop {
                    let dropped = journal.stats().dropped;
                    let message = if i == 0 {
                        "old event 50%_literal 界".into()
                    } else {
                        format!("event {i}")
                    };
                    journal.log("info", "page-fixture", "event", &message, None);
                    if journal.stats().dropped == dropped {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
            }
            while journal.stats().written != 123 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("log fixtures did not commit");
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let token = "log-page-secret";
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            token.into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let base = server.base_url();
        let client = reqwest::Client::new();
        let before = serde_json::to_value(arm.snapshot()).unwrap();
        for path in ["/v1/logs", "/v1/logs/page", "/v1/logs/page?through_id=-1"] {
            for auth in [None, Some("wrong-key")] {
                let mut request = client.get(format!("{base}{path}"));
                if let Some(auth) = auth {
                    request = request.bearer_auth(auth);
                }
                let response = request.send().await.unwrap();
                assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
                assert_eq!(
                    response.json::<serde_json::Value>().await.unwrap()["code"],
                    "unauthorized"
                );
            }
            let response = client
                .get(format!("{base}{path}"))
                .bearer_auth(token)
                .header(header::ORIGIN, "https://unrelated.example")
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::FORBIDDEN);
            assert_eq!(
                response.json::<serde_json::Value>().await.unwrap()["code"],
                "forbidden_origin"
            );
        }
        for (name, value) in [
            ("page", "bad".to_owned()),
            ("page", "-1".to_owned()),
            ("page", "4294967296".to_owned()),
            ("page_size", "bad".to_owned()),
            ("page_size", "-1".to_owned()),
            ("through_id", "-1".to_owned()),
            ("through_id", "bad".to_owned()),
            ("through_id", "9223372036854775808".to_owned()),
            ("search", "x".repeat(513)),
            ("search", "界".repeat(171)),
        ] {
            let response = client
                .get(page_url(&base, &[(name, value.as_str())]))
                .bearer_auth(token)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::BAD_REQUEST, "query {name}");
            let error: serde_json::Value = response.json().await.unwrap();
            assert_eq!(error["code"], "invalid_log_query");
            assert!(error["message"].is_string());
        }
        let first = page(&client, &base, token, &[]).await;
        assert_eq!(
            (first.page, first.page_size, first.items.len()),
            (1, 50, 50)
        );
        assert!(first.total >= 123);
        assert!(first.items.windows(2).all(|r| r[0].id > r[1].id));
        let legacy: serde_json::Value = client
            .get(format!("{base}/v1/logs"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!(legacy.is_array());
        assert_eq!(legacy, serde_json::to_value(&first.items).unwrap());
        for (limit, expected) in [("3", 3), ("0", 1), ("4294967295", first.total as usize)] {
            let legacy: Vec<LogRecord> = client
                .get(format!("{base}/v1/logs?limit={limit}"))
                .bearer_auth(token)
                .send()
                .await
                .unwrap()
                .json()
                .await
                .unwrap();
            assert_eq!(legacy.len(), expected);
        }
        let through = first.through_id.to_string();
        let second = page(
            &client,
            &base,
            token,
            &[
                ("page", "2"),
                ("search", "page-fixture"),
                ("through_id", &through),
            ],
        )
        .await;
        assert_eq!(
            (
                second.page,
                second.total,
                second.total_pages,
                second.items.len()
            ),
            (2, 123, 3, 50)
        );
        let last = page(
            &client,
            &base,
            token,
            &[
                ("page", "4294967295"),
                ("search", "page-fixture"),
                ("through_id", &through),
            ],
        )
        .await;
        assert_eq!((last.page, last.items.len()), (3, 23));
        assert_eq!(
            last.items.last().unwrap().message,
            "old event 50%_literal 界"
        );
        let smallest = page(&client, &base, token, &[("page", "0"), ("page_size", "0")]).await;
        assert_eq!(
            (smallest.page, smallest.page_size, smallest.items.len()),
            (1, 1, 1)
        );
        let largest = page(&client, &base, token, &[("page_size", "4294967295")]).await;
        assert_eq!(
            (largest.page_size, largest.items.len()),
            (500, first.total as usize)
        );
        let literal = page(&client, &base, token, &[("search", "50%_LITERAL")]).await;
        assert_eq!((literal.total, literal.items.len()), (1, 1));
        assert_eq!(literal.items[0].id, 1);
        let boundary = format!("{}xx", "界".repeat(170));
        let empty = page(&client, &base, token, &[("search", &boundary)]).await;
        assert_eq!((empty.page, empty.total, empty.total_pages), (1, 0, 1));
        assert!(empty.items.is_empty());
        let empty = page(&client, &base, token, &[("through_id", "0")]).await;
        assert_eq!(
            (empty.total, empty.total_pages, empty.through_id),
            (0, 1, 0)
        );
        assert!(empty.items.is_empty());

        tokio::time::timeout(Duration::from_secs(8), async {
            loop {
                let dropped = journal.stats().dropped;
                journal.log("info", "page-fixture", "new", "after snapshot", None);
                if journal.stats().dropped == dropped {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            while journal.stats().pending != 0 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("new log did not commit");
        let fixed = page(
            &client,
            &base,
            token,
            &[
                ("page", "2"),
                ("search", "page-fixture"),
                ("through_id", &through),
            ],
        )
        .await;
        assert_eq!(fixed.total, 123);
        assert_eq!(
            serde_json::to_value(&fixed.items).unwrap(),
            serde_json::to_value(&second.items).unwrap()
        );
        let latest = page(
            &client,
            &base,
            token,
            &[
                ("search", "page-fixture"),
                ("through_id", "9223372036854775807"),
            ],
        )
        .await;
        assert_eq!(latest.total, 124);
        assert_eq!(latest.items[0].message, "after snapshot");
        assert!(latest.through_id > first.through_id);
        let after = serde_json::to_value(arm.snapshot()).unwrap();
        for key in [
            "generation",
            "phase",
            "side",
            "port",
            "enabled",
            "owner",
            "action",
            "last_result",
            "results",
        ] {
            assert_eq!(before[key], after[key], "log query changed {key}");
        }
        assert!(arm.shutdown(Duration::from_millis(500)));
        server.shutdown().await;
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn authenticated_http_ws_and_sqlite_share_real_controller_state() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.sqlite3")).unwrap();
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let token = "test-session-key";
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            token.into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let base = format!("http://{}", server.address);
        let client = reqwest::Client::new();
        assert_eq!(
            client
                .get(format!("{base}/v1/state"))
                .send()
                .await
                .unwrap()
                .status(),
            401
        );
        let state: serde_json::Value = client
            .get(format!("{base}/v1/state"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(state["joints"].as_array().unwrap().len(), 7);
        assert!(
            state["joints"]
                .as_array()
                .unwrap()
                .iter()
                .all(|joint| joint["q"].is_null())
        );
        let owner: serde_json::Value = client
            .post(format!("{base}/v1/control"))
            .bearer_auth(token)
            .json(&serde_json::json!({"action":"acquire","client_id":"test-client"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let lease = owner["lease_id"].as_str().unwrap();
        let queued = client.post(format!("{base}/v1/commands")).bearer_auth(token).json(&serde_json::json!({"request_id":"enable-disconnected","client_id":"test-client","lease_id":lease,"generation":owner["generation"],"kind":"enable"})).send().await.unwrap();
        assert_eq!(queued.status(), 202);
        for _ in 0..50 {
            if arm
                .snapshot()
                .last_result
                .as_ref()
                .is_some_and(|r| r.request_id == "enable-disconnected")
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let snapshot = arm.snapshot();
        assert_eq!(snapshot.last_result.as_ref().unwrap().status, "rejected");
        assert!(!serde_json::to_string(&snapshot).unwrap().contains(lease));
        let (mut socket, _) = connect_async(format!("ws://{}/v1/events", server.address))
            .await
            .unwrap();
        assert!(
            tokio::time::timeout(Duration::from_millis(50), socket.next())
                .await
                .is_err()
        );
        socket
            .send(ClientMessage::Text(
                serde_json::json!({"token":token}).to_string().into(),
            ))
            .await
            .unwrap();
        let text = tokio::time::timeout(Duration::from_secs(1), socket.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap()
            .into_text()
            .unwrap();
        let update: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(update["type"], "state");
        assert_eq!(update["data"]["owner"]["client_id"], "test-client");
        assert!(!text.contains(lease));
        assert!(!text.contains(token));
        for _ in 0..50 {
            if journal.stats().written >= 2 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let records: Vec<serde_json::Value> = client
            .get(format!("{base}/v1/logs?limit=50"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!(
            records
                .iter()
                .any(|r| r["request_id"] == "enable-disconnected")
        );
        assert!(!serde_json::to_string(&records).unwrap().contains(lease));
        let _ = socket.close(None).await;
        assert!(arm.shutdown(Duration::from_millis(500)));
        server.shutdown().await;
        assert!(journal.shutdown(Duration::from_secs(1)));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn unknown_browser_origin_is_rejected_even_with_valid_token() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.sqlite3")).unwrap();
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            "key".into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let response = reqwest::Client::new()
            .get(format!("http://{}/v1/state", server.address))
            .bearer_auth("key")
            .header("Origin", "https://unrelated.example")
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), 403);
        arm.shutdown(Duration::from_millis(500));
        server.shutdown().await;
        journal.shutdown(Duration::from_secs(1));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn trace_http_auth_download_and_offline_observation_do_not_issue_commands() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let token = "trace-http-secret";
        let server = launch_at(
            arm.clone(),
            journal.clone(),
            token.into(),
            SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let base = server.base_url();
        let client = reqwest::Client::new();
        let missing = uuid::Uuid::new_v4().to_string();
        for (method, path) in [
            (Method::POST, "/v1/traces".to_owned()),
            (Method::GET, "/v1/traces?limit=20".to_owned()),
            (Method::POST, "/v1/traces/stop".to_owned()),
            (Method::GET, format!("/v1/traces/{missing}?download=true")),
        ] {
            let response = client
                .request(method.clone(), format!("{base}{path}"))
                .json(&serde_json::json!({}))
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
            assert_eq!(
                response.json::<serde_json::Value>().await.unwrap()["code"],
                "unauthorized"
            );
            let response = client
                .request(method, format!("{base}{path}"))
                .bearer_auth(token)
                .header(header::ORIGIN, "https://unrelated.example")
                .json(&serde_json::json!({}))
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::FORBIDDEN);
        }
        let before = serde_json::to_value(arm.snapshot()).unwrap();
        let response = client
            .post(format!("{base}/v1/traces"))
            .bearer_auth(token)
            .json(&serde_json::json!({}))
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        assert_eq!(
            response.json::<serde_json::Value>().await.unwrap()["code"],
            "not_connected"
        );
        for body in [
            serde_json::json!({"duration_ms":999}),
            serde_json::json!({"duration_ms":20_001}),
            serde_json::json!({"duration_ms":-1}),
            serde_json::json!({"duration_ms":"1000"}),
            serde_json::json!({"lease_id":"must-not-be-recorded"}),
        ] {
            let response = client
                .post(format!("{base}/v1/traces"))
                .bearer_auth(token)
                .json(&body)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::BAD_REQUEST);
            let error = response.json::<serde_json::Value>().await.unwrap();
            assert!(error["code"].is_string());
            assert!(error["message"].is_string());
        }
        // Seed observation through the journal without any device connection. The
        // endpoints must stop/read it while disconnected and without a motion lease.
        let trace = journal
            .trace_start(20_000, "left", Some("offline-fixture"))
            .unwrap();
        journal.sample(
            std::time::Instant::now(),
            crate::journal::TraceData::Feedback {
                joint: 0,
                q: 0.1,
                dq: 0.2,
                tau: 0.3,
                temperature: 30.0,
                voltage: 48.0,
                error: 0,
                mode: 1,
                timeout: false,
            },
        );
        let list: serde_json::Value = client
            .get(format!("{base}/v1/traces?limit=20"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(list[0]["id"], trace.id);
        let stop = client
            .post(format!("{base}/v1/traces/stop"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(stop.status(), StatusCode::OK);
        assert_eq!(
            stop.json::<serde_json::Value>().await.unwrap()["id"],
            trace.id
        );
        let stop: serde_json::Value = client
            .post(format!("{base}/v1/traces/stop"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!(stop.is_null());
        for _ in 0..100 {
            if journal.trace_list(1).unwrap()[0].status == "complete" {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        for query in ["", "?download=true", "?attachment=true"] {
            let response = client
                .get(format!("{base}/v1/traces/{}{query}", trace.id))
                .bearer_auth(token)
                .header(header::ORIGIN, "http://localhost:1420")
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::OK);
            assert_eq!(response.headers()[header::CONTENT_TYPE], "application/json");
            assert_eq!(response.headers()[header::CACHE_CONTROL], "no-store");
            if !query.is_empty() {
                assert_eq!(
                    response.headers()[header::CONTENT_DISPOSITION],
                    format!("attachment; filename=\"trace-{}.json\"", trace.id)
                );
            }
            let text = response.text().await.unwrap();
            assert!(!text.contains(token));
            assert!(!text.contains("lease"));
            assert!(!text.contains("request_id"));
            let recording: serde_json::Value = serde_json::from_str(&text).unwrap();
            assert_eq!(recording["trace"]["status"], "complete");
            assert_eq!(
                recording["trace"]["accepted"],
                recording["trace"]["written"]
            );
            let written = recording["trace"]["written"].as_u64().unwrap();
            assert!(written >= 1);
            assert_eq!(recording["trace"]["pending"], 0);
            assert_eq!(
                recording["samples"].as_array().unwrap().len() as u64,
                written
            );
            assert!(
                recording["samples"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|sample| sample["source"] == "feedback" && sample["q"] == 0.1)
            );
        }
        let not_found = client
            .get(format!("{base}/v1/traces/{missing}"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(not_found.status(), StatusCode::NOT_FOUND);
        assert_eq!(
            not_found.json::<serde_json::Value>().await.unwrap()["code"],
            "trace_not_found"
        );
        let invalid = client
            .get(format!("{base}/v1/traces?limit=bad"))
            .bearer_auth(token)
            .send()
            .await
            .unwrap();
        assert_eq!(invalid.status(), StatusCode::BAD_REQUEST);
        assert_eq!(
            invalid.json::<serde_json::Value>().await.unwrap()["code"],
            "invalid_trace_request"
        );
        let after = serde_json::to_value(arm.snapshot()).unwrap();
        for key in [
            "generation",
            "phase",
            "side",
            "port",
            "enabled",
            "owner",
            "action",
            "last_result",
            "results",
        ] {
            assert_eq!(before[key], after[key], "observation changed {key}");
        }
        assert!(arm.shutdown(Duration::from_millis(500)));
        server.shutdown().await;
        assert!(journal.shutdown(Duration::from_secs(2)));
    }
}
