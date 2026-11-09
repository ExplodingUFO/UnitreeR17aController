use crate::{
    arm::{ApiError, ArmHandle, CommandRequest, ControlRequest},
    journal::{Journal, LogRecord},
    protocol,
};
use axum::{
    Json, Router,
    extract::{
        DefaultBodyLimit, Query, State, WebSocketUpgrade,
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
        .allow_headers([header::AUTHORIZATION, header::CONTENT_TYPE]);
    let router = Router::new()
        .route("/v1/ports", get(ports))
        .route("/v1/state", get(snapshot))
        .route("/v1/control", post(control))
        .route("/v1/commands", post(command))
        .route("/v1/logs", get(logs))
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
            "closing" | "journal_unavailable" => StatusCode::SERVICE_UNAVAILABLE,
            "timeout" => StatusCode::GATEWAY_TIMEOUT,
            "owned" | "not_owner" | "stale_context" | "motion_busy" => StatusCode::CONFLICT,
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
    use tokio_tungstenite::{connect_async, tungstenite::Message as ClientMessage};

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
}
