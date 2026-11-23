mod arm;
mod calibration;
mod calibration_path;
mod calibration_spline;
mod friction;
mod journal;
mod kinematics;
mod power;
mod protocol;
mod reference;
#[cfg(test)]
mod send_replay;
mod server;
mod teaching;
mod trace;
mod trajectory;
mod tuning;

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;
use tauri::{Emitter, Manager};

#[derive(Clone, serde::Serialize)]
struct SessionInfo {
    base_url: String,
    token: String,
    client_id: String,
    version: String,
    journal_path: String,
}

struct Runtime {
    session: SessionInfo,
    startup_error: Option<String>,
    arm: arm::ArmHandle,
    journal: journal::Journal,
    server: Mutex<Option<server::ServerHandle>>,
    closing: AtomicBool,
    exit_ready: AtomicBool,
}

struct RuntimeState(Arc<Runtime>);

fn keep_awake(state: &arm::Snapshot) -> bool {
    // A dead serial worker retains its port name for fault display. Driver
    // faults on a live connection still need observation and recovery.
    let serial_closed = !state.enabled
        && state.phase == arm::Phase::Fault
        && state
            .error
            .as_ref()
            .is_some_and(|error| error.code == "communication_error");
    state.phase != arm::Phase::Closing && (state.port.is_some() || state.enabled) && !serial_closed
}

#[tauri::command]
fn session_info(
    window: tauri::WebviewWindow,
    state: tauri::State<'_, RuntimeState>,
) -> Result<SessionInfo, String> {
    if window.label() != "main" {
        return Err("无权访问此入口".into());
    }
    if let Some(error) = &state.0.startup_error {
        return Err(error.clone());
    }
    Ok(state.0.session.clone())
}

#[tauri::command]
fn host_stop(
    window: tauri::WebviewWindow,
    state: tauri::State<'_, RuntimeState>,
    reason: String,
) -> Result<(), String> {
    if window.label() != "main" {
        return Err("无权访问此入口".into());
    }
    if reason.is_empty() || reason.len() > 80 {
        return Err("停止原因无效".into());
    }
    state.0.arm.host_stop(reason);
    Ok(())
}

#[tauri::command]
async fn close_app(
    window: tauri::WebviewWindow,
    allow_release: Option<bool>,
) -> Result<(), String> {
    if window.label() != "main" {
        return Err("无权访问此入口".into());
    }
    prepare_close(window.app_handle(), allow_release.unwrap_or(false)).await
}

async fn prepare_close(app: &tauri::AppHandle, allow_release: bool) -> Result<(), String> {
    let Some(runtime) = app.try_state::<RuntimeState>().map(|state| state.0.clone()) else {
        app.exit(0);
        return Ok(());
    };
    if runtime.closing.load(Ordering::Acquire) {
        return Ok(());
    }
    match runtime.arm.prepare_close(allow_release).await {
        Ok(()) => shutdown(app),
        Err(error) if error.code == "support_required" => {
            app.emit_to("main", "close-requested", ())
                .map_err(|error| error.to_string())?;
        }
        Err(error) => return Err(error.message),
    }
    Ok(())
}

fn request_close(app: &tauri::AppHandle) {
    let app = app.clone();
    tauri::async_runtime::spawn(async move {
        if let Err(error) = prepare_close(&app, false).await {
            if let Some(runtime) = app.try_state::<RuntimeState>() {
                runtime
                    .0
                    .journal
                    .log("error", "app", "close_failed", &error, None);
            }
            let _ = app.emit_to("main", "close-requested", ());
        }
    });
}

fn shutdown(app: &tauri::AppHandle) {
    let Some(runtime) = app.try_state::<RuntimeState>().map(|state| state.0.clone()) else {
        return;
    };
    if runtime.closing.swap(true, Ordering::AcqRel) {
        return;
    }
    runtime.arm.request_shutdown();
    let app = app.clone();
    std::thread::spawn(move || {
        let stopped = runtime.arm.shutdown(Duration::from_millis(1200));
        let server = runtime
            .server
            .lock()
            .ok()
            .and_then(|mut server| server.take());
        if let Some(server) = server {
            tauri::async_runtime::block_on(server.shutdown());
        }
        runtime.journal.log(
            "info",
            "app",
            "shutdown",
            if stopped {
                "控制线程已结束"
            } else {
                "设备退出结果未知"
            },
            None,
        );
        runtime.journal.shutdown(Duration::from_millis(600));
        runtime.exit_ready.store(true, Ordering::Release);
        app.exit(0);
    });
}

pub fn run() {
    let application = tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![session_info, host_stop, close_app])
        .setup(|app| {
            let path = app.path().app_log_dir()?.join("events.sqlite3");
            let journal = journal::Journal::start(path.clone()).map_err(std::io::Error::other)?;
            let arm = arm::ArmHandle::spawn(journal.clone()).map_err(std::io::Error::other)?;
            let mut states = arm.subscribe();
            let token = uuid::Uuid::new_v4().to_string();
            let launched = tauri::async_runtime::block_on(server::launch(
                arm.clone(),
                journal.clone(),
                token.clone(),
            ));
            let (server, startup_error) = match launched {
                Ok(server) => (Some(server), None),
                Err(error) => {
                    journal.log("error", "server", "bind_failed", &error, None);
                    (None, Some(error))
                }
            };
            let base_url = server
                .as_ref()
                .map(|server| server.base_url())
                .unwrap_or_else(|| "http://127.0.0.1:8177".into());
            journal.log("info", "app", "started", "控制工作台已启动", None);
            app.manage(RuntimeState(Arc::new(Runtime {
                session: SessionInfo {
                    base_url,
                    token,
                    client_id: "desktop".into(),
                    version: env!("CARGO_PKG_VERSION").into(),
                    journal_path: path.to_string_lossy().into_owned(),
                },
                startup_error,
                arm,
                journal,
                server: Mutex::new(server),
                closing: AtomicBool::new(false),
                exit_ready: AtomicBool::new(false),
            })));
            let app = app.handle().clone();
            tauri::async_runtime::spawn(async move {
                let mut power = power::IdleSleep::default();
                loop {
                    let (connected, closing) = {
                        let state = states.borrow_and_update();
                        (keep_awake(&state), state.phase == arm::Phase::Closing)
                    };
                    // This observer is separate from arm-control. Release the
                    // watch borrow before making short native power calls.
                    if let Err(error) = power.update(connected)
                        && let Some(runtime) = app.try_state::<RuntimeState>()
                    {
                        runtime.0.journal.log(
                            "warn",
                            "app",
                            "power_assertion_failed",
                            &error,
                            None,
                        );
                    }
                    if closing {
                        // An approved close can outlive its IPC timeout. The
                        // committed state must still finish server/log cleanup.
                        shutdown(&app);
                        break;
                    }
                    if states.changed().await.is_err() {
                        break;
                    }
                }
            });
            Ok(())
        })
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::CloseRequested { api, .. } = event {
                api.prevent_close();
                request_close(window.app_handle());
            }
        })
        .build(tauri::generate_context!())
        .expect("控制工作台启动失败");
    application.run(|app, event| {
        if let tauri::RunEvent::ExitRequested { api, .. } = event {
            let ready = app
                .try_state::<RuntimeState>()
                .is_none_or(|state| state.0.exit_ready.load(Ordering::Acquire));
            if !ready {
                api.prevent_exit();
                request_close(app);
            }
        }
    });
}

#[cfg(test)]
mod power_state_tests {
    use super::*;

    #[test]
    fn power_scope_releases_permanent_io_fault_and_preserves_online_faults() {
        // Start only the offline controller to obtain its real Snapshot shape.
        // The mock port below is snapshot data; no Connect command is issued.
        let directory = tempfile::tempdir().unwrap();
        let journal = journal::Journal::start(directory.path().join("events.db")).unwrap();
        let arm = arm::ArmHandle::spawn(journal.clone()).unwrap();
        let mut state = arm.snapshot();
        assert!(arm.shutdown(Duration::from_secs(1)));
        assert!(!keep_awake(&state));

        state.port = Some("/dev/mock-power-snapshot".into());
        for phase in [
            arm::Phase::Connecting,
            arm::Phase::Disabled,
            arm::Phase::Ready,
            arm::Phase::Moving,
            arm::Phase::Stopping,
            arm::Phase::Teaching,
        ] {
            state.phase = phase;
            assert!(keep_awake(&state), "connected {phase:?}");
        }
        state.phase = arm::Phase::Fault;
        state.enabled = false;
        for code in [
            "device_error",
            "device_disabled",
            "device_timeout",
            "feedback_timeout",
        ] {
            state.error = Some(arm::ApiError::new(code, "online drive fault"));
            assert!(keep_awake(&state), "online fault {code}");
        }
        state.error = Some(arm::ApiError::new(
            "communication_error",
            "serial I/O thread is closed",
        ));
        assert!(state.port.is_some());
        assert!(!keep_awake(&state), "permanent serial I/O failure");
        state.enabled = true;
        assert!(
            keep_awake(&state),
            "only the committed disabled fault ends protection"
        );
        state.enabled = false;
        state.phase = arm::Phase::Ready;
        assert!(
            keep_awake(&state),
            "error text alone does not end protection"
        );

        state.phase = arm::Phase::Closing;
        assert!(
            !keep_awake(&state),
            "exit releases even with a retained port"
        );
        state.phase = arm::Phase::Disconnected;
        state.port = None;
        state.error = None;
        assert!(!keep_awake(&state));
        state.phase = arm::Phase::Connecting;
        state.port = Some("/dev/mock-power-snapshot".into());
        assert!(keep_awake(&state), "reconnect restores protection");
        assert!(journal.shutdown(Duration::from_secs(1)));
    }
}
