mod arm;
mod journal;
mod kinematics;
mod protocol;
mod server;

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;
use tauri::Manager;

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
) -> Result<(), String> {
    if window.label() != "main" {
        return Err("无权访问此入口".into());
    }
    state.0.arm.host_stop();
    Ok(())
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
        .invoke_handler(tauri::generate_handler![session_info, host_stop])
        .setup(|app| {
            let path = app.path().app_log_dir()?.join("events.sqlite3");
            let journal = journal::Journal::start(path.clone()).map_err(std::io::Error::other)?;
            let arm = arm::ArmHandle::spawn(journal.clone()).map_err(std::io::Error::other)?;
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
            Ok(())
        })
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::CloseRequested { api, .. } = event {
                api.prevent_close();
                shutdown(window.app_handle());
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
                shutdown(app);
            }
        }
    });
}
