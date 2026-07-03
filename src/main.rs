use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::mem::size_of;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::{ArgAction, Parser, ValueEnum};
use flatbuffers::FlatBufferBuilder;
use qualisys_rust_sdk::rt::{
    AssembledFrame, Client, ClientOptions, ComponentData, ComponentSelection, ComponentType,
    FrameAccumulator, MocapParameters, MocapSkeletonSegment, StreamFramesRequest, StreamPacket,
    StreamRate, StreamTransport,
};
use semver::Version;
use serde::{Deserialize, Serialize};
use synapse_fbs::synapse::types::{Quaternionf, Vec3f};
use synapse_fbs::topic::{
    MocapDefinition, MocapDefinitionArgs, MocapFrame, MocapFrameArgs, MocapMarkerDefinition,
    MocapMarkerDefinitionArgs, MocapMarkerSample, MocapRigidBodyDefinition,
    MocapRigidBodyDefinitionArgs, MocapRigidBodySample, MocapSegmentDefinition,
    MocapSegmentDefinitionArgs, MocapSegmentSample,
};
use thiserror::Error;
use tiny_http::{Header, Method, Request, Response, Server, StatusCode};
use zenoh::query::{ConsolidationMode, QueryTarget};
use zenoh::{Wait, config::Config};

const APP_NAME: &str = "Synapse Qualisys Bridge";
const BIN_NAME: &str = "synapse-qualisys-bridge";
const GITHUB_OWNER: &str = "CogniPilot";
const GITHUB_REPO: &str = "synapse_qualisys_bridge";
const INDEX_HTML: &str = include_str!("web/index.html");
const APP_CSS: &str = include_str!("web/app.css");
const APP_JS: &str = include_str!("web/app.js");
const QTM_MILLIMETERS_TO_METERS: f32 = 0.001;

#[derive(Debug, Parser)]
#[command(
    name = BIN_NAME,
    version,
    about = "Bridge Qualisys QTM RT mocap data to Synapse FlatBuffers on Zenoh",
    long_about = "Runs a controllable local web interface, listens to a Qualisys QTM RT stream, \
converts selected mocap components into synapse.topic.MocapFrame FlatBuffers, and publishes them on Zenoh.",
    next_line_help = true,
    after_help = "\
Examples:
  synapse-qualisys-bridge --open
  synapse-qualisys-bridge --config bridge.toml --open
  synapse-qualisys-bridge --qualisys-host 192.168.1.10

Environment:
  SYNAPSE_QUALISYS_BRIDGE_CONFIG
  QUALISYS_HOST, QUALISYS_PORT, QUALISYS_TIMEOUT_MS
  ZENOH_MODE, ZENOH_CONNECT, ZENOH_LISTEN
  ZENOH_TOPIC, ZENOH_DEFINITION_TOPIC
  ZENOH_RIGID_BODY_POSE_TOPIC_PREFIX, ZENOH_NO_RIGID_BODY_POSE_TOPICS
  MOCAP_INCLUDE"
)]
struct Cli {
    #[arg(
        long,
        env = "SYNAPSE_QUALISYS_BRIDGE_CONFIG",
        value_name = "FILE",
        help = "TOML configuration file"
    )]
    config: Option<PathBuf>,

    #[arg(
        long,
        value_name = "FILE",
        help = "Write a default TOML configuration file and exit"
    )]
    write_default_config: Option<PathBuf>,

    #[command(flatten)]
    qualisys: QualisysArgs,

    #[command(flatten)]
    stream: StreamArgs,

    #[command(flatten)]
    zenoh: ZenohArgs,

    #[command(flatten)]
    web: WebArgs,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Qualisys")]
struct QualisysArgs {
    #[arg(
        long = "qualisys-host",
        env = "QUALISYS_HOST",
        value_name = "HOST",
        help = "QTM RT host name or IP address"
    )]
    qualisys_host: Option<String>,

    #[arg(
        long = "qualisys-port",
        env = "QUALISYS_PORT",
        value_name = "PORT",
        help = "QTM RT little-endian TCP command port"
    )]
    qualisys_port: Option<u16>,

    #[arg(
        long = "timeout-ms",
        env = "QUALISYS_TIMEOUT_MS",
        value_name = "MS",
        help = "QTM command and stream receive timeout"
    )]
    timeout_ms: Option<u64>,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "QTM Stream")]
struct StreamArgs {
    #[arg(
        long = "include",
        env = "MOCAP_INCLUDE",
        value_enum,
        value_delimiter = ',',
        value_name = "DATA",
        help = "High-rate data to include. Repeat or comma-separate. Default: rigid-bodies"
    )]
    include: Vec<MocapData>,

    #[arg(long, value_enum, help = "Transport used for QTM streamframes")]
    transport: Option<StreamTransportArg>,

    #[arg(
        long,
        value_name = "ADDR:PORT",
        help = "Local UDP bind address when --transport udp"
    )]
    udp_bind: Option<SocketAddr>,

    #[arg(
        long,
        value_name = "IP",
        help = "Optional QTM UDP destination IP; defaults to QTM's peer address"
    )]
    udp_destination: Option<IpAddr>,
}

#[derive(Debug, Clone, Copy, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum StreamTransportArg {
    Udp,
    Tcp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum MocapData {
    RigidBodies,
    LabeledMarkers,
    UnlabeledMarkers,
    Skeleton,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Zenoh")]
struct ZenohArgs {
    #[arg(
        long = "zenoh-mode",
        env = "ZENOH_MODE",
        value_enum,
        value_name = "MODE",
        help = "Zenoh runtime mode: router, client, or peer"
    )]
    zenoh_mode: Option<ZenohMode>,

    #[arg(
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        help = "Zenoh router locator or upstream router endpoint. Repeat endpoints with commas."
    )]
    zenoh_connect: Option<String>,

    #[arg(
        long = "zenoh-listen",
        env = "ZENOH_LISTEN",
        value_name = "LOCATOR",
        help = "Zenoh listen endpoint used in router or peer mode. Repeat endpoints with commas."
    )]
    zenoh_listen: Option<String>,

    #[arg(
        long = "topic",
        alias = "zenoh-topic",
        env = "ZENOH_TOPIC",
        value_name = "KEYEXPR",
        help = "Zenoh key expression for synapse.topic.MocapFrame payloads"
    )]
    topic: Option<String>,

    #[arg(
        long = "definition-topic",
        alias = "zenoh-definition-topic",
        env = "ZENOH_DEFINITION_TOPIC",
        value_name = "KEYEXPR",
        help = "Zenoh key expression for low-rate synapse.topic.MocapDefinition metadata"
    )]
    definition_topic: Option<String>,

    #[arg(
        long = "rigid-body-pose-topic-prefix",
        env = "ZENOH_RIGID_BODY_POSE_TOPIC_PREFIX",
        value_name = "KEYEXPR",
        help = "Zenoh key expression prefix for compact per-rigid-body pose topics"
    )]
    rigid_body_pose_topic_prefix: Option<String>,

    #[arg(
        long = "no-rigid-body-pose-topics",
        env = "ZENOH_NO_RIGID_BODY_POSE_TOPICS",
        action = ArgAction::SetTrue,
        help = "Disable compact per-rigid-body pose topic publishing"
    )]
    no_rigid_body_pose_topics: bool,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Web Interface")]
struct WebArgs {
    #[arg(
        long = "web-bind",
        value_name = "ADDR:PORT",
        help = "Local web interface bind address"
    )]
    web_bind: Option<SocketAddr>,

    #[arg(
        long = "no-web",
        action = ArgAction::SetTrue,
        help = "Disable the web interface and run the bridge in the foreground"
    )]
    no_web: bool,

    #[arg(
        long = "open",
        action = ArgAction::SetTrue,
        help = "Open the local web interface in the default browser"
    )]
    open_browser: bool,

    #[arg(
        long = "no-autostart",
        action = ArgAction::SetTrue,
        help = "Start the web interface without automatically starting the bridge"
    )]
    no_autostart: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct AppConfig {
    bridge: BridgeConfig,
    qualisys: QualisysConfig,
    stream: StreamConfig,
    zenoh: ZenohConfig,
    web: WebConfig,
    logging: LoggingConfig,
    updates: UpdateConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            bridge: BridgeConfig::default(),
            qualisys: QualisysConfig::default(),
            stream: StreamConfig::default(),
            zenoh: ZenohConfig::default(),
            web: WebConfig::default(),
            logging: LoggingConfig::default(),
            updates: UpdateConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct BridgeConfig {
    autostart: bool,
    reconnect_delay_ms: u64,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            autostart: true,
            reconnect_delay_ms: 2_000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct QualisysConfig {
    host: String,
    port: u16,
    timeout_ms: u64,
}

impl Default for QualisysConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".to_owned(),
            port: qualisys_rust_sdk::rt::LITTLE_ENDIAN_PORT,
            timeout_ms: 5_000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct StreamConfig {
    include: Vec<MocapData>,
    transport: StreamTransportArg,
    udp_bind: SocketAddr,
    udp_destination: Option<IpAddr>,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            include: vec![MocapData::RigidBodies],
            transport: StreamTransportArg::Udp,
            udp_bind: "0.0.0.0:0".parse().expect("valid default UDP bind"),
            udp_destination: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct ZenohConfig {
    mode: ZenohMode,
    connect: String,
    listen: String,
    topic: String,
    definition_topic: String,
    rigid_body_pose_topic_prefix: String,
    no_rigid_body_pose_topics: bool,
    admin_query_timeout_ms: u64,
}

impl Default for ZenohConfig {
    fn default() -> Self {
        Self {
            mode: ZenohMode::Router,
            connect: String::new(),
            listen: "udp/0.0.0.0:7447,tcp/0.0.0.0:7447".to_owned(),
            topic: "synapse/mocap/frame".to_owned(),
            definition_topic: "synapse/mocap/definition".to_owned(),
            rigid_body_pose_topic_prefix: "synapse/mocap/rigid_body".to_owned(),
            no_rigid_body_pose_topics: false,
            admin_query_timeout_ms: 1_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum ZenohMode {
    Router,
    Client,
    Peer,
}

impl ZenohMode {
    fn as_str(self) -> &'static str {
        match self {
            ZenohMode::Router => "router",
            ZenohMode::Client => "client",
            ZenohMode::Peer => "peer",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct WebConfig {
    enabled: bool,
    bind: SocketAddr,
    open_browser: bool,
}

impl Default for WebConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            bind: "127.0.0.1:8787".parse().expect("valid default web bind"),
            open_browser: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
struct LoggingConfig {
    log_file: Option<PathBuf>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct UpdateConfig {
    enabled: bool,
    auto_check: bool,
    auto_install: bool,
    check_interval_minutes: u64,
    github_owner: String,
    github_repo: String,
}

impl Default for UpdateConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            auto_check: true,
            auto_install: cfg!(windows),
            check_interval_minutes: 360,
            github_owner: GITHUB_OWNER.to_owned(),
            github_repo: GITHUB_REPO.to_owned(),
        }
    }
}

#[derive(Debug, Error)]
enum BridgeError {
    #[error(transparent)]
    Qtm(#[from] qualisys_rust_sdk::QtmError),
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("frame does not contain requested {0} component")]
    MissingComponent(&'static str),
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("configuration error: {0}")]
    Config(String),
    #[error("web interface error: {0}")]
    Web(String),
    #[error("update error: {0}")]
    Update(String),
}

type Result<T> = std::result::Result<T, BridgeError>;

#[derive(Debug)]
struct BridgeWorker {
    stop: Arc<AtomicBool>,
    join: JoinHandle<()>,
}

#[derive(Debug)]
struct AppInner {
    config: RwLock<AppConfig>,
    state: Mutex<AppState>,
    config_path: PathBuf,
    log_path: PathBuf,
    bridge_worker: Mutex<Option<BridgeWorker>>,
    zenoh_session: Mutex<Option<zenoh::Session>>,
}

#[derive(Debug, Clone)]
struct AppHandle(Arc<AppInner>);

impl AppHandle {
    fn new(config: AppConfig, config_path: PathBuf, log_path: PathBuf) -> Self {
        Self(Arc::new(AppInner {
            config: RwLock::new(config),
            state: Mutex::new(AppState::new()),
            config_path,
            log_path,
            bridge_worker: Mutex::new(None),
            zenoh_session: Mutex::new(None),
        }))
    }

    fn config(&self) -> AppConfig {
        self.0.config.read().expect("config lock poisoned").clone()
    }

    fn replace_config(&self, config: AppConfig) -> Result<()> {
        save_config(&self.0.config_path, &config)?;
        *self.0.config.write().expect("config lock poisoned") = config;
        self.close_zenoh_session()?;
        self.log("info", "Configuration saved");
        Ok(())
    }

    fn snapshot(&self) -> AppSnapshot {
        let config = self.config();
        let state = self.0.state.lock().expect("state lock poisoned");
        AppSnapshot {
            app_name: APP_NAME.to_owned(),
            version: env!("CARGO_PKG_VERSION").to_owned(),
            config_path: self.0.config_path.display().to_string(),
            log_path: self.0.log_path.display().to_string(),
            web_url: web_url(config.web.bind),
            bridge: state.bridge.clone(),
            qualisys: state.qualisys.clone(),
            zenoh: state.zenoh.clone(),
            zenoh_access: zenoh_access(&config),
            stats: state.stats.clone(),
            topics: state.topics.values().cloned().collect(),
            rigid_bodies: state.rigid_bodies.clone(),
            logs: state.logs.iter().cloned().collect(),
            update: state.update.clone(),
        }
    }

    fn update_state<F>(&self, update: F)
    where
        F: FnOnce(&mut AppState),
    {
        let mut state = self.0.state.lock().expect("state lock poisoned");
        update(&mut state);
    }

    fn set_bridge_status(&self, state: &str, detail: impl Into<String>, error: Option<String>) {
        self.update_state(|app| {
            app.bridge = ComponentStatus::new(state, detail, error);
        });
    }

    fn set_qualisys_status(&self, state: &str, detail: impl Into<String>, error: Option<String>) {
        self.update_state(|app| {
            app.qualisys = ComponentStatus::new(state, detail, error);
        });
    }

    fn set_zenoh_status(&self, state: &str, detail: impl Into<String>, error: Option<String>) {
        self.update_state(|app| {
            app.zenoh = ComponentStatus::new(state, detail, error);
        });
    }

    fn reset_run_stats(&self) {
        self.update_state(|app| {
            app.stats = BridgeStats::default();
            app.topics.clear();
            app.rigid_bodies.clear();
            app.rate_window_at_ms = now_ms();
            app.rate_window_frames = 0;
        });
    }

    fn record_frame(
        &self,
        frame_number: u32,
        timestamp_us: u64,
        rigid_bodies: Vec<RigidBodyStatus>,
    ) {
        let now = now_ms();
        self.update_state(|app| {
            app.stats.frames += 1;
            app.stats.last_frame_number = Some(frame_number);
            app.stats.last_frame_timestamp_us = Some(timestamp_us);
            app.stats.last_frame_at_ms = Some(now);
            app.rigid_bodies = rigid_bodies;

            let delta_ms = now.saturating_sub(app.rate_window_at_ms);
            if delta_ms >= 1_000 {
                let delta_frames = app.stats.frames.saturating_sub(app.rate_window_frames);
                app.stats.frames_per_second = (delta_frames as f64) * 1000.0 / (delta_ms as f64);
                app.rate_window_at_ms = now;
                app.rate_window_frames = app.stats.frames;
            }
        });
    }

    fn record_topic(&self, key_expr: impl Into<String>, role: &str, payload_len: usize) {
        let key_expr = key_expr.into();
        let now = now_ms();
        self.update_state(|app| {
            let topic = app
                .topics
                .entry(key_expr.clone())
                .or_insert_with(|| TopicStats {
                    key_expr,
                    role: role.to_owned(),
                    messages: 0,
                    bytes: 0,
                    last_payload_bytes: 0,
                    last_published_at_ms: None,
                });
            topic.messages += 1;
            topic.bytes += payload_len as u64;
            topic.last_payload_bytes = payload_len as u64;
            topic.last_published_at_ms = Some(now);
            app.stats.published_messages += 1;
            app.stats.published_bytes += payload_len as u64;
        });
    }

    fn log(&self, level: &str, message: impl AsRef<str>) {
        let message = message.as_ref().to_owned();
        let entry = LogEntry {
            time_ms: now_ms(),
            level: level.to_owned(),
            message: message.clone(),
        };

        {
            let mut state = self.0.state.lock().expect("state lock poisoned");
            state.logs.push_back(entry.clone());
            while state.logs.len() > 300 {
                state.logs.pop_front();
            }
        }

        eprintln!(
            "[{}] {} {}",
            entry.time_ms,
            entry.level.to_uppercase(),
            message
        );
        if let Some(parent) = self.0.log_path.parent() {
            let _ = fs::create_dir_all(parent);
        }
        if let Ok(mut file) = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.0.log_path)
        {
            let _ = writeln!(
                file,
                "[{}] {} {}",
                entry.time_ms,
                entry.level.to_uppercase(),
                message
            );
        }
    }

    fn clear_logs(&self) -> Result<()> {
        {
            let mut state = self.0.state.lock().expect("state lock poisoned");
            state.logs.clear();
        }
        if let Some(parent) = self.0.log_path.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(&self.0.log_path, "")?;
        Ok(())
    }

    fn start_bridge(&self) -> bool {
        let mut worker = self.0.bridge_worker.lock().expect("bridge lock poisoned");
        if worker
            .as_ref()
            .is_some_and(|existing| !existing.join.is_finished())
        {
            return false;
        }
        if let Some(done) = worker.take() {
            let _ = done.join.join();
        }

        self.reset_run_stats();
        let stop = Arc::new(AtomicBool::new(false));
        let app = self.clone();
        let thread_stop = stop.clone();
        let join = thread::spawn(move || run_bridge_loop(app, thread_stop));
        *worker = Some(BridgeWorker { stop, join });
        true
    }

    fn stop_bridge(&self) -> bool {
        let mut worker = self.0.bridge_worker.lock().expect("bridge lock poisoned");
        if worker
            .as_ref()
            .is_some_and(|existing| existing.join.is_finished())
        {
            if let Some(done) = worker.take() {
                let _ = done.join.join();
            }
            self.set_bridge_status("stopped", "Bridge is stopped", None);
            return false;
        }

        if let Some(existing) = worker.as_ref() {
            existing.stop.store(true, Ordering::SeqCst);
            self.set_bridge_status("stopping", "Stopping bridge", None);
            self.log("info", "Bridge stop requested");
            return true;
        }
        false
    }

    fn restart_bridge(&self) -> bool {
        self.stop_bridge();
        thread::sleep(Duration::from_millis(250));
        self.start_bridge()
    }

    fn run_qualisys_diagnostic(&self) -> QualisysDiagnostic {
        let config = self.config();
        match diagnose_qualisys(&config) {
            Ok(report) => {
                self.set_qualisys_status(
                    "connected",
                    format!(
                        "{} rigid bodies, {} marker labels, {} skeletons",
                        report.rigid_bodies, report.marker_labels, report.skeletons
                    ),
                    None,
                );
                self.log(
                    "info",
                    format!(
                        "Qualisys diagnostic succeeded in {} ms",
                        report.round_trip_ms
                    ),
                );
                report
            }
            Err(error) => {
                let report = QualisysDiagnostic {
                    ok: false,
                    host: config.qualisys.host,
                    port: config.qualisys.port,
                    round_trip_ms: 0,
                    rigid_bodies: 0,
                    marker_labels: 0,
                    skeletons: 0,
                    error: Some(error.to_string()),
                };
                self.set_qualisys_status(
                    "error",
                    "Qualisys diagnostic failed",
                    report.error.clone(),
                );
                self.log("error", format!("Qualisys diagnostic failed: {error}"));
                report
            }
        }
    }

    fn discover_zenoh(&self) -> ZenohDiscovery {
        match discover_zenoh(self) {
            Ok(discovery) => {
                self.set_zenoh_status("connected", "Zenoh admin discovery completed", None);
                discovery
            }
            Err(error) => {
                self.set_zenoh_status(
                    "error",
                    "Zenoh admin discovery failed",
                    Some(error.to_string()),
                );
                ZenohDiscovery {
                    ok: false,
                    subscribers: Vec::new(),
                    publishers: Vec::new(),
                    queryables: Vec::new(),
                    errors: vec![error.to_string()],
                }
            }
        }
    }

    fn ensure_zenoh_session(&self) -> Result<zenoh::Session> {
        let mut session = self
            .0
            .zenoh_session
            .lock()
            .expect("zenoh session lock poisoned");
        if let Some(existing) = session.as_ref() {
            return Ok(existing.clone());
        }

        let config = self.config();
        self.set_zenoh_status(
            "starting",
            format!("Starting Zenoh {} mode", config.zenoh.mode.as_str()),
            None,
        );
        let opened = zenoh::open(zenoh_config(&config)?)
            .wait()
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
        let detail = zenoh_status_detail(&config);
        self.set_zenoh_status("running", detail.clone(), None);
        self.log("info", detail);
        *session = Some(opened.clone());
        Ok(opened)
    }

    fn close_zenoh_session(&self) -> Result<bool> {
        let session = self
            .0
            .zenoh_session
            .lock()
            .expect("zenoh session lock poisoned")
            .take();
        if let Some(session) = session {
            session
                .close()
                .wait()
                .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
            self.set_zenoh_status("stopped", "Zenoh session stopped", None);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn check_update(&self) -> UpdateStatus {
        let config = self.config();
        let status = match check_for_update(&config.updates) {
            Ok(status) => status,
            Err(error) => UpdateStatus {
                state: "error".to_owned(),
                checked_at_ms: Some(now_ms()),
                latest_version: None,
                current_version: env!("CARGO_PKG_VERSION").to_owned(),
                release_url: None,
                asset_name: None,
                asset_download_url: None,
                available: false,
                message: Some(error.to_string()),
            },
        };
        self.update_state(|state| {
            state.update = status.clone();
        });
        status
    }

    fn install_update(&self) -> UpdateStatus {
        let mut status = {
            let state = self.0.state.lock().expect("state lock poisoned");
            state.update.clone()
        };

        if !status.available || status.asset_download_url.is_none() {
            status = self.check_update();
        }

        let result = install_update_from_status(&status);
        let next = match result {
            Ok(message) => UpdateStatus {
                state: "installing".to_owned(),
                message: Some(message),
                ..status
            },
            Err(error) => UpdateStatus {
                state: "error".to_owned(),
                message: Some(error.to_string()),
                ..status
            },
        };

        self.update_state(|state| {
            state.update = next.clone();
        });
        next
    }
}

#[derive(Debug, Clone, Serialize)]
struct AppSnapshot {
    app_name: String,
    version: String,
    config_path: String,
    log_path: String,
    web_url: String,
    bridge: ComponentStatus,
    qualisys: ComponentStatus,
    zenoh: ComponentStatus,
    zenoh_access: ZenohAccess,
    stats: BridgeStats,
    topics: Vec<TopicStats>,
    rigid_bodies: Vec<RigidBodyStatus>,
    logs: Vec<LogEntry>,
    update: UpdateStatus,
}

#[derive(Debug, Clone, Serialize)]
struct ComponentStatus {
    state: String,
    detail: String,
    last_error: Option<String>,
    updated_at_ms: u128,
}

#[derive(Debug, Clone, Serialize)]
struct ZenohAccess {
    mode: String,
    listen: String,
    connect: String,
    subscriber_endpoint: Option<String>,
    subscriber_endpoints: Vec<String>,
    frame_topic: String,
    definition_topic: String,
    rigid_body_topic_prefix: String,
    sample_key_expr: String,
    example_subscriber_args: Option<String>,
}

impl ComponentStatus {
    fn new(state: &str, detail: impl Into<String>, last_error: Option<String>) -> Self {
        Self {
            state: state.to_owned(),
            detail: detail.into(),
            last_error,
            updated_at_ms: now_ms(),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
struct BridgeStats {
    frames: u64,
    frames_per_second: f64,
    published_messages: u64,
    published_bytes: u64,
    last_frame_number: Option<u32>,
    last_frame_timestamp_us: Option<u64>,
    last_frame_at_ms: Option<u128>,
}

#[derive(Debug, Clone, Serialize)]
struct TopicStats {
    key_expr: String,
    role: String,
    messages: u64,
    bytes: u64,
    last_payload_bytes: u64,
    last_published_at_ms: Option<u128>,
}

#[derive(Debug, Clone, Serialize)]
struct RigidBodyStatus {
    id: i32,
    name: String,
    position_m: [f32; 3],
    quaternion: [f32; 4],
    residual: f32,
    tracking_valid: bool,
}

#[derive(Debug, Clone, Serialize)]
struct LogEntry {
    time_ms: u128,
    level: String,
    message: String,
}

#[derive(Debug, Clone, Serialize)]
struct UpdateStatus {
    state: String,
    checked_at_ms: Option<u128>,
    latest_version: Option<String>,
    current_version: String,
    release_url: Option<String>,
    asset_name: Option<String>,
    asset_download_url: Option<String>,
    available: bool,
    message: Option<String>,
}

impl Default for UpdateStatus {
    fn default() -> Self {
        Self {
            state: "idle".to_owned(),
            checked_at_ms: None,
            latest_version: None,
            current_version: env!("CARGO_PKG_VERSION").to_owned(),
            release_url: None,
            asset_name: None,
            asset_download_url: None,
            available: false,
            message: None,
        }
    }
}

#[derive(Debug)]
struct AppState {
    bridge: ComponentStatus,
    qualisys: ComponentStatus,
    zenoh: ComponentStatus,
    stats: BridgeStats,
    topics: BTreeMap<String, TopicStats>,
    rigid_bodies: Vec<RigidBodyStatus>,
    logs: VecDeque<LogEntry>,
    update: UpdateStatus,
    rate_window_at_ms: u128,
    rate_window_frames: u64,
}

impl AppState {
    fn new() -> Self {
        Self {
            bridge: ComponentStatus::new("stopped", "Bridge is stopped", None),
            qualisys: ComponentStatus::new("unknown", "No Qualisys connection attempted", None),
            zenoh: ComponentStatus::new("unknown", "No Zenoh connection attempted", None),
            stats: BridgeStats::default(),
            topics: BTreeMap::new(),
            rigid_bodies: Vec::new(),
            logs: VecDeque::new(),
            update: UpdateStatus::default(),
            rate_window_at_ms: now_ms(),
            rate_window_frames: 0,
        }
    }
}

#[derive(Debug, Serialize)]
struct MessageResponse {
    message: String,
}

#[derive(Debug, Serialize)]
struct QualisysDiagnostic {
    ok: bool,
    host: String,
    port: u16,
    round_trip_ms: u128,
    rigid_bodies: usize,
    marker_labels: usize,
    skeletons: usize,
    error: Option<String>,
}

#[derive(Debug, Serialize)]
struct ZenohDiscovery {
    ok: bool,
    subscribers: Vec<ZenohAdminEntry>,
    publishers: Vec<ZenohAdminEntry>,
    queryables: Vec<ZenohAdminEntry>,
    errors: Vec<String>,
}

#[derive(Debug, Serialize)]
struct ZenohAdminEntry {
    key_expr: String,
    payload: Option<serde_json::Value>,
}

#[derive(Debug, Deserialize)]
struct GitHubRelease {
    tag_name: String,
    html_url: String,
    assets: Vec<GitHubAsset>,
}

#[derive(Debug, Deserialize)]
struct GitHubAsset {
    name: String,
    browser_download_url: String,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    if let Some(path) = cli.write_default_config.as_ref() {
        save_config(path, &AppConfig::default())?;
        println!("Wrote default configuration to {}", path.display());
        return Ok(());
    }

    let config_path = resolve_config_path(cli.config.as_deref());
    let mut config = load_config_or_default(&config_path)?;
    apply_cli_overrides(&mut config, &cli);
    ensure_config_file(&config_path, &config)?;

    let log_path = resolve_log_path(&config);
    let app = AppHandle::new(config.clone(), config_path, log_path);
    app.log("info", format!("{APP_NAME} {}", env!("CARGO_PKG_VERSION")));
    app.log(
        "info",
        format!("Using configuration {}", app.0.config_path.display()),
    );

    if !config.web.enabled {
        app.log(
            "info",
            "Web interface disabled; running bridge in foreground",
        );
        run_bridge_loop(app, Arc::new(AtomicBool::new(false)));
        return Ok(());
    }

    let web_bind = config.web.bind;
    let web_app = app.clone();
    thread::spawn(move || {
        if let Err(error) = run_web_server(web_app.clone(), web_bind) {
            web_app.log("error", format!("Web interface failed: {error}"));
        }
    });

    let url = web_url(web_bind);
    app.log("info", format!("Web interface listening at {url}"));

    if config.updates.enabled && config.updates.auto_check {
        let update_app = app.clone();
        thread::spawn(move || update_worker(update_app));
    }

    if config.bridge.autostart {
        app.start_bridge();
    }

    if config.web.open_browser {
        open_browser(&url);
    }

    loop {
        thread::park();
    }
}

fn apply_cli_overrides(config: &mut AppConfig, cli: &Cli) {
    if let Some(host) = cli.qualisys.qualisys_host.as_ref() {
        config.qualisys.host = host.clone();
    }
    if let Some(port) = cli.qualisys.qualisys_port {
        config.qualisys.port = port;
    }
    if let Some(timeout_ms) = cli.qualisys.timeout_ms {
        config.qualisys.timeout_ms = timeout_ms;
    }
    if !cli.stream.include.is_empty() {
        config.stream.include = cli.stream.include.clone();
    }
    if let Some(transport) = cli.stream.transport {
        config.stream.transport = transport;
    }
    if let Some(udp_bind) = cli.stream.udp_bind {
        config.stream.udp_bind = udp_bind;
    }
    if cli.stream.udp_destination.is_some() {
        config.stream.udp_destination = cli.stream.udp_destination;
    }
    if let Some(mode) = cli.zenoh.zenoh_mode {
        config.zenoh.mode = mode;
    }
    if let Some(connect) = cli.zenoh.zenoh_connect.as_ref() {
        config.zenoh.connect = connect.clone();
    }
    if let Some(listen) = cli.zenoh.zenoh_listen.as_ref() {
        config.zenoh.listen = listen.clone();
    }
    if let Some(topic) = cli.zenoh.topic.as_ref() {
        config.zenoh.topic = topic.clone();
    }
    if let Some(topic) = cli.zenoh.definition_topic.as_ref() {
        config.zenoh.definition_topic = topic.clone();
    }
    if let Some(prefix) = cli.zenoh.rigid_body_pose_topic_prefix.as_ref() {
        config.zenoh.rigid_body_pose_topic_prefix = prefix.clone();
    }
    if cli.zenoh.no_rigid_body_pose_topics {
        config.zenoh.no_rigid_body_pose_topics = true;
    }
    if let Some(bind) = cli.web.web_bind {
        config.web.bind = bind;
    }
    if cli.web.no_web {
        config.web.enabled = false;
    }
    if cli.web.open_browser {
        config.web.open_browser = true;
    }
    if cli.web.no_autostart {
        config.bridge.autostart = false;
    }
}

fn run_bridge_loop(app: AppHandle, stop: Arc<AtomicBool>) {
    app.set_bridge_status("starting", "Bridge thread starting", None);
    app.log("info", "Bridge thread started");

    while !stop.load(Ordering::SeqCst) {
        let config = app.config();
        match run_bridge_once(&config, &app, &stop) {
            Ok(()) => {
                if !stop.load(Ordering::SeqCst) {
                    app.log("warn", "QTM stream ended; reconnecting");
                }
            }
            Err(error) => {
                match &error {
                    BridgeError::Qtm(_) => app.set_qualisys_status(
                        "error",
                        "Qualisys connection or stream failed",
                        Some(error.to_string()),
                    ),
                    BridgeError::Zenoh(_) => {
                        let message = error.to_string();
                        let _ = app.close_zenoh_session();
                        app.set_zenoh_status(
                            "error",
                            "Zenoh connection or publish failed",
                            Some(message),
                        );
                    }
                    _ => {}
                }
                app.set_bridge_status("retrying", "Bridge will retry", Some(error.to_string()));
                app.log("error", format!("Bridge error: {error}"));
            }
        }

        if !stop.load(Ordering::SeqCst) {
            sleep_stop_aware(
                &stop,
                Duration::from_millis(app.config().bridge.reconnect_delay_ms),
            );
        }
    }

    app.set_bridge_status("stopped", "Bridge stopped", None);
    app.set_qualisys_status("unknown", "Bridge stopped", None);
    app.log("info", "Bridge thread stopped");
}

fn run_bridge_once(config: &AppConfig, app: &AppHandle, stop: &AtomicBool) -> Result<()> {
    app.set_bridge_status("starting", "Opening Zenoh session", None);
    let session = app.ensure_zenoh_session()?;

    let publisher = session
        .declare_publisher(config.zenoh.topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let definition_publisher = session
        .declare_publisher(config.zenoh.definition_topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    app.set_qualisys_status(
        "connecting",
        format!(
            "Connecting to {}:{}",
            config.qualisys.host, config.qualisys.port
        ),
        None,
    );
    let mut client = connect_qualisys(config)?;
    app.set_qualisys_status(
        "connected",
        format!(
            "Connected to {}:{}",
            config.qualisys.host, config.qualisys.port
        ),
        None,
    );

    let selection = selected_data(config);
    let parameters = client.get_mocap_parameters()?;
    let rigid_body_names = rigid_body_names(&parameters);
    let definition = encode_mocap_definition(&parameters, &selection)?;
    let definition_len = definition.len();
    definition_publisher
        .put(definition)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    app.record_topic(
        config.zenoh.definition_topic.clone(),
        "mocap definition publisher",
        definition_len,
    );

    let components = stream_components(&selection);
    let request = StreamFramesRequest::new(
        StreamRate::AllFrames,
        stream_transport(config),
        components.clone(),
    );
    client.start_stream_frames(&request)?;
    app.set_bridge_status("running", "QTM stream running", None);
    app.log(
        "info",
        format!(
            "Bridge running: QTM {}:{}, {}, topic {}",
            config.qualisys.host,
            config.qualisys.port,
            zenoh_status_detail(config),
            config.zenoh.topic
        ),
    );

    let mut accumulator = FrameAccumulator::for_components(components);
    while !stop.load(Ordering::SeqCst) {
        match client.recv_stream_packet()? {
            StreamPacket::Data(packet) => {
                for frame in accumulator.push(packet) {
                    let payload = encode_mocap_frame(&frame, &selection)?;
                    let payload_len = payload.len();
                    publisher
                        .put(payload)
                        .wait()
                        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
                    app.record_topic(
                        config.zenoh.topic.clone(),
                        "mocap frame publisher",
                        payload_len,
                    );

                    let compact_topics = publish_rigid_body_pose_topics(
                        &session,
                        config,
                        &frame,
                        &selection,
                        &rigid_body_names,
                    )?;
                    for (topic, payload_len) in compact_topics {
                        app.record_topic(topic, "compact rigid body pose publisher", payload_len);
                    }

                    let body_statuses = if selection.contains(&MocapData::RigidBodies) {
                        rigid_body_statuses(&frame, &rigid_body_names)?
                    } else {
                        Vec::new()
                    };
                    app.record_frame(frame.frame_number, frame.timestamp, body_statuses);
                }
            }
            StreamPacket::NoMoreData => return Ok(()),
        }
    }

    Ok(())
}

fn zenoh_config(config: &AppConfig) -> Result<Config> {
    let mut zenoh_config = Config::default();
    zenoh_config
        .insert_json5("mode", &format!("\"{}\"", config.zenoh.mode.as_str()))
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    let connect = endpoint_list(&config.zenoh.connect);
    if !connect.is_empty() {
        zenoh_config
            .insert_json5(
                "connect/endpoints",
                &serde_json::to_string(&connect)
                    .map_err(|error| BridgeError::Zenoh(error.to_string()))?,
            )
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    }

    let listen = endpoint_list(&config.zenoh.listen);
    if !listen.is_empty() && !matches!(config.zenoh.mode, ZenohMode::Client) {
        zenoh_config
            .insert_json5(
                "listen/endpoints",
                &serde_json::to_string(&listen)
                    .map_err(|error| BridgeError::Zenoh(error.to_string()))?,
            )
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    }

    zenoh_config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    Ok(zenoh_config)
}

fn endpoint_list(value: &str) -> Vec<String> {
    value
        .split(|ch: char| ch == ',' || ch == ';' || ch.is_whitespace())
        .filter_map(|part| {
            let endpoint = part.trim();
            (!endpoint.is_empty()).then(|| endpoint.to_owned())
        })
        .collect()
}

fn zenoh_access(config: &AppConfig) -> ZenohAccess {
    let subscriber_endpoints = match config.zenoh.mode {
        ZenohMode::Client => endpoint_list(&config.zenoh.connect),
        ZenohMode::Router | ZenohMode::Peer => advertised_zenoh_endpoints(&config.zenoh.listen),
    };
    let subscriber_endpoint = subscriber_endpoints.first().cloned();
    let sample_key_expr = subscription_key_expr(config);
    let example_subscriber_args = subscriber_endpoint
        .as_ref()
        .map(|endpoint| format!("-e {endpoint} -k '{sample_key_expr}'"));

    ZenohAccess {
        mode: config.zenoh.mode.as_str().to_owned(),
        listen: config.zenoh.listen.clone(),
        connect: config.zenoh.connect.clone(),
        subscriber_endpoint,
        subscriber_endpoints,
        frame_topic: config.zenoh.topic.clone(),
        definition_topic: config.zenoh.definition_topic.clone(),
        rigid_body_topic_prefix: config.zenoh.rigid_body_pose_topic_prefix.clone(),
        sample_key_expr,
        example_subscriber_args,
    }
}

fn advertised_zenoh_endpoints(listen: &str) -> Vec<String> {
    let host = local_network_ip()
        .map(|ip| ip.to_string())
        .unwrap_or_else(|| "<bridge-pc-ip>".to_owned());
    endpoint_list(listen)
        .into_iter()
        .map(|endpoint| {
            endpoint
                .replace("0.0.0.0", &host)
                .replace("[::]", &host)
                .replace("::", &host)
                .replace("127.0.0.1", &host)
                .replace("localhost", &host)
        })
        .collect()
}

fn local_network_ip() -> Option<IpAddr> {
    let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
    socket.connect("8.8.8.8:80").ok()?;
    let ip = socket.local_addr().ok()?.ip();
    (!ip.is_loopback()).then_some(ip)
}

fn subscription_key_expr(config: &AppConfig) -> String {
    let topics = [
        config.zenoh.topic.as_str(),
        config.zenoh.definition_topic.as_str(),
        config.zenoh.rigid_body_pose_topic_prefix.as_str(),
    ];
    let parts: Vec<Vec<&str>> = topics
        .iter()
        .map(|topic| topic.trim_matches('/').split('/').collect())
        .filter(|parts: &Vec<&str>| !parts.is_empty())
        .collect();

    let Some(first) = parts.first() else {
        return "**".to_owned();
    };

    let mut shared = Vec::new();
    for (index, part) in first.iter().enumerate() {
        if parts
            .iter()
            .all(|candidate| candidate.get(index).is_some_and(|value| value == part))
        {
            shared.push(*part);
        } else {
            break;
        }
    }

    if shared.is_empty() {
        config.zenoh.topic.clone()
    } else {
        format!("{}/**", shared.join("/"))
    }
}

fn zenoh_status_detail(config: &AppConfig) -> String {
    let listen = endpoint_list(&config.zenoh.listen).join(", ");
    let connect = endpoint_list(&config.zenoh.connect).join(", ");
    match config.zenoh.mode {
        ZenohMode::Router => format!(
            "Zenoh router mode listening on {}{}",
            listen,
            if connect.is_empty() {
                String::new()
            } else {
                format!(", connected to {}", connect)
            }
        ),
        ZenohMode::Peer => format!(
            "Zenoh peer mode listening on {}{}",
            listen,
            if connect.is_empty() {
                String::new()
            } else {
                format!(", connected to {}", connect)
            }
        ),
        ZenohMode::Client => format!("Zenoh client mode connected to {}", connect),
    }
}

fn connect_qualisys(config: &AppConfig) -> Result<Client> {
    Ok(Client::connect(
        &config.qualisys.host,
        ClientOptions {
            port: config.qualisys.port,
            read_timeout: Duration::from_millis(config.qualisys.timeout_ms),
            ..ClientOptions::default()
        },
    )?)
}

fn stream_transport(config: &AppConfig) -> StreamTransport {
    match config.stream.transport {
        StreamTransportArg::Udp => StreamTransport::Udp {
            bind_address: config.stream.udp_bind,
            destination: config.stream.udp_destination,
        },
        StreamTransportArg::Tcp => StreamTransport::Tcp,
    }
}

fn selected_data(config: &AppConfig) -> BTreeSet<MocapData> {
    if config.stream.include.is_empty() {
        return BTreeSet::from([MocapData::RigidBodies]);
    }

    config.stream.include.iter().copied().collect()
}

fn stream_components(selection: &BTreeSet<MocapData>) -> Vec<ComponentSelection> {
    let mut components = Vec::new();
    if selection.contains(&MocapData::RigidBodies) {
        components.push(ComponentSelection::SixDResidual);
    }
    if selection.contains(&MocapData::LabeledMarkers) {
        components.push(ComponentSelection::ThreeDResidual);
    }
    if selection.contains(&MocapData::UnlabeledMarkers) {
        components.push(ComponentSelection::ThreeDNoLabelsResidual);
    }
    if selection.contains(&MocapData::Skeleton) {
        components.push(ComponentSelection::Skeleton { global: true });
    }
    components
}

fn rigid_body_names(parameters: &MocapParameters) -> Vec<String> {
    parameters
        .six_d
        .as_ref()
        .map(|six_d| {
            six_d
                .bodies
                .iter()
                .map(|body| body.name.clone())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default()
}

fn publish_rigid_body_pose_topics(
    session: &zenoh::Session,
    config: &AppConfig,
    frame: &AssembledFrame,
    selection: &BTreeSet<MocapData>,
    rigid_body_names: &[String],
) -> Result<Vec<(String, usize)>> {
    if config.zenoh.no_rigid_body_pose_topics || !selection.contains(&MocapData::RigidBodies) {
        return Ok(Vec::new());
    }

    let mut published = Vec::new();
    for body in rigid_bodies(frame)? {
        let topic = rigid_body_pose_topic(
            &config.zenoh.rigid_body_pose_topic_prefix,
            &body,
            rigid_body_names,
        );
        let payload = encode_compact_pose(&body);
        let payload_len = payload.len();
        session
            .put(topic.clone(), payload)
            .wait()
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
        published.push((topic, payload_len));
    }

    Ok(published)
}

fn rigid_body_pose_topic(
    prefix: &str,
    body: &MocapRigidBodySample,
    rigid_body_names: &[String],
) -> String {
    let name = usize::try_from(body.id().saturating_sub(1))
        .ok()
        .and_then(|index| rigid_body_names.get(index))
        .filter(|name| !name.is_empty())
        .map(|name| sanitize_keyexpr_segment(name))
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| format!("id_{}", body.id()));
    format!("{}/{name}/pose", prefix.trim_end_matches('/'))
}

fn sanitize_keyexpr_segment(input: &str) -> String {
    input
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || matches!(ch, '_' | '-' | '.') {
                ch
            } else {
                '_'
            }
        })
        .collect::<String>()
        .trim_matches('_')
        .to_owned()
}

fn qtm_mm_to_m(value: f32) -> f32 {
    value * QTM_MILLIMETERS_TO_METERS
}

fn qtm_position_m(x_mm: f32, y_mm: f32, z_mm: f32) -> Vec3f {
    Vec3f::new(qtm_mm_to_m(x_mm), qtm_mm_to_m(y_mm), qtm_mm_to_m(z_mm))
}

fn qtm_rigid_body_tracking_valid(
    drop_rate: i16,
    out_of_sync_rate: i16,
    x_mm: f32,
    y_mm: f32,
    z_mm: f32,
    rotation_matrix: [f32; 9],
) -> bool {
    drop_rate == 0
        && out_of_sync_rate == 0
        && [x_mm, y_mm, z_mm].into_iter().all(f32::is_finite)
        && rotation_matrix.into_iter().all(f32::is_finite)
}

fn encode_compact_pose(body: &MocapRigidBodySample) -> Vec<u8> {
    let position = body.position_enu_m();
    let attitude = body.attitude();
    let values = [
        position.x(),
        position.y(),
        position.z(),
        attitude.x(),
        attitude.y(),
        attitude.z(),
        attitude.w(),
    ];
    let mut payload = Vec::with_capacity(values.len() * size_of::<f32>());
    for value in values {
        payload.extend_from_slice(&value.to_le_bytes());
    }
    payload
}

fn rigid_body_statuses(
    frame: &AssembledFrame,
    rigid_body_names: &[String],
) -> Result<Vec<RigidBodyStatus>> {
    Ok(rigid_bodies(frame)?
        .into_iter()
        .map(|body| {
            let name = usize::try_from(body.id().saturating_sub(1))
                .ok()
                .and_then(|index| rigid_body_names.get(index))
                .filter(|name| !name.is_empty())
                .cloned()
                .unwrap_or_else(|| format!("id_{}", body.id()));
            let position = body.position_enu_m();
            let attitude = body.attitude();
            RigidBodyStatus {
                id: body.id(),
                name,
                position_m: [position.x(), position.y(), position.z()],
                quaternion: [attitude.x(), attitude.y(), attitude.z(), attitude.w()],
                residual: body.residual(),
                tracking_valid: body.tracking_valid(),
            }
        })
        .collect())
}

fn encode_mocap_frame(frame: &AssembledFrame, selection: &BTreeSet<MocapData>) -> Result<Vec<u8>> {
    let mut builder = FlatBufferBuilder::new();
    let rigid_bodies = if selection.contains(&MocapData::RigidBodies) {
        Some(builder.create_vector(&rigid_bodies(frame)?))
    } else {
        None
    };
    let labeled_markers = if selection.contains(&MocapData::LabeledMarkers) {
        Some(builder.create_vector(&labeled_markers(frame)?))
    } else {
        None
    };
    let unlabeled_markers = if selection.contains(&MocapData::UnlabeledMarkers) {
        Some(builder.create_vector(&unlabeled_markers(frame)?))
    } else {
        None
    };
    let skeleton_segments = if selection.contains(&MocapData::Skeleton) {
        Some(builder.create_vector(&skeleton_segments(frame)?))
    } else {
        None
    };

    let frame = MocapFrame::create(
        &mut builder,
        &MocapFrameArgs {
            timestamp_us: frame.timestamp,
            frame_number: frame.frame_number,
            labeled_markers,
            unlabeled_markers,
            rigid_bodies,
            skeleton_segments,
            ..MocapFrameArgs::default()
        },
    );
    builder.finish(frame, None);
    Ok(builder.finished_data().to_vec())
}

fn labeled_markers(frame: &AssembledFrame) -> Result<Vec<MocapMarkerSample>> {
    let component = frame
        .component(ComponentType::ThreeDResidual)
        .or_else(|| frame.component(ComponentType::ThreeD))
        .ok_or(BridgeError::MissingComponent("labeled marker"))?;

    match &component.data {
        ComponentData::ThreeDResidual(component) => Ok(component
            .markers
            .iter()
            .enumerate()
            .map(|(index, marker)| {
                MocapMarkerSample::new(
                    (index + 1) as i32,
                    &qtm_position_m(marker.x, marker.y, marker.z),
                    marker.residual,
                )
            })
            .collect()),
        ComponentData::ThreeD(component) => Ok(component
            .markers
            .iter()
            .enumerate()
            .map(|(index, marker)| {
                MocapMarkerSample::new(
                    (index + 1) as i32,
                    &qtm_position_m(marker.x, marker.y, marker.z),
                    0.0,
                )
            })
            .collect()),
        _ => Err(BridgeError::MissingComponent("labeled marker")),
    }
}

fn unlabeled_markers(frame: &AssembledFrame) -> Result<Vec<MocapMarkerSample>> {
    let component = frame
        .component(ComponentType::ThreeDNoLabelsResidual)
        .or_else(|| frame.component(ComponentType::ThreeDNoLabels))
        .ok_or(BridgeError::MissingComponent("unlabeled marker"))?;

    match &component.data {
        ComponentData::ThreeDNoLabelsResidual(component) => Ok(component
            .markers
            .iter()
            .map(|marker| {
                MocapMarkerSample::new(
                    marker.id,
                    &qtm_position_m(marker.x, marker.y, marker.z),
                    marker.residual,
                )
            })
            .collect()),
        ComponentData::ThreeDNoLabels(component) => Ok(component
            .markers
            .iter()
            .map(|marker| {
                MocapMarkerSample::new(
                    marker.id,
                    &qtm_position_m(marker.x, marker.y, marker.z),
                    0.0,
                )
            })
            .collect()),
        _ => Err(BridgeError::MissingComponent("unlabeled marker")),
    }
}

fn skeleton_segments(frame: &AssembledFrame) -> Result<Vec<MocapSegmentSample>> {
    let component = frame
        .component(ComponentType::Skeleton)
        .ok_or(BridgeError::MissingComponent("skeleton"))?;

    match &component.data {
        ComponentData::Skeleton(component) => Ok(component
            .skeletons
            .iter()
            .enumerate()
            .flat_map(|(skeleton_index, skeleton)| {
                skeleton.segments.iter().map(move |segment| {
                    MocapSegmentSample::new(
                        (skeleton_index + 1) as i32,
                        segment.id,
                        &qtm_position_m(segment.position.x, segment.position.y, segment.position.z),
                        &Quaternionf::new(
                            segment.rotation.x,
                            segment.rotation.y,
                            segment.rotation.z,
                            segment.rotation.w,
                        ),
                        true,
                    )
                })
            })
            .collect()),
        _ => Err(BridgeError::MissingComponent("skeleton")),
    }
}

fn encode_mocap_definition(
    parameters: &MocapParameters,
    selection: &BTreeSet<MocapData>,
) -> Result<Vec<u8>> {
    let mut builder = FlatBufferBuilder::new();
    let source = builder.create_string("qualisys");
    let frame_id = builder.create_string("qualisys");

    let rigid_bodies = if selection.contains(&MocapData::RigidBodies) {
        let definitions = parameters
            .six_d
            .as_ref()
            .map(|six_d| {
                six_d
                    .bodies
                    .iter()
                    .enumerate()
                    .map(|(index, body)| {
                        let name = builder.create_string(&body.name);
                        MocapRigidBodyDefinition::create(
                            &mut builder,
                            &MocapRigidBodyDefinitionArgs {
                                id: (index + 1) as i32,
                                name: Some(name),
                            },
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        Some(builder.create_vector(&definitions))
    } else {
        None
    };

    let labeled_markers = if selection.contains(&MocapData::LabeledMarkers) {
        let definitions = parameters
            .three_d
            .as_ref()
            .map(|three_d| {
                three_d
                    .labels
                    .iter()
                    .enumerate()
                    .map(|(index, label)| {
                        let name = builder.create_string(&label.name);
                        MocapMarkerDefinition::create(
                            &mut builder,
                            &MocapMarkerDefinitionArgs {
                                id: (index + 1) as i32,
                                name: Some(name),
                                color: label.rgb_color.unwrap_or(0),
                            },
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        Some(builder.create_vector(&definitions))
    } else {
        None
    };

    let skeleton_segments = if selection.contains(&MocapData::Skeleton) {
        let definitions = skeleton_segment_definitions(parameters, &mut builder);
        Some(builder.create_vector(&definitions))
    } else {
        None
    };

    let definition = MocapDefinition::create(
        &mut builder,
        &MocapDefinitionArgs {
            source: Some(source),
            frame_id: Some(frame_id),
            labeled_markers,
            rigid_bodies,
            skeleton_segments,
        },
    );
    builder.finish(definition, None);
    Ok(builder.finished_data().to_vec())
}

fn skeleton_segment_definitions<'a>(
    parameters: &MocapParameters,
    builder: &mut FlatBufferBuilder<'a>,
) -> Vec<flatbuffers::WIPOffset<MocapSegmentDefinition<'a>>> {
    let mut definitions = Vec::new();
    if let Some(skeletons) = &parameters.skeletons {
        for (skeleton_index, skeleton) in skeletons.skeletons.iter().enumerate() {
            append_skeleton_segment_definitions(
                builder,
                &mut definitions,
                (skeleton_index + 1) as i32,
                &skeleton.name,
                &skeleton.segments,
            );
        }
    }
    definitions
}

fn append_skeleton_segment_definitions<'a>(
    builder: &mut FlatBufferBuilder<'a>,
    definitions: &mut Vec<flatbuffers::WIPOffset<MocapSegmentDefinition<'a>>>,
    skeleton_id: i32,
    skeleton_name: &str,
    segments: &[MocapSkeletonSegment],
) {
    for (index, segment) in segments.iter().enumerate() {
        let skeleton_name_offset = builder.create_string(skeleton_name);
        let segment_name = builder.create_string(&segment.name);
        definitions.push(MocapSegmentDefinition::create(
            builder,
            &MocapSegmentDefinitionArgs {
                skeleton_id,
                skeleton_name: Some(skeleton_name_offset),
                segment_id: segment.id.unwrap_or((index + 1) as i32),
                segment_name: Some(segment_name),
            },
        ));
        append_skeleton_segment_definitions(
            builder,
            definitions,
            skeleton_id,
            skeleton_name,
            &segment.child_segments,
        );
    }
}

fn rigid_bodies(frame: &AssembledFrame) -> Result<Vec<MocapRigidBodySample>> {
    let component = frame
        .component(ComponentType::SixDResidual)
        .or_else(|| frame.component(ComponentType::SixD))
        .ok_or(BridgeError::MissingComponent("rigid body"))?;

    match &component.data {
        ComponentData::SixDResidual(component) => Ok(component
            .bodies
            .iter()
            .enumerate()
            .map(|(index, body)| {
                let tracking_valid = qtm_rigid_body_tracking_valid(
                    component.drop_rate,
                    component.out_of_sync_rate,
                    body.position.x,
                    body.position.y,
                    body.position.z,
                    body.rotation_matrix,
                );
                MocapRigidBodySample::new(
                    (index + 1) as i32,
                    &qtm_position_m(body.position.x, body.position.y, body.position.z),
                    &quat_from_rotation_matrix(body.rotation_matrix),
                    body.residual,
                    tracking_valid,
                )
            })
            .collect()),
        ComponentData::SixD(component) => Ok(component
            .bodies
            .iter()
            .enumerate()
            .map(|(index, body)| {
                let tracking_valid = qtm_rigid_body_tracking_valid(
                    component.drop_rate,
                    component.out_of_sync_rate,
                    body.position.x,
                    body.position.y,
                    body.position.z,
                    body.rotation_matrix,
                );
                MocapRigidBodySample::new(
                    (index + 1) as i32,
                    &qtm_position_m(body.position.x, body.position.y, body.position.z),
                    &quat_from_rotation_matrix(body.rotation_matrix),
                    0.0,
                    tracking_valid,
                )
            })
            .collect()),
        _ => Err(BridgeError::MissingComponent("rigid body")),
    }
}

fn quat_from_rotation_matrix(m: [f32; 9]) -> Quaternionf {
    let trace = m[0] + m[4] + m[8];
    let (x, y, z, w) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            (m[7] - m[5]) / s,
            (m[2] - m[6]) / s,
            (m[3] - m[1]) / s,
            0.25 * s,
        )
    } else if m[0] > m[4] && m[0] > m[8] {
        let s = (1.0 + m[0] - m[4] - m[8]).sqrt() * 2.0;
        (
            0.25 * s,
            (m[1] + m[3]) / s,
            (m[2] + m[6]) / s,
            (m[7] - m[5]) / s,
        )
    } else if m[4] > m[8] {
        let s = (1.0 + m[4] - m[0] - m[8]).sqrt() * 2.0;
        (
            (m[1] + m[3]) / s,
            0.25 * s,
            (m[5] + m[7]) / s,
            (m[2] - m[6]) / s,
        )
    } else {
        let s = (1.0 + m[8] - m[0] - m[4]).sqrt() * 2.0;
        (
            (m[2] + m[6]) / s,
            (m[5] + m[7]) / s,
            0.25 * s,
            (m[3] - m[1]) / s,
        )
    };

    Quaternionf::new(x, y, z, w)
}

fn run_web_server(app: AppHandle, bind: SocketAddr) -> Result<()> {
    let server = Server::http(bind).map_err(|error| BridgeError::Web(error.to_string()))?;
    for request in server.incoming_requests() {
        handle_request(app.clone(), request);
    }
    Ok(())
}

fn handle_request(app: AppHandle, mut request: Request) {
    let method = request.method().clone();
    let url = request.url().to_owned();
    let path = url.split_once('?').map(|(path, _)| path).unwrap_or(&url);

    let result = match (method, path) {
        (Method::Get, "/") => html_response(INDEX_HTML),
        (Method::Get, "/app.css") => text_response(APP_CSS, "text/css; charset=utf-8"),
        (Method::Get, "/app.js") => text_response(APP_JS, "text/javascript; charset=utf-8"),
        (Method::Get, "/api/status") => json_response(&app.snapshot()),
        (Method::Get, "/api/config") => json_response(&app.config()),
        (Method::Post, "/api/config") => {
            let body = read_request_body(&mut request);
            match body.and_then(|body| {
                serde_json::from_str::<AppConfig>(&body)
                    .map_err(|error| BridgeError::Config(error.to_string()))
            }) {
                Ok(config) => match app.replace_config(config) {
                    Ok(()) => json_response(&MessageResponse {
                        message: "Configuration saved".to_owned(),
                    }),
                    Err(error) => error_response(StatusCode(400), &error.to_string()),
                },
                Err(error) => error_response(StatusCode(400), &error.to_string()),
            }
        }
        (Method::Post, "/api/bridge/start") => {
            let started = app.start_bridge();
            json_response(&MessageResponse {
                message: if started {
                    "Bridge start requested".to_owned()
                } else {
                    "Bridge is already running".to_owned()
                },
            })
        }
        (Method::Post, "/api/bridge/stop") => {
            let stopped = app.stop_bridge();
            json_response(&MessageResponse {
                message: if stopped {
                    "Bridge stop requested".to_owned()
                } else {
                    "Bridge is not running".to_owned()
                },
            })
        }
        (Method::Post, "/api/bridge/restart") => {
            app.restart_bridge();
            json_response(&MessageResponse {
                message: "Bridge restart requested".to_owned(),
            })
        }
        (Method::Post, "/api/logs/clear") => match app.clear_logs() {
            Ok(()) => json_response(&MessageResponse {
                message: "Logs cleared".to_owned(),
            }),
            Err(error) => error_response(StatusCode(500), &error.to_string()),
        },
        (Method::Get, "/api/zenoh/discovery") => json_response(&app.discover_zenoh()),
        (Method::Post, "/api/diagnostics/qualisys") => {
            json_response(&app.run_qualisys_diagnostic())
        }
        (Method::Post, "/api/updates/check") => json_response(&app.check_update()),
        (Method::Post, "/api/updates/install") => json_response(&app.install_update()),
        _ => error_response(StatusCode(404), "Not found"),
    };

    if let Err(error) = request.respond(result) {
        app.log(
            "error",
            format!("Failed to respond to HTTP request: {error}"),
        );
    }
}

fn read_request_body(request: &mut Request) -> Result<String> {
    let mut body = String::new();
    request.as_reader().read_to_string(&mut body)?;
    Ok(body)
}

fn html_response(body: &str) -> Response<std::io::Cursor<Vec<u8>>> {
    text_response(body, "text/html; charset=utf-8")
}

fn json_response<T: Serialize>(body: &T) -> Response<std::io::Cursor<Vec<u8>>> {
    match serde_json::to_string(body) {
        Ok(json) => text_response(&json, "application/json; charset=utf-8"),
        Err(error) => error_response(StatusCode(500), &error.to_string()),
    }
}

fn error_response(status: StatusCode, message: &str) -> Response<std::io::Cursor<Vec<u8>>> {
    let body = serde_json::json!({
        "ok": false,
        "error": message,
    });
    Response::from_string(body.to_string())
        .with_status_code(status)
        .with_header(content_type("application/json; charset=utf-8"))
}

fn text_response(body: &str, content_type_value: &str) -> Response<std::io::Cursor<Vec<u8>>> {
    Response::from_string(body.to_owned()).with_header(content_type(content_type_value))
}

fn content_type(value: &str) -> Header {
    Header::from_bytes(&b"Content-Type"[..], value.as_bytes()).expect("valid content type header")
}

fn diagnose_qualisys(config: &AppConfig) -> Result<QualisysDiagnostic> {
    let started = Instant::now();
    let mut client = connect_qualisys(config)?;
    let parameters = client.get_mocap_parameters()?;
    let rigid_bodies = parameters
        .six_d
        .as_ref()
        .map(|six_d| six_d.bodies.len())
        .unwrap_or(0);
    let marker_labels = parameters
        .three_d
        .as_ref()
        .map(|three_d| three_d.labels.len())
        .unwrap_or(0);
    let skeletons = parameters
        .skeletons
        .as_ref()
        .map(|skeletons| skeletons.skeletons.len())
        .unwrap_or(0);

    Ok(QualisysDiagnostic {
        ok: true,
        host: config.qualisys.host.clone(),
        port: config.qualisys.port,
        round_trip_ms: started.elapsed().as_millis(),
        rigid_bodies,
        marker_labels,
        skeletons,
        error: None,
    })
}

fn discover_zenoh(app: &AppHandle) -> Result<ZenohDiscovery> {
    let config = app.config();
    let session = app.ensure_zenoh_session()?;
    let timeout = Duration::from_millis(config.zenoh.admin_query_timeout_ms);
    let mut errors = Vec::new();
    let subscribers = query_zenoh_admin(&session, "@/**/subscriber/**", timeout, &mut errors);
    let publishers = query_zenoh_admin(&session, "@/**/publisher/**", timeout, &mut errors);
    let queryables = query_zenoh_admin(&session, "@/**/queryable/**", timeout, &mut errors);
    Ok(ZenohDiscovery {
        ok: errors.is_empty(),
        subscribers,
        publishers,
        queryables,
        errors,
    })
}

fn query_zenoh_admin(
    session: &zenoh::Session,
    selector: &str,
    timeout: Duration,
    errors: &mut Vec<String>,
) -> Vec<ZenohAdminEntry> {
    let replies = match session
        .get(selector)
        .target(QueryTarget::All)
        .consolidation(ConsolidationMode::None)
        .timeout(timeout)
        .wait()
    {
        Ok(replies) => replies,
        Err(error) => {
            errors.push(format!("{selector}: {error}"));
            return Vec::new();
        }
    };

    let mut entries = Vec::new();
    let deadline = Instant::now() + timeout + Duration::from_millis(200);
    loop {
        match replies.recv_deadline(deadline) {
            Ok(Some(reply)) => match reply.result() {
                Ok(sample) => {
                    let payload = sample
                        .payload()
                        .try_to_string()
                        .ok()
                        .and_then(|text| serde_json::from_str::<serde_json::Value>(&text).ok());
                    entries.push(ZenohAdminEntry {
                        key_expr: sample.key_expr().to_string(),
                        payload,
                    });
                }
                Err(error) => errors.push(format!("{selector}: {error}")),
            },
            Ok(None) => break,
            Err(error) => {
                errors.push(format!("{selector}: {error}"));
                break;
            }
        }
    }
    entries
}

fn update_worker(app: AppHandle) {
    loop {
        let config = app.config();
        if config.updates.enabled {
            let status = app.check_update();
            if config.updates.auto_install && status.available {
                app.log(
                    "info",
                    format!(
                        "Auto-installing update {}",
                        status.latest_version.as_deref().unwrap_or("unknown")
                    ),
                );
                app.install_update();
            }
        }

        let minutes = app.config().updates.check_interval_minutes.max(15);
        thread::sleep(Duration::from_secs(minutes * 60));
    }
}

fn check_for_update(config: &UpdateConfig) -> Result<UpdateStatus> {
    if !config.enabled {
        return Ok(UpdateStatus {
            state: "disabled".to_owned(),
            checked_at_ms: Some(now_ms()),
            latest_version: None,
            current_version: env!("CARGO_PKG_VERSION").to_owned(),
            release_url: None,
            asset_name: None,
            asset_download_url: None,
            available: false,
            message: Some("Updates are disabled".to_owned()),
        });
    }

    let url = format!(
        "https://api.github.com/repos/{}/{}/releases/latest",
        config.github_owner, config.github_repo
    );
    let release: GitHubRelease = ureq::get(&url)
        .set("User-Agent", BIN_NAME)
        .call()
        .map_err(|error| BridgeError::Update(error.to_string()))?
        .into_json()
        .map_err(|error| BridgeError::Update(error.to_string()))?;

    let current = parse_version(env!("CARGO_PKG_VERSION"))?;
    let latest = parse_version(&release.tag_name)?;
    let available = latest > current;
    let installer = release
        .assets
        .iter()
        .find(|asset| asset.name.ends_with("windows-x86_64-setup.exe"))
        .or_else(|| {
            release
                .assets
                .iter()
                .find(|asset| asset.name.ends_with(".exe"))
        })
        .or_else(|| {
            release
                .assets
                .iter()
                .find(|asset| asset.name.contains("windows-x86_64"))
        });

    Ok(UpdateStatus {
        state: "checked".to_owned(),
        checked_at_ms: Some(now_ms()),
        latest_version: Some(release.tag_name),
        current_version: env!("CARGO_PKG_VERSION").to_owned(),
        release_url: Some(release.html_url),
        asset_name: installer.map(|asset| asset.name.clone()),
        asset_download_url: installer.map(|asset| asset.browser_download_url.clone()),
        available,
        message: Some(if available {
            "Update available".to_owned()
        } else {
            "You are running the latest version".to_owned()
        }),
    })
}

fn parse_version(input: &str) -> Result<Version> {
    Version::parse(input.trim_start_matches('v'))
        .map_err(|error| BridgeError::Update(format!("invalid version `{input}`: {error}")))
}

fn install_update_from_status(status: &UpdateStatus) -> Result<String> {
    if !status.available {
        return Ok("No update is available".to_owned());
    }
    let url = status
        .asset_download_url
        .as_ref()
        .ok_or_else(|| BridgeError::Update("No Windows installer asset was found".to_owned()))?;
    let name = status
        .asset_name
        .as_deref()
        .unwrap_or("synapse-qualisys-bridge-update.exe");
    let name = Path::new(name)
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("synapse-qualisys-bridge-update.exe");
    let target = std::env::temp_dir().join(format!("{BIN_NAME}-{}-{}", now_ms(), name));

    let mut response = ureq::get(url)
        .set("User-Agent", BIN_NAME)
        .call()
        .map_err(|error| BridgeError::Update(error.to_string()))?
        .into_reader();
    let mut file = fs::File::create(&target)?;
    std::io::copy(&mut response, &mut file)?;
    file.flush()?;
    drop(file);

    if cfg!(windows) {
        Command::new(&target)
            .args([
                "/VERYSILENT",
                "/NORESTART",
                "/CLOSEAPPLICATIONS",
                "/FORCECLOSEAPPLICATIONS",
                "/RESTARTAPPLICATIONS",
            ])
            .spawn()
            .map_err(|error| BridgeError::Update(error.to_string()))?;
        Ok(format!("Installer launched from {}", target.display()))
    } else {
        Ok(format!(
            "Downloaded update to {}; install manually on this platform",
            target.display()
        ))
    }
}

fn load_config_or_default(path: &Path) -> Result<AppConfig> {
    if path.exists() {
        let text = fs::read_to_string(path)?;
        toml::from_str(&text).map_err(|error| BridgeError::Config(error.to_string()))
    } else {
        Ok(AppConfig::default())
    }
}

fn ensure_config_file(path: &Path, config: &AppConfig) -> Result<()> {
    if !path.exists() {
        save_config(path, config)?;
    }
    Ok(())
}

fn save_config(path: &Path, config: &AppConfig) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let text =
        toml::to_string_pretty(config).map_err(|error| BridgeError::Config(error.to_string()))?;
    fs::write(path, text)?;
    Ok(())
}

fn resolve_config_path(cli_path: Option<&Path>) -> PathBuf {
    if let Some(path) = cli_path {
        return path.to_path_buf();
    }

    let local = PathBuf::from("bridge.toml");
    if local.exists() {
        return local;
    }

    platform_app_dir().join("bridge.toml")
}

fn resolve_log_path(config: &AppConfig) -> PathBuf {
    config
        .logging
        .log_file
        .clone()
        .unwrap_or_else(|| platform_app_dir().join("logs").join("bridge.log"))
}

fn platform_app_dir() -> PathBuf {
    if cfg!(windows) {
        if let Some(path) = std::env::var_os("LOCALAPPDATA") {
            return PathBuf::from(path).join("Synapse Qualisys Bridge");
        }
    }

    if let Some(path) = std::env::var_os("XDG_CONFIG_HOME") {
        return PathBuf::from(path).join("synapse-qualisys-bridge");
    }

    if let Some(home) = std::env::var_os("HOME") {
        return PathBuf::from(home)
            .join(".config")
            .join("synapse-qualisys-bridge");
    }

    PathBuf::from(".")
}

fn web_url(bind: SocketAddr) -> String {
    let host = if bind.ip().is_unspecified() {
        "127.0.0.1".to_owned()
    } else {
        bind.ip().to_string()
    };
    format!("http://{host}:{}", bind.port())
}

fn open_browser(url: &str) {
    let result = if cfg!(windows) {
        Command::new("cmd").args(["/C", "start", "", url]).spawn()
    } else if cfg!(target_os = "macos") {
        Command::new("open").arg(url).spawn()
    } else {
        Command::new("xdg-open").arg(url).spawn()
    };
    let _ = result;
}

fn sleep_stop_aware(stop: &AtomicBool, duration: Duration) {
    let started = Instant::now();
    while started.elapsed() < duration && !stop.load(Ordering::SeqCst) {
        thread::sleep(Duration::from_millis(100));
    }
}

fn now_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qtm_positions_are_converted_from_millimeters_to_meters() {
        let position = qtm_position_m(1000.0, -2500.0, 12.5);

        assert_eq!(position.x(), 1.0);
        assert_eq!(position.y(), -2.5);
        assert_eq!(position.z(), 0.0125);
    }

    #[test]
    fn rigid_body_tracking_valid_rejects_dropped_or_invalid_data() {
        let rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

        assert!(qtm_rigid_body_tracking_valid(0, 0, 1.0, 2.0, 3.0, rotation));
        assert!(!qtm_rigid_body_tracking_valid(
            1, 0, 1.0, 2.0, 3.0, rotation
        ));
        assert!(!qtm_rigid_body_tracking_valid(
            0, 1, 1.0, 2.0, 3.0, rotation
        ));
        assert!(!qtm_rigid_body_tracking_valid(
            0,
            0,
            f32::NAN,
            2.0,
            3.0,
            rotation
        ));

        let mut invalid_rotation = rotation;
        invalid_rotation[0] = f32::NAN;
        assert!(!qtm_rigid_body_tracking_valid(
            0,
            0,
            1.0,
            2.0,
            3.0,
            invalid_rotation
        ));
    }
}
