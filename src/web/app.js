const state = {
  config: null,
  snapshot: null,
  toastTimer: null,
};

const $ = (id) => document.getElementById(id);

function toast(message) {
  const el = $("toast");
  el.textContent = message;
  el.classList.add("visible");
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => el.classList.remove("visible"), 3600);
}

async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const text = await response.text();
  const data = text ? JSON.parse(text) : {};
  if (!response.ok) {
    throw new Error(data.error || response.statusText);
  }
  return data;
}

async function post(path, body) {
  return request(path, {
    method: "POST",
    body: body === undefined ? "" : JSON.stringify(body),
  });
}

function setStatus(prefix, status) {
  const stateEl = $(`${prefix}State`);
  const detailEl = $(`${prefix}Detail`);
  stateEl.textContent = status.state;
  stateEl.className = `state-${status.state}`;
  detailEl.textContent = status.last_error || status.detail || "";
}

function fmtInt(value) {
  return Number(value || 0).toLocaleString();
}

function fmtFloat(value) {
  return Number(value || 0).toFixed(1);
}

function fmtBytes(value) {
  const bytes = Number(value || 0);
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
  return `${(bytes / 1024 / 1024).toFixed(1)} MiB`;
}

function fmtTime(ms) {
  if (!ms) return "";
  return new Date(Number(ms)).toLocaleTimeString();
}

function renderStatus(snapshot) {
  $("versionLine").textContent = `${snapshot.version}  |  ${snapshot.web_url}`;
  $("pathsLine").textContent = `${snapshot.config_path}  |  ${snapshot.log_path}`;

  setStatus("bridge", snapshot.bridge);
  setStatus("qualisys", snapshot.qualisys);
  setStatus("zenoh", snapshot.zenoh);

  $("framesMetric").textContent = fmtInt(snapshot.stats.frames);
  $("fpsMetric").textContent = fmtFloat(snapshot.stats.frames_per_second);
  $("messagesMetric").textContent = fmtInt(snapshot.stats.published_messages);
  $("bytesMetric").textContent = fmtBytes(snapshot.stats.published_bytes);

  renderNetworkSubscribe(snapshot.zenoh_access);
  renderTopics(snapshot.topics || []);
  renderRigidBodies(snapshot.rigid_bodies || []);
  renderOdometry(snapshot.odometry || []);
  renderLogs(snapshot.logs || []);
  updateBridgeToggle(snapshot.bridge.state);

  const installButton = $("installUpdateButton");
  installButton.disabled = !snapshot.update.available;
  installButton.textContent = snapshot.update.available ? "Install Update" : "No Update";
}

function renderNetworkSubscribe(access) {
  $("networkMode").textContent = access ? access.mode : "";
  const endpoints = access?.subscriber_endpoints || [];
  $("subscriberEndpoint").textContent = endpoints.length
    ? endpoints.join(", ")
    : "Set a listen or connect endpoint";
  $("subscriberKeyExpr").textContent = access?.sample_key_expr || "-";
  $("subscriberArgs").textContent = access?.example_subscriber_args || "-";
}

function renderTopics(topics) {
  const tbody = $("topicsTable");
  tbody.replaceChildren();
  if (!topics.length) {
    tbody.append(row(["No published topics yet", "", "", ""]));
    return;
  }
  for (const topic of topics) {
    tbody.append(
      row([
        topic.key_expr,
        topic.role,
        fmtInt(topic.messages),
        fmtBytes(topic.bytes),
      ]),
    );
  }
}

function renderRigidBodies(bodies) {
  const tbody = $("rigidBodiesTable");
  tbody.replaceChildren();
  if (!bodies.length) {
    tbody.append(row(["No rigid bodies received", "", "", "", ""]));
    return;
  }
  for (const body of bodies) {
    tbody.append(
      row([
        body.name,
        body.id,
        body.tracking_valid ? "true" : "false",
        body.position_m.map((v) => Number(v).toFixed(3)).join(", "),
        Number(body.residual).toFixed(3),
      ]),
    );
  }
}

// Tangent-state block offsets in the 12x12 odometry covariance matrix.
const COV_LABELS = ["rx", "ry", "rz", "vx", "vy", "vz", "px", "py", "pz", "wx", "wy", "wz"];
const COV_ATTITUDE = 0;
const COV_VELOCITY = 3;
const COV_POSITION = 6;
const COV_ANGULAR = 9;

// Keys of currently expanded odometry tree nodes, so re-rendering on each
// status poll does not collapse what the user opened.
const openNodes = new Set();

function collapsible(key, summaryChildren, bodyChildren) {
  const details = document.createElement("details");
  details.dataset.key = key;
  if (openNodes.has(key)) details.open = true;
  details.addEventListener("toggle", () => {
    if (details.open) openNodes.add(key);
    else openNodes.delete(key);
  });
  const summary = document.createElement("summary");
  summary.append(...summaryChildren);
  details.append(summary, ...bodyChildren);
  return details;
}

function fmtValue(value) {
  const v = Number(value);
  if (!Number.isFinite(v)) return String(value);
  if (v === 0) return "0";
  const abs = Math.abs(v);
  if (abs >= 1e4 || abs < 1e-3) return v.toExponential(3);
  return v.toFixed(4);
}

function covSigma(covariance, index) {
  const variance = Number(covariance?.[index]?.[index]);
  if (!Number.isFinite(variance) || variance < 0) return null;
  return Math.sqrt(variance);
}

function quatToEulerDeg([w, x, y, z]) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const deg = 180 / Math.PI;
  return [roll * deg, pitch * deg, yaw * deg];
}

function odometryStatusClass(status) {
  if (status === "Filtered") return "ok";
  if (status === "Lost" || status === "Unknown") return "bad";
  return "warn";
}

function valueTable(rows) {
  const table = document.createElement("table");
  table.className = "odom-table";
  for (const [label, value] of rows) {
    const tr = document.createElement("tr");
    const labelTd = document.createElement("td");
    labelTd.className = "odom-label";
    labelTd.textContent = label;
    const valueTd = document.createElement("td");
    valueTd.textContent = value;
    tr.append(labelTd, valueTd);
    table.append(tr);
  }
  return table;
}

function vectorGroup(key, title, values, covariance, covOffset) {
  const axes = ["x", "y", "z"];
  const rows = axes.map((axis, index) => {
    const sigma = covSigma(covariance, covOffset + index);
    const value = fmtValue(values?.[index]);
    return [axis, sigma === null ? value : `${value} ± ${fmtValue(sigma)}`];
  });
  const summary = document.createElement("span");
  summary.textContent = title;
  return collapsible(key, [summary], [valueTable(rows)]);
}

function attitudeGroup(key, quaternion, covariance) {
  const [roll, pitch, yaw] = quatToEulerDeg((quaternion || []).map(Number));
  const rows = [
    ["w", fmtValue(quaternion?.[0])],
    ["x", fmtValue(quaternion?.[1])],
    ["y", fmtValue(quaternion?.[2])],
    ["z", fmtValue(quaternion?.[3])],
    ["roll °", fmtValue(roll)],
    ["pitch °", fmtValue(pitch)],
    ["yaw °", fmtValue(yaw)],
  ];
  for (const [index, axis] of ["rx", "ry", "rz"].entries()) {
    const sigma = covSigma(covariance, COV_ATTITUDE + index);
    if (sigma !== null) rows.push([`σ ${axis} rad`, fmtValue(sigma)]);
  }
  const summary = document.createElement("span");
  summary.textContent = "Attitude (quaternion w x y z)";
  return collapsible(key, [summary], [valueTable(rows)]);
}

function covarianceGroup(key, covariance) {
  const wrap = document.createElement("div");
  wrap.className = "table-wrap cov-wrap";
  const table = document.createElement("table");
  table.className = "cov-table";

  const header = document.createElement("tr");
  header.append(document.createElement("th"));
  for (const label of COV_LABELS) {
    const th = document.createElement("th");
    th.textContent = label;
    header.append(th);
  }
  table.append(header);

  for (const [rowIndex, label] of COV_LABELS.entries()) {
    const tr = document.createElement("tr");
    const th = document.createElement("th");
    th.textContent = label;
    tr.append(th);
    for (let colIndex = 0; colIndex < COV_LABELS.length; colIndex += 1) {
      const td = document.createElement("td");
      const value = Number(covariance?.[rowIndex]?.[colIndex]);
      td.textContent = Number.isFinite(value) ? value.toExponential(2) : "-";
      if (rowIndex === colIndex) td.className = "cov-diagonal";
      tr.append(td);
    }
    table.append(tr);
  }
  wrap.append(table);

  const summary = document.createElement("span");
  summary.textContent = "Covariance (12×12 tangent: rotation, velocity, position, angular rate)";
  return collapsible(key, [summary], [wrap]);
}

function odometryNode(body) {
  const name = document.createElement("strong");
  name.textContent = `${body.name} (id ${body.id})`;
  const status = document.createElement("span");
  status.className = `odom-status ${odometryStatusClass(body.status)}`;
  status.textContent = body.status;
  const badge = document.createElement("span");
  badge.className = "odom-badge";
  badge.textContent = body.subscribed ? "publishing" : "no subscriber";
  const topic = document.createElement("code");
  topic.textContent = body.topic;

  const meta = document.createElement("p");
  meta.className = "odom-meta";
  const flags = (body.flags || []).join(", ") || "none";
  meta.textContent = `t = ${fmtInt(body.timestamp_us)} µs · flags: ${flags}`;

  const key = `odom:${body.id}`;
  const covariance = body.covariance || [];
  return collapsible(
    key,
    [name, status, badge, topic],
    [
      meta,
      vectorGroup(`${key}:pos`, "Position (ENU m)", body.position_enu_m, covariance, COV_POSITION),
      attitudeGroup(`${key}:att`, body.attitude_wxyz, covariance),
      vectorGroup(
        `${key}:vel`,
        "Linear velocity (ENU m/s)",
        body.linear_velocity_enu_m_s,
        covariance,
        COV_VELOCITY,
      ),
      vectorGroup(
        `${key}:angvel`,
        "Angular velocity (FLU rad/s)",
        body.angular_velocity_flu_rad_s,
        covariance,
        COV_ANGULAR,
      ),
      covarianceGroup(`${key}:cov`, covariance),
    ],
  );
}

function renderOdometry(odometry) {
  const container = $("odometryList");
  container.replaceChildren();
  if (!odometry.length) {
    const empty = document.createElement("p");
    empty.className = "subtle odom-empty";
    empty.textContent = "No external odometry estimates yet";
    container.append(empty);
    return;
  }
  for (const body of odometry) {
    container.append(odometryNode(body));
  }
}

function renderLogs(logs) {
  const list = $("logsList");
  list.replaceChildren();
  for (const entry of logs.slice(-120).reverse()) {
    const li = document.createElement("li");
    const time = document.createElement("span");
    const level = document.createElement("strong");
    const message = document.createElement("span");
    time.textContent = fmtTime(entry.time_ms);
    level.textContent = entry.level.toUpperCase();
    level.className = `level-${entry.level}`;
    message.textContent = entry.message;
    li.append(time, level, message);
    list.append(li);
  }
}

function row(values) {
  const tr = document.createElement("tr");
  for (const value of values) {
    const td = document.createElement("td");
    td.textContent = value;
    tr.append(td);
  }
  return tr;
}

async function refreshStatus() {
  try {
    const snapshot = await request("/api/status");
    state.snapshot = snapshot;
    renderStatus(snapshot);
  } catch (error) {
    toast(error.message);
  }
}

function updateBridgeToggle(bridgeState) {
  const button = $("bridgeToggleButton");
  const running = ["running", "starting", "retrying", "stopping"].includes(bridgeState);
  button.dataset.nextAction = running ? "stop" : "start";
  button.textContent = running ? "Stop Bridge" : "Start Bridge";
}

async function loadConfig() {
  state.config = await request("/api/config");
  const cfg = state.config;
  $("qualisysHost").value = cfg.qualisys.host;
  $("qualisysPort").value = cfg.qualisys.port;
  $("qualisysTimeout").value = cfg.qualisys.timeout_ms;
  $("zenohMode").value = cfg.zenoh.mode;
  $("zenohListen").value = cfg.zenoh.listen;
  $("zenohConnect").value = cfg.zenoh.connect;
  $("zenohTopic").value = cfg.zenoh.topic;
  $("externalOdometryPrefix").value = cfg.zenoh.external_odometry_topic_prefix;
  $("publishExternalOdometry").checked = !cfg.zenoh.no_external_odometry;
  $("transport").value = cfg.stream.transport;
  $("udpBind").value = cfg.stream.udp_bind;
  $("velocityMaxGap").value = cfg.stream.velocity_max_gap_ms;
  $("velocityMinDt").value = cfg.stream.velocity_min_dt_ms;

  for (const checkbox of document.querySelectorAll("input[name='include']")) {
    checkbox.checked = cfg.stream.include.includes(checkbox.value);
  }
}

function readConfigForm() {
  const cfg = structuredClone(state.config);
  cfg.qualisys.host = $("qualisysHost").value.trim();
  cfg.qualisys.port = Number($("qualisysPort").value);
  cfg.qualisys.timeout_ms = Number($("qualisysTimeout").value);
  cfg.zenoh.mode = $("zenohMode").value;
  cfg.zenoh.listen = $("zenohListen").value.trim();
  cfg.zenoh.connect = $("zenohConnect").value.trim();
  cfg.zenoh.topic = $("zenohTopic").value.trim();
  cfg.zenoh.external_odometry_topic_prefix = $("externalOdometryPrefix").value.trim();
  cfg.zenoh.no_external_odometry = !$("publishExternalOdometry").checked;
  cfg.stream.transport = $("transport").value;
  cfg.stream.udp_bind = $("udpBind").value.trim();
  cfg.stream.velocity_max_gap_ms = Number($("velocityMaxGap").value);
  cfg.stream.velocity_min_dt_ms = Number($("velocityMinDt").value);
  cfg.stream.include = Array.from(document.querySelectorAll("input[name='include']:checked")).map(
    (checkbox) => checkbox.value,
  );
  if (!cfg.stream.include.length) {
    cfg.stream.include = ["rigid-bodies"];
  }
  return cfg;
}

async function saveConfig() {
  const cfg = readConfigForm();
  await post("/api/config", cfg);
  state.config = cfg;
  toast("Configuration saved");
  await refreshStatus();
}

async function action(path, message) {
  const result = await post(path);
  toast(result.message || message);
  await refreshStatus();
}

async function diagnoseQualisys() {
  const result = await post("/api/diagnostics/qualisys");
  toast(result.ok ? `Qualisys OK in ${result.round_trip_ms} ms` : result.error);
  await refreshStatus();
}

async function checkUpdates() {
  const update = await post("/api/updates/check");
  toast(update.message || "Update check complete");
  await refreshStatus();
}

async function installUpdate() {
  const update = await post("/api/updates/install");
  toast(update.message || "Update installer launched");
  await refreshStatus();
}

async function clearLogs() {
  const result = await post("/api/logs/clear");
  toast(result.message || "Logs cleared");
  await refreshStatus();
}

function bindEvents() {
  $("saveConfigButton").addEventListener("click", () => saveConfig().catch((e) => toast(e.message)));
  $("openLogsButton").addEventListener("click", () => refreshStatus());
  $("clearLogsButton").addEventListener("click", () => clearLogs().catch((e) => toast(e.message)));
  $("checkUpdatesButton").addEventListener("click", () => checkUpdates().catch((e) => toast(e.message)));
  $("installUpdateButton").addEventListener("click", () => installUpdate().catch((e) => toast(e.message)));

  $("bridgeToggleButton").addEventListener("click", () => {
    const nextAction = $("bridgeToggleButton").dataset.nextAction || "start";
    const path = nextAction === "stop" ? "/api/bridge/stop" : "/api/bridge/start";
    const message = nextAction === "stop" ? "Bridge stop requested" : "Bridge start requested";
    action(path, message).catch((e) => toast(e.message));
  });

  document.querySelector("[data-action='qualisys-diagnostic']").addEventListener("click", () =>
    diagnoseQualisys().catch((e) => toast(e.message)),
  );
}

async function init() {
  bindEvents();
  await loadConfig();
  await refreshStatus();
  setInterval(refreshStatus, 1500);
}

init().catch((error) => toast(error.message));
