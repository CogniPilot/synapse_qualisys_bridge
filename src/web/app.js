const state = {
  config: null,
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
  setStatus("router", snapshot.router);

  $("framesMetric").textContent = fmtInt(snapshot.stats.frames);
  $("fpsMetric").textContent = fmtFloat(snapshot.stats.frames_per_second);
  $("messagesMetric").textContent = fmtInt(snapshot.stats.published_messages);
  $("bytesMetric").textContent = fmtBytes(snapshot.stats.published_bytes);

  renderTopics(snapshot.topics || []);
  renderRigidBodies(snapshot.rigid_bodies || []);
  renderLogs(snapshot.logs || []);

  const installButton = $("installUpdateButton");
  installButton.disabled = !snapshot.update.available;
  installButton.textContent = snapshot.update.available ? "Install Update" : "No Update";
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
    tbody.append(row(["No rigid bodies received", "", "", ""]));
    return;
  }
  for (const body of bodies) {
    tbody.append(
      row([
        body.name,
        body.id,
        body.position_m.map((v) => Number(v).toFixed(3)).join(", "),
        Number(body.residual).toFixed(3),
      ]),
    );
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

function fillList(id, entries) {
  const list = $(id);
  list.replaceChildren();
  if (!entries.length) {
    const item = document.createElement("li");
    item.textContent = "No entries returned";
    list.append(item);
    return;
  }
  for (const entry of entries) {
    const item = document.createElement("li");
    item.textContent = entry.key_expr;
    list.append(item);
  }
}

async function refreshStatus() {
  try {
    const snapshot = await request("/api/status");
    renderStatus(snapshot);
  } catch (error) {
    toast(error.message);
  }
}

async function loadConfig() {
  state.config = await request("/api/config");
  const cfg = state.config;
  $("qualisysHost").value = cfg.qualisys.host;
  $("qualisysPort").value = cfg.qualisys.port;
  $("qualisysTimeout").value = cfg.qualisys.timeout_ms;
  $("zenohConnect").value = cfg.zenoh.connect;
  $("zenohTopic").value = cfg.zenoh.topic;
  $("definitionTopic").value = cfg.zenoh.definition_topic;
  $("posePrefix").value = cfg.zenoh.rigid_body_pose_topic_prefix;
  $("routerCommand").value = cfg.zenoh.router_command || "";
  $("transport").value = cfg.stream.transport;
  $("udpBind").value = cfg.stream.udp_bind;

  for (const checkbox of document.querySelectorAll("input[name='include']")) {
    checkbox.checked = cfg.stream.include.includes(checkbox.value);
  }
}

function readConfigForm() {
  const cfg = structuredClone(state.config);
  cfg.qualisys.host = $("qualisysHost").value.trim();
  cfg.qualisys.port = Number($("qualisysPort").value);
  cfg.qualisys.timeout_ms = Number($("qualisysTimeout").value);
  cfg.zenoh.connect = $("zenohConnect").value.trim();
  cfg.zenoh.topic = $("zenohTopic").value.trim();
  cfg.zenoh.definition_topic = $("definitionTopic").value.trim();
  cfg.zenoh.rigid_body_pose_topic_prefix = $("posePrefix").value.trim();
  cfg.zenoh.router_command = $("routerCommand").value.trim() || null;
  cfg.stream.transport = $("transport").value;
  cfg.stream.udp_bind = $("udpBind").value.trim();
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

async function refreshDiscovery() {
  $("discoveryStatus").textContent = "Refreshing...";
  const discovery = await request("/api/zenoh/discovery");
  fillList("subscribersList", discovery.subscribers || []);
  fillList("publishersList", discovery.publishers || []);
  fillList("queryablesList", discovery.queryables || []);
  $("discoveryStatus").textContent = discovery.ok
    ? "Admin discovery OK"
    : (discovery.errors || []).join(" | ") || "Discovery unavailable";
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

function bindEvents() {
  $("saveConfigButton").addEventListener("click", () => saveConfig().catch((e) => toast(e.message)));
  $("openLogsButton").addEventListener("click", () => refreshStatus());
  $("checkUpdatesButton").addEventListener("click", () => checkUpdates().catch((e) => toast(e.message)));
  $("installUpdateButton").addEventListener("click", () => installUpdate().catch((e) => toast(e.message)));

  document.querySelector("[data-action='bridge-start']").addEventListener("click", () =>
    action("/api/bridge/start", "Bridge start requested").catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='bridge-stop']").addEventListener("click", () =>
    action("/api/bridge/stop", "Bridge stop requested").catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='bridge-restart']").addEventListener("click", () =>
    action("/api/bridge/restart", "Bridge restart requested").catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='router-start']").addEventListener("click", () =>
    action("/api/zenoh/router/start", "Zenoh start requested").catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='router-stop']").addEventListener("click", () =>
    action("/api/zenoh/router/stop", "Zenoh stop requested").catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='qualisys-diagnostic']").addEventListener("click", () =>
    diagnoseQualisys().catch((e) => toast(e.message)),
  );
  document.querySelector("[data-action='zenoh-discovery']").addEventListener("click", () =>
    refreshDiscovery().catch((e) => toast(e.message)),
  );
}

async function init() {
  bindEvents();
  await loadConfig();
  await refreshStatus();
  setInterval(refreshStatus, 1500);
}

init().catch((error) => toast(error.message));
