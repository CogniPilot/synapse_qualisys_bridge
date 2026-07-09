const { test, expect } = require("@playwright/test");
const fs = require("node:fs/promises");
const net = require("node:net");
const os = require("node:os");
const path = require("node:path");
const { spawn } = require("node:child_process");

test.describe.configure({ mode: "serial" });

let ctx;

test.beforeAll(async () => {
  ctx = await startStack();
});

test.afterAll(async () => {
  await ctx?.cleanup();
});

test("web GUI renders live QTM stream status", async ({ page }) => {
  await page.goto(ctx.webUrl);

  await expect(page.locator("#bridgeState")).toHaveText(/running|retrying/, {
    timeout: 15_000,
  });
  await expect(page.locator("#qualisysState")).toHaveText("connected", {
    timeout: 15_000,
  });
  await expect(page.locator("#zenohState")).toHaveText(/running|connected/, {
    timeout: 15_000,
  });

  await expect
    .poll(async () => metricNumber(await page.locator("#framesMetric").textContent()), {
      timeout: 15_000,
    })
    .toBeGreaterThan(0);
  await expect(page.locator("#rigidBodiesTable")).toContainText("sim_body_1", {
    timeout: 15_000,
  });
  await expect(page.locator("#topicsTable")).toContainText(
    "synapse/v1/topic/mocap_frame",
    { timeout: 15_000 },
  );
});

test("bridge publishes mocap and external odometry over real Zenoh", async () => {
  await waitForZenohSample("synapse/v1/topic/mocap_frame", 15_000);
  await waitForZenohSample("synapse/v1/topic/external_odometry/1", 20_000);

  await expect
    .poll(async () => {
      const status = await getJson(`${ctx.webUrl}/api/status`);
      return status.topics.map((topic) => topic.key_expr);
    }, { timeout: 10_000 })
    .toContain("synapse/v1/topic/external_odometry/1");
});

async function startStack() {
  const bridgeBin = requiredEnv("BRIDGE_BIN");
  const simBin = requiredEnv("QUALISYS_SIM_BIN");
  const tempDir = await fs.mkdtemp(path.join(os.tmpdir(), "sqb-e2e-"));
  const qtmPort = await freePort();
  const webPort = await freePort();
  const zenohPort = await freePort();
  const configPath = path.join(tempDir, "bridge.toml");
  const logPath = path.join(tempDir, "bridge.log");
  const webUrl = `http://127.0.0.1:${webPort}`;
  const zenohEndpoint = `tcp/127.0.0.1:${zenohPort}`;
  const children = [];

  await fs.writeFile(
    configPath,
    bridgeConfig({
      qtmPort,
      webPort,
      zenohPort,
      logPath,
    }),
  );

  children.push(
    spawnLogged(
      simBin,
      [
        "--bind",
        `127.0.0.1:${qtmPort}`,
        "--hz",
        "240",
        "--rigid-bodies",
        "2",
      ],
      "qualisys-sim",
    ),
  );
  await waitForTcp("127.0.0.1", qtmPort, 10_000);

  children.push(spawnLogged(bridgeBin, ["--config", configPath], "bridge"));
  await waitForHttp(`${webUrl}/api/status`, 15_000);
  await waitForBridgeFrames(webUrl, 20_000);

  return {
    tempDir,
    webUrl,
    zenohEndpoint,
    async cleanup() {
      for (const child of children.reverse()) {
        await terminate(child);
      }
      await fs.rm(tempDir, { recursive: true, force: true });
    },
  };
}

function bridgeConfig({ qtmPort, webPort, zenohPort, logPath }) {
  const escapedLogPath = logPath.replaceAll("\\", "\\\\").replaceAll('"', '\\"');
  return `
[bridge]
autostart = true
reconnect_delay_ms = 200

[qualisys]
host = "127.0.0.1"
port = ${qtmPort}
timeout_ms = 1000

[stream]
include = ["rigid-bodies"]
transport = "udp"
udp_bind = "127.0.0.1:0"
velocity_max_gap_ms = 100
velocity_min_dt_ms = 1
position_stddev_m = 0.002
attitude_stddev_rad = 0.01
linear_velocity_stddev_m_s = 0.05
angular_velocity_stddev_rad_s = 0.10

[zenoh]
mode = "router"
connect = ""
listen = "tcp/127.0.0.1:${zenohPort}"
topic = "synapse/v1/topic/mocap_frame"
external_odometry_topic_prefix = "synapse/v1/topic/external_odometry"
no_external_odometry = false
admin_query_timeout_ms = 200

[web]
enabled = true
bind = "127.0.0.1:${webPort}"
open_browser = false

[logging]
log_file = "${escapedLogPath}"

[updates]
enabled = false
auto_check = false
auto_install = false
check_interval_minutes = 360
github_owner = "CogniPilot"
github_repo = "synapse_qualisys_bridge"
`;
}

async function waitForBridgeFrames(webUrl, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const status = await getJson(`${webUrl}/api/status`);
    if (
      status.bridge.state === "running" &&
      status.qualisys.state === "connected" &&
      status.stats.frames > 0 &&
      status.rigid_bodies.length > 0
    ) {
      return;
    }
    await delay(250);
  }
  throw new Error("bridge did not report live QTM frames");
}

async function waitForZenohSample(key, timeoutMs) {
  const child = spawnLogged(
    "z_sub",
    [
      "-m",
      "client",
      "-e",
      ctx.zenohEndpoint,
      "-k",
      key,
      "--no-multicast-scouting",
    ],
    `z_sub:${key}`,
  );

  try {
    await waitForOutput(child, (output) => hasZenohReceive(output, key), timeoutMs);
  } finally {
    await terminate(child);
  }
}

function waitForOutput(child, predicate, timeoutMs) {
  return new Promise((resolve, reject) => {
    let output = "";
    const timer = setTimeout(() => {
      reject(new Error(`timed out waiting for Zenoh sample; output: ${output}`));
    }, timeoutMs);

    const onData = (chunk) => {
      output += chunk.toString("utf8");
      if (predicate(output)) {
        clearTimeout(timer);
        resolve();
      }
    };
    child.stdout.on("data", onData);
    child.stderr.on("data", onData);
    child.once("exit", (code, signal) => {
      clearTimeout(timer);
      reject(
        new Error(
          `subscriber exited before Zenoh sample; code=${code} signal=${signal}; output: ${output}`,
        ),
      );
    });
  });
}

function hasZenohReceive(output, key) {
  return output
    .split(/\r?\n/)
    .some((line) => line.includes("Received") && line.includes(`'${key}'`));
}

function metricNumber(text) {
  return Number((text || "").replaceAll(",", ""));
}

async function waitForHttp(url, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      const response = await fetch(url);
      if (response.ok) return;
    } catch (_) {
      // Retry until the process binds the web socket.
    }
    await delay(250);
  }
  throw new Error(`HTTP endpoint did not become ready: ${url}`);
}

async function waitForTcp(host, port, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      await new Promise((resolve, reject) => {
        const socket = net.createConnection({ host, port }, () => {
          socket.end();
          resolve();
        });
        socket.once("error", reject);
        socket.setTimeout(500, () => {
          socket.destroy();
          reject(new Error("TCP connect timed out"));
        });
      });
      return;
    } catch (_) {
      await delay(100);
    }
  }
  throw new Error(`TCP endpoint did not become ready: ${host}:${port}`);
}

async function getJson(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`GET ${url} failed: ${response.status}`);
  }
  return response.json();
}

function spawnLogged(command, args, label) {
  const child = spawn(command, args, {
    stdio: ["ignore", "pipe", "pipe"],
    env: {
      ...process.env,
      RUST_BACKTRACE: "1",
    },
  });
  child.stdout.on("data", (chunk) =>
    process.stdout.write(`[${label}] ${chunk}`),
  );
  child.stderr.on("data", (chunk) =>
    process.stderr.write(`[${label}] ${chunk}`),
  );
  child.once("exit", (code, signal) => {
    if (code !== 0 && signal !== "SIGTERM" && signal !== "SIGKILL") {
      process.stderr.write(`[${label}] exited code=${code} signal=${signal}\n`);
    }
  });
  return child;
}

async function terminate(child) {
  if (!child || child.exitCode !== null || child.signalCode !== null) return;
  child.kill("SIGTERM");
  const exited = await Promise.race([
    new Promise((resolve) => child.once("exit", resolve)),
    delay(2_000).then(() => false),
  ]);
  if (exited === false) {
    child.kill("SIGKILL");
    await new Promise((resolve) => child.once("exit", resolve));
  }
}

function requiredEnv(name) {
  const value = process.env[name];
  if (!value) throw new Error(`${name} is required`);
  return value;
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function freePort() {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.listen(0, "127.0.0.1", () => {
      const { port } = server.address();
      server.close(() => resolve(port));
    });
    server.once("error", reject);
  });
}
