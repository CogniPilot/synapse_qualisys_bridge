# synapse_qualisys_bridge

Rust bridge from Qualisys QTM RT motion-capture frames to Synapse FlatBuffers on
Zenoh.

## Web interface

The bridge runs a local web dashboard by default:

```sh
cargo run --bin synapse-qualisys-bridge -- --open
```

Open <http://127.0.0.1:8787> to start/stop the bridge, edit the main Qualisys
and Zenoh settings, run a Qualisys connection diagnostic, inspect published
topic counters, and view recent logs.

The bridge runs Zenoh in embedded `router` mode by default, so Windows users do
not need to install `zenohd`. Fresh installs listen on both
`udp/0.0.0.0:7447` and `tcp/0.0.0.0:7447`; other computers on the network can
subscribe through `udp/<bridge-pc-ip>:7447` or `tcp/<bridge-pc-ip>:7447`. UDP is
listed first because it is the preferred low-latency transport for this bridge,
while TCP remains available as a compatibility fallback.

To join this bridge to another Zenoh router, set `zenoh.connect` to one or more
upstream endpoints. Set `zenoh.mode = "client"` in `bridge.toml` to publish
only through an external Zenoh router instead.

Run against a real QTM host:

```sh
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 192.168.1.10
```

Run against the SDK simulator:

```sh
cargo run --manifest-path ../qualisys_rust_sdk/Cargo.toml --bin qualisys-sim
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 127.0.0.1
```

Useful defaults can also come from environment variables:

```sh
QUALISYS_HOST=127.0.0.1 cargo run --bin synapse-qualisys-bridge
```

Topic keys follow the synapse_fbs 0.6 topic catalog: raw mocap frames publish
on `qualisys/mocap` by default, and per-rigid-body external odometry publishes
on `<body_name>/external_pose` in each tracked vehicle's own namespace. Run
`cargo run --bin synapse-qualisys-bridge -- --help` for all options.

Example subscriber configuration from another machine:

```text
endpoint: udp/<bridge-pc-ip>:7447
key expression: qualisys/mocap        # raw frames
key expression: <body_name>/external_pose   # estimator input for one vehicle
```

Every published Zenoh value carries the synapse_fbs 0.6 value contract
encoding
(`<media-type>;type=<wire-type>;schema=sha256-128:<hash>`), so catalog-aware
consumers can validate payloads before decoding.

`zenoh.listen` and `zenoh.connect` can each contain multiple endpoints separated
by commas, semicolons, or whitespace.

Write a default configuration file:

```sh
cargo run --bin synapse-qualisys-bridge -- --write-default-config bridge.toml
```

Run from a configuration file:

```sh
synapse-qualisys-bridge --config bridge.toml --open
```

By default, the bridge requests rigid-body pose from the QTM RT stream and
publishes it continuously on Zenoh. Add high-rate QTM components explicitly
when bandwidth allows:

```sh
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 192.168.1.10 \
  --include rigid-bodies,labeled-markers
```

Supported `--include` values are `rigid-bodies`, `labeled-markers`, and
`unlabeled-markers`. These options select which components the bridge requests
from Qualisys; Zenoh subscribers consume the already-published topics and do not
request QTM components on demand.

QTM reports mocap positions in millimeters. The bridge converts all Synapse
positions to meters before publishing. The raw `synapse.topic.MocapFrame`
FlatBuffer preserves source-like marker and 6D rigid-body samples for logging
and tooling. Rigid-body samples are invalid only when the pose values are not
finite. A nonzero QTM dropped or out-of-sync rate is retained as degraded
quality, so a finite pose remains available for visualization while derived
odometry reports `Degraded` rather than silently appearing lost.

The bridge publishes a small JSON ID/name mapping at startup and every five
seconds on `qualisys/rigid_body_names` (configurable with
`zenoh.rigid_body_names_topic`). This lets visualizers select a body by its QTM
name instead of assuming a numeric ID:

```json
{"source":"qualisys","rigidBodies":[{"id":1,"name":"cub1"}]}
```

The bridge also publishes fixed-layout per-rigid-body external odometry payloads
by default:

```text
[<prefix>/]<body_name>/external_pose
```

`<body_name>` is the QTM rigid-body name sanitized to a lowercase snake_case
Zenoh key segment (`Cub 1` becomes `cub_1`), with a `body_<id>` fallback for
empty or unusable names, matching the synapse_fbs convention that bridges
write estimator inputs into the consuming vehicle's own namespace. Use
`--external-odometry-topic-prefix` to prepend a deployment namespace (for
example `field_lab`), or `--no-external-odometry` to disable these topics.

Each payload is the 64-byte `synapse.topic.ExternalOdometryData` struct:
timestamp, position in ENU meters, attitude as body-FLU to ENU quaternion,
linear velocity in ENU meters per second, and angular velocity in body-FLU
radians per second. The estimator runs for every tracked body so the
dashboard's External Odometry panel can display the live Kalman filter state
(position, attitude, velocities, and the full 12x12 covariance); the Zenoh
publication stays demand-driven, only sending a body's payloads while Zenoh
reports a matching subscriber for its key expression. The estimator is a 12D
Lie/error-state EKF prediction model generated from the
`Estimation.Examples.MocapExternalOdometryIEKF` Modelica example in the pinned
`third_party/modelica_models` submodule. It estimates position, attitude,
linear velocity, and angular velocity without a centered least-squares fit that
would add lookahead latency. State resets across dropped, out-of-sync, invalid,
or too-sparse samples.

Configurations saved by bridge versions before the synapse_fbs 0.6 update keep
working: the legacy `synapse/v1/topic/...` default key expressions are migrated
to the catalog keys on startup, while custom key expressions are left as-is.

Regenerate the estimator kernel after updating Rumoca or `modelica_models`:

```sh
git submodule update --init third_party/modelica_models
rumoca compile \
  third_party/modelica_models/Estimation/Examples/MocapExternalOdometryIEKF.mo \
  --model Estimation.Examples.MocapExternalOdometryIEKF \
  --source-root third_party/modelica_models \
  --target rust-fixed-solve \
  --output /tmp/synapse_qualisys_bridge_codegen
cp /tmp/synapse_qualisys_bridge_codegen/Estimation_Examples_MocapExternalOdometryIEKF_fixed_solve.rs \
  src/estimator/generated/MocapExternalOdometryEstimator_solve.rs
```

## Release binaries and schema compatibility

Pushing a `v*` tag builds release archives for Linux, macOS, and Windows and
uploads them to a GitHub release. The Windows release also includes an Inno
Setup installer named `synapse-qualisys-bridge-<version>-windows-x86_64-setup.exe`.
The Linux archive is built for
`x86_64-unknown-linux-musl` to avoid a glibc runtime dependency. The Windows
archive statically links the MSVC C runtime. The macOS archive contains a
universal binary for Intel and Apple Silicon Macs.

Each archive contains `synapse-qualisys-bridge`, `README.md`, `LICENSE`, and
`SYNAPSE_FBS_COMPATIBILITY.txt`. The Windows archive also contains a default
`bridge.toml`. The workflow publishes a `.sha256` checksum for every archive and
installer.

On Windows, the installer is per-user and creates Start Menu entries for:

- launching the bridge and opening the dashboard
- opening the dashboard
- opening the log directory
- editing `bridge.toml`

The Windows binary is a GUI-subsystem executable, so launching it never shows
a console window; log output goes to the dashboard and the log file (running
it from a terminal still prints there). When the dashboard opens, the bridge
launches it as a dedicated Edge or Chrome app window, and closing that window
shuts the bridge down. If neither browser is found the dashboard opens in the
default browser without that lifecycle coupling.

It can optionally add a startup shortcut so the bridge starts when the user
signs in. The installed dashboard checks GitHub releases for newer installer
assets and can launch the installer to update itself.

The release workflow reads the locked `synapse_fbs` crate version from
`Cargo.lock`, adds it to the GitHub release notes, and includes it in each
archive. Consumers should decode Synapse FlatBuffers payloads with the
documented `synapse_fbs` version for that release.
