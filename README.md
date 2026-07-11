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

The bridge publishes two topics per tracked rigid body, nested under the
mocap system's namespace as the synapse_fbs 0.6 README recommends for
infrastructure sources:

```text
qualisys/<body_name>/pose   # synapse.topic.ExternalOdometryData (64 bytes)
qualisys/<body_name>/odom   # synapse.topic.OdometryEstimateData (232 bytes)
```

`pose` carries the pose/twist estimator input; `odom` carries the full
Kalman filter estimate including the pose and velocity covariance blocks,
reset counter, and quality percentage. The `pose` key is a deliberate
deviation from the catalog's curated `external_pose` key; consumers identify
payload types from the mandatory value contract, not the key. Run
`cargo run --bin synapse-qualisys-bridge -- --help` for all options.

Example subscriber configuration from another machine:

```text
endpoint: udp/<bridge-pc-ip>:7447
key expression: qualisys/<body_name>/pose   # estimator input for one vehicle
key expression: qualisys/<body_name>/odom   # full estimate with covariance
key expression: qualisys/**                 # everything from this bridge
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

The bridge continuously requests the QTM 6DOF rigid-body component. QTM
reports mocap positions in millimeters; the bridge converts all Synapse
positions to meters before publishing. Rigid-body samples are invalid only
when the pose values are not finite.

`<body_name>` is the QTM rigid-body name sanitized to a lowercase snake_case
Zenoh key segment (`Cub 1` becomes `cub_1`), with a `body_<id>` fallback for
empty or unusable names, so subscribers select a vehicle by name directly
from the key. Use `--namespace` to change the leading namespace (for example
`field_lab/qualisys`).

The estimator runs for every tracked body so the dashboard's External
Odometry panel can display the live Kalman filter state (position, attitude,
velocities, the full 12x12 covariance, reset counter, quality, and the raw
QTM 2D drop and out-of-sync rates); each Zenoh publication stays
demand-driven, only sending a body's `pose` or `odom` payloads while Zenoh
reports a matching subscriber for that key expression. The estimator is a 12D
Lie/error-state EKF prediction model generated from the
`Estimation.Examples.MocapExternalOdometryIEKF` Modelica example in the pinned
`third_party/modelica_models` submodule. It estimates position, attitude,
linear velocity, and angular velocity without a centered least-squares fit that
would add lookahead latency. State resets across dropped, out-of-sync, invalid,
or too-sparse samples.

QTM reports 2D drop and out-of-sync rates in 1/100 of a percent, and trace
nonzero values are normal on real capture volumes. Odometry is marked
`Degraded` only while either rate is at or above
`stream.degraded_rate_threshold_dpermille` (default 100, i.e. 1%); set it to
0 to disable the marking.

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
