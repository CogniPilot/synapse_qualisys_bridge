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

The default Zenoh key expression is `synapse/v1/topic/mocap_frame`. Run
`cargo run --bin synapse-qualisys-bridge -- --help` for all options.

Example subscriber configuration from another machine:

```text
endpoint: udp/<bridge-pc-ip>:7447
key expression: synapse/v1/topic/**
```

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
and tooling. Rigid-body samples are marked invalid when QTM reports a dropped or
out-of-sync 6D component, or when the pose values are not finite.

The bridge also publishes fixed-layout per-rigid-body external odometry payloads
by default:

```text
synapse/v1/topic/external_odometry/<rigid_body_id>
```

Each payload is the 136-byte `synapse.topic.ExternalOdometryData` struct:
timestamp, position in ENU meters, attitude as body-FLU to ENU quaternion,
linear velocity in ENU meters per second, and angular velocity in body-FLU
radians per second. The derived odometry path is demand-driven: the bridge only
runs the per-body estimator and publishes these payloads when Zenoh reports a
matching subscriber for that body's odometry key expression. Subscribing only
to raw mocap frames does not force odometry work. The estimator is a 12D
Lie/error-state EKF prediction model generated from the
`Estimation.Examples.MocapExternalOdometryIEKF` Modelica example in the pinned
`third_party/modelica_models` submodule. It estimates position, attitude,
linear velocity, and angular velocity without a centered least-squares fit that
would add lookahead latency. State resets across dropped, out-of-sync, invalid,
or too-sparse samples. Use `--external-odometry-topic-prefix` to change the
prefix, or `--no-external-odometry` to disable these topics.

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

It can optionally add a startup shortcut so the bridge starts when the user
signs in. The installed dashboard checks GitHub releases for newer installer
assets and can launch the installer to update itself.

The release workflow reads the locked `synapse_fbs` crate version from
`Cargo.lock`, adds it to the GitHub release notes, and includes it in each
archive. Consumers should decode Synapse FlatBuffers payloads with the
documented `synapse_fbs` version for that release.
