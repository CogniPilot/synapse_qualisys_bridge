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
topic counters, view recent logs, and run best-effort Zenoh admin discovery for
publishers, subscribers, and queryables.

The dashboard can also manage a local Zenoh router process when
`zenoh.router_command` is configured in `bridge.toml`. The default command is
`zenohd`, so it works when `zenohd` is available on `PATH`.

Run against a real QTM host:

```sh
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 192.168.1.10 \
  --zenoh-connect udp/127.0.0.1:7447
```

Run against the SDK simulator:

```sh
cargo run --manifest-path ../qualisys_rust_sdk/Cargo.toml --bin qualisys-sim
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 127.0.0.1 \
  --zenoh-connect udp/127.0.0.1:7447
```

Useful defaults can also come from environment variables:

```sh
QUALISYS_HOST=127.0.0.1 ZENOH_CONNECT=udp/127.0.0.1:7447 cargo run --bin synapse-qualisys-bridge
```

The default Zenoh key expression is `synapse/mocap/frame`. Run
`cargo run --bin synapse-qualisys-bridge -- --help` for all options.

Write a default configuration file:

```sh
cargo run --bin synapse-qualisys-bridge -- --write-default-config bridge.toml
```

Run from a configuration file:

```sh
synapse-qualisys-bridge --config bridge.toml --open
```

By default, the bridge requests and publishes rigid-body pose only. Add
high-rate data explicitly when bandwidth allows:

```sh
cargo run --bin synapse-qualisys-bridge -- \
  --qualisys-host 192.168.1.10 \
  --include rigid-bodies,labeled-markers
```

Supported `--include` values are `rigid-bodies`, `labeled-markers`,
`unlabeled-markers`, and `skeleton`. Static metadata is published once on
`synapse/mocap/definition`.

The bridge also publishes compact per-rigid-body pose topics by default:

```text
synapse/mocap/rigid_body/<rigid_body_name>/pose
```

Each payload is 28 bytes: seven little-endian `f32` values in this order:
`position_m.x`, `position_m.y`, `position_m.z`, `quat.x`, `quat.y`, `quat.z`,
`quat.w`. Rigid body names come from QTM parameters and are sanitized for Zenoh
key expressions; missing names fall back to `id_<n>`. Use
`--rigid-body-pose-topic-prefix` to change the prefix, or
`--no-rigid-body-pose-topics` to disable these compact topics.

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
