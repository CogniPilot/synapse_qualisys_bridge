# synapse_qualisys_bridge

Rust bridge from Qualisys QTM RT motion-capture frames to Synapse FlatBuffers on
Zenoh.

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
