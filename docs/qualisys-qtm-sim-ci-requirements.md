# Qualisys QTM Simulator Requirements For CI

This simulator is needed by `synapse_qualisys_bridge` CI and should live in the
Qualisys SDK repo so the SDK can test it directly and downstream bridges can
reuse the same fake QTM endpoint.

## Goal

Provide a small QTM RT-compatible simulator that exercises the real SDK client
and bridge streaming path without a physical Qualisys system.

The bridge CI will:

1. Start the simulator on localhost.
2. Start `synapse-qualisys-bridge` pointed at the simulator.
3. Subscribe to the bridge's Zenoh topics.
4. Assert that raw `MocapFrame` and demand-driven `ExternalOdometryData`
   messages are published.

## CLI Contract

Provide a binary named:

```sh
qualisys-sim
```

Required arguments:

```sh
qualisys-sim --bind 127.0.0.1:22223 --hz 240 --rigid-bodies 2
```

Recommended defaults:

- `--bind`: `127.0.0.1:22223`
- `--hz`: `240`
- `--rigid-bodies`: `1`

The process should run until terminated.

## QTM RT Command Support

The simulator should speak the same little-endian framed packet protocol that
the SDK client already parses.

Required behavior:

- On TCP client connection, send a command packet containing:
  `QTM RT Interface connected`
- `Version 1.27` returns a command packet beginning with:
  `Version set to 1.27`
- `qtmversion` returns any command text response.
- `byteorder` returns:
  `little endian`
- `getparameters ...` returns an XML packet containing `General` frequency and
  a `The_6D` body list.
- `getcurrentframe ...` returns one data packet.
- `streamframes ... udp:<host>:<port> 6dres` streams data packets over UDP to
  the requested destination.
- `streamframes ... 6dres` without a UDP destination streams data packets on
  the existing TCP command socket.
- `streamframes stop` stops active streaming for that client.

Unsupported commands can return QTM error packets with useful text.

## Parameters XML

`getparameters` must return enough metadata for the bridge to name rigid bodies.

Example body entries:

```xml
<QTM_Parameters_Ver_1.27>
  <General>
    <Frequency>240</Frequency>
  </General>
  <The_6D>
    <Body>
      <Name>sim_body_1</Name>
      <Color R="0" G="255" B="0" />
      <Enabled>true</Enabled>
    </Body>
  </The_6D>
</QTM_Parameters_Ver_1.27>
```

Generate one `<Body>` per `--rigid-bodies`.

## Stream Data

The bridge currently needs `6dres` / `SixDResidual` frames.

Each streamed packet should include:

- Incrementing frame number.
- Monotonic timestamp in microseconds.
- One `SixDResidual` component.
- `drop_rate = 0`.
- `out_of_sync_rate = 0`.
- `N` rigid bodies, where `N == --rigid-bodies`.

Rigid body sample requirements:

- Position values are in QTM units, i.e. millimeters.
- Positions are finite and change over time.
- Rotation matrix is finite and approximately orthonormal.
- Residual is finite and small.

Simple generated motion is sufficient, for example a slow circular trajectory
and yaw rotation:

- `x_mm = 750 * cos(t)`
- `y_mm = 750 * sin(t)`
- `z_mm = 1000 + 100 * sin(2t)`
- yaw rate around `0.35 rad/s`
- residual around `0.001`

## SDK-Level Tests

The SDK repo should test the simulator before bridge CI depends on it:

- Start `qualisys-sim` on a local random/free port.
- Connect with the SDK client.
- Verify `qtmversion`, `byteorder`, and `get_mocap_parameters`.
- Request `SixDResidual` streaming over UDP.
- Assemble several frames with the SDK frame accumulator.
- Assert frame numbers increase and rigid body samples are finite.
- Repeat once with TCP streaming if the SDK supports TCP stream tests cheaply.

## Bridge CI Usage

The bridge CI expects to be able to run something equivalent to:

```sh
cargo build --manifest-path ../qualisys_rust_sdk/Cargo.toml --bin qualisys-sim
../qualisys_rust_sdk/target/debug/qualisys-sim \
  --bind 127.0.0.1:22223 \
  --hz 240 \
  --rigid-bodies 2
```

Once this simulator is available from the SDK checkout, the bridge tests can
start it as a background process and use the real bridge binary plus real Zenoh
subscribers for end-to-end assertions.
