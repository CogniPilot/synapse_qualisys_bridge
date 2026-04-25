use std::collections::BTreeSet;
use std::mem::size_of;
use std::net::{IpAddr, SocketAddr};
use std::time::Duration;

use clap::{Parser, ValueEnum};
use flatbuffers::FlatBufferBuilder;
use qualisys_rust_sdk::rt::{
    AssembledFrame, Client, ClientOptions, ComponentData, ComponentSelection, ComponentType,
    FrameAccumulator, MocapParameters, MocapSkeletonSegment, StreamFramesRequest, StreamPacket,
    StreamRate, StreamTransport,
};
use synapse_fbs::topic::{
    MocapDefinition, MocapDefinitionArgs, MocapFrame, MocapFrameArgs, MocapMarkerDefinition,
    MocapMarkerDefinitionArgs, MocapMarkerSample, MocapRigidBodyDefinition,
    MocapRigidBodyDefinitionArgs, MocapRigidBodySample, MocapSegmentDefinition,
    MocapSegmentDefinitionArgs, MocapSegmentSample, Quaternionf, Vec3f,
};
use thiserror::Error;
use zenoh::{Wait, config::Config};

#[derive(Debug, Parser)]
#[command(
    name = "synapse-qualisys-bridge",
    version,
    about = "Bridge Qualisys QTM RT mocap data to Synapse FlatBuffers on Zenoh",
    long_about = "Listens to a Qualisys QTM RT stream, converts selected mocap components into \
synapse.topic.MocapFrame FlatBuffers, and publishes them on a Zenoh key expression.",
    next_line_help = true,
    after_help = "\
Examples:
  synapse-qualisys-bridge --qualisys-host 192.168.1.10
  synapse-qualisys-bridge --qualisys-host 127.0.0.1 --qualisys-port 22224 --zenoh-connect udp/127.0.0.1:7447

Environment:
  QUALISYS_HOST, QUALISYS_PORT, QUALISYS_TIMEOUT_MS
  ZENOH_CONNECT, ZENOH_TOPIC, ZENOH_DEFINITION_TOPIC
  ZENOH_RIGID_BODY_POSE_TOPIC_PREFIX, ZENOH_NO_RIGID_BODY_POSE_TOPICS
  MOCAP_INCLUDE"
)]
struct Cli {
    #[command(flatten)]
    qualisys: QualisysArgs,

    #[command(flatten)]
    stream: StreamArgs,

    #[command(flatten)]
    zenoh: ZenohArgs,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Qualisys")]
struct QualisysArgs {
    #[arg(
        long = "qualisys-host",
        env = "QUALISYS_HOST",
        value_name = "HOST",
        default_value = "127.0.0.1",
        help = "QTM RT host name or IP address"
    )]
    qualisys_host: String,

    #[arg(
        long = "qualisys-port",
        env = "QUALISYS_PORT",
        value_name = "PORT",
        default_value_t = qualisys_rust_sdk::rt::LITTLE_ENDIAN_PORT,
        help = "QTM RT little-endian TCP command port"
    )]
    qualisys_port: u16,

    #[arg(
        long = "timeout-ms",
        env = "QUALISYS_TIMEOUT_MS",
        value_name = "MS",
        default_value_t = 5_000,
        help = "QTM command and stream receive timeout"
    )]
    timeout_ms: u64,
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
        help = "High-rate data to include. Repeat or comma-separate. Default: rigid-bodies",
        long_help = "High-rate data to include in synapse.topic.MocapFrame. Repeat or \
comma-separate values. Default is rigid-bodies only to minimize bandwidth."
    )]
    include: Vec<MocapData>,

    #[arg(
        long,
        value_enum,
        default_value = "udp",
        help = "Transport used for QTM streamframes"
    )]
    transport: StreamTransportArg,

    #[arg(
        long,
        value_name = "ADDR:PORT",
        default_value = "0.0.0.0:0",
        help = "Local UDP bind address when --transport udp"
    )]
    udp_bind: SocketAddr,

    #[arg(
        long,
        value_name = "IP",
        help = "Optional QTM UDP destination IP; defaults to QTM's peer address"
    )]
    udp_destination: Option<IpAddr>,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum StreamTransportArg {
    Udp,
    Tcp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
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
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        default_value = "udp/127.0.0.1:7447",
        help = "Zenoh router locator"
    )]
    zenoh_connect: String,

    #[arg(
        long = "topic",
        alias = "zenoh-topic",
        env = "ZENOH_TOPIC",
        value_name = "KEYEXPR",
        default_value = "synapse/mocap/frame",
        help = "Zenoh key expression for synapse.topic.MocapFrame payloads"
    )]
    topic: String,

    #[arg(
        long = "definition-topic",
        alias = "zenoh-definition-topic",
        env = "ZENOH_DEFINITION_TOPIC",
        value_name = "KEYEXPR",
        default_value = "synapse/mocap/definition",
        help = "Zenoh key expression for low-rate synapse.topic.MocapDefinition metadata"
    )]
    definition_topic: String,

    #[arg(
        long = "rigid-body-pose-topic-prefix",
        env = "ZENOH_RIGID_BODY_POSE_TOPIC_PREFIX",
        value_name = "KEYEXPR",
        default_value = "synapse/mocap/rigid_body",
        help = "Zenoh key expression prefix for compact per-rigid-body pose topics"
    )]
    rigid_body_pose_topic_prefix: String,

    #[arg(
        long = "no-rigid-body-pose-topics",
        env = "ZENOH_NO_RIGID_BODY_POSE_TOPICS",
        default_value_t = false,
        help = "Disable compact per-rigid-body pose topic publishing"
    )]
    no_rigid_body_pose_topics: bool,
}

#[derive(Debug, Error)]
enum BridgeError {
    #[error(transparent)]
    Qtm(#[from] qualisys_rust_sdk::QtmError),
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("frame does not contain requested {0} component")]
    MissingComponent(&'static str),
}

type Result<T> = std::result::Result<T, BridgeError>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let session = zenoh::open(zenoh_config(&cli)?)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let publisher = session
        .declare_publisher(cli.zenoh.topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let definition_publisher = session
        .declare_publisher(cli.zenoh.definition_topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    let mut client = connect_qualisys(&cli)?;
    let selection = selected_data(&cli);
    let parameters = client.get_mocap_parameters()?;
    let rigid_body_names = rigid_body_names(&parameters);
    let definition = encode_mocap_definition(&parameters, &selection)?;
    definition_publisher
        .put(definition)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    let components = stream_components(&selection);
    let request = StreamFramesRequest::new(
        StreamRate::AllFrames,
        stream_transport(&cli),
        components.clone(),
    );
    client.start_stream_frames(&request)?;

    let mut accumulator = FrameAccumulator::for_components(components);
    loop {
        match client.recv_stream_packet()? {
            StreamPacket::Data(packet) => {
                for frame in accumulator.push(packet) {
                    let payload = encode_mocap_frame(&frame, &selection)?;
                    publisher
                        .put(payload)
                        .wait()
                        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
                    publish_rigid_body_pose_topics(
                        &session,
                        &cli,
                        &frame,
                        &selection,
                        &rigid_body_names,
                    )?;
                }
            }
            StreamPacket::NoMoreData => return Ok(()),
        }
    }
}

fn zenoh_config(cli: &Cli) -> Result<Config> {
    let mut config = Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    config
        .insert_json5(
            "connect/endpoints",
            &format!("[\"{}\"]", cli.zenoh.zenoh_connect),
        )
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    Ok(config)
}

fn connect_qualisys(cli: &Cli) -> Result<Client> {
    Ok(Client::connect(
        &cli.qualisys.qualisys_host,
        ClientOptions {
            port: cli.qualisys.qualisys_port,
            read_timeout: Duration::from_millis(cli.qualisys.timeout_ms),
            ..ClientOptions::default()
        },
    )?)
}

fn stream_transport(cli: &Cli) -> StreamTransport {
    match cli.stream.transport {
        StreamTransportArg::Udp => StreamTransport::Udp {
            bind_address: cli.stream.udp_bind,
            destination: cli.stream.udp_destination,
        },
        StreamTransportArg::Tcp => StreamTransport::Tcp,
    }
}

fn selected_data(cli: &Cli) -> BTreeSet<MocapData> {
    if cli.stream.include.is_empty() {
        return BTreeSet::from([MocapData::RigidBodies]);
    }

    cli.stream.include.iter().copied().collect()
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
    cli: &Cli,
    frame: &AssembledFrame,
    selection: &BTreeSet<MocapData>,
    rigid_body_names: &[String],
) -> Result<()> {
    if cli.zenoh.no_rigid_body_pose_topics || !selection.contains(&MocapData::RigidBodies) {
        return Ok(());
    }

    for body in rigid_bodies(frame)? {
        let topic = rigid_body_pose_topic(
            &cli.zenoh.rigid_body_pose_topic_prefix,
            &body,
            rigid_body_names,
        );
        session
            .put(topic, encode_compact_pose(&body))
            .wait()
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    }

    Ok(())
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

fn encode_compact_pose(body: &MocapRigidBodySample) -> Vec<u8> {
    let position = body.position();
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
                    &Vec3f::new(marker.x, marker.y, marker.z),
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
                    &Vec3f::new(marker.x, marker.y, marker.z),
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
                    &Vec3f::new(marker.x, marker.y, marker.z),
                    marker.residual,
                )
            })
            .collect()),
        ComponentData::ThreeDNoLabels(component) => Ok(component
            .markers
            .iter()
            .map(|marker| {
                MocapMarkerSample::new(marker.id, &Vec3f::new(marker.x, marker.y, marker.z), 0.0)
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
                        &Vec3f::new(segment.position.x, segment.position.y, segment.position.z),
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
                MocapRigidBodySample::new(
                    (index + 1) as i32,
                    &Vec3f::new(body.position.x, body.position.y, body.position.z),
                    &quat_from_rotation_matrix(body.rotation_matrix),
                    body.residual,
                    true,
                )
            })
            .collect()),
        ComponentData::SixD(component) => Ok(component
            .bodies
            .iter()
            .enumerate()
            .map(|(index, body)| {
                MocapRigidBodySample::new(
                    (index + 1) as i32,
                    &Vec3f::new(body.position.x, body.position.y, body.position.z),
                    &quat_from_rotation_matrix(body.rotation_matrix),
                    0.0,
                    true,
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
