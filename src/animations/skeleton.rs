use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// NOTE: these types will be fed to AI when generating SkeletonParts

/// Represents the orientation and rotation order for a joint
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointOrientation {
    /// Default rotation as a quaternion
    pub default_rotation: [f32; 4], // Quat
    /// Rotation order (e.g., "XYZ", "ZXY")
    pub rotation_order: String,
    /// Whether this joint's rotation should be mirrored when retargeting
    pub mirror_rotation: bool,
}

/// Represents constraints on joint movement
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointConstraints {
    /// Minimum rotation angles in degrees for each axis
    pub rotation_min: [f32; 3], // Vec3
    /// Maximum rotation angles in degrees for each axis
    pub rotation_max: [f32; 3], // Vec3
    /// Whether to preserve volume during deformation
    pub preserve_volume: bool,
    /// Maximum allowed stretch factor for stretchy IK
    pub max_stretch: f32,
}

/// Represents a single bone/joint in the skeleton
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Joint {
    /// Unique identifier for the joint
    pub id: String,
    /// Display name of the joint
    pub name: String,
    /// ID of the parent joint (None for root)
    pub parent_id: Option<String>,
    /// Local position relative to parent
    pub local_position: [f32; 3], // Vec3
    /// Local orientation
    pub orientation: JointOrientation,
    /// Optional movement constraints
    pub constraints: Option<JointConstraints>,
    /// Joint type for special behavior
    pub joint_type: JointType,
    /// Additional metadata for engine-specific features
    pub metadata: HashMap<String, String>,
}

/// Categorizes joints by their purpose
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum JointType {
    /// Standard skeletal joint
    Standard,
    /// Helper joint for better deformation
    Helper,
    /// Joint for attaching equipment/props
    Attachment,
    /// Joint for physics simulation
    Physics,
    /// Joint specifically for twist distribution
    Twist,
    /// Specialized joint for facial animation
    Facial,
    /// Control joint for IK systems
    IKControl,
}

/// Represents an IK chain in the skeleton
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IKChain {
    /// Unique identifier for the chain
    pub id: String,
    /// Name of the chain
    pub name: String,
    /// Joint ID of the start of the chain
    pub start_joint: String,
    /// Joint ID of the end of the chain
    pub end_joint: String,
    /// Joint ID of the IK control joint
    pub control_joint: String,
    /// Number of iterations for IK solving
    pub solver_iterations: u32,
    /// Chain-specific settings
    pub settings: IKSettings,
}

/// Settings for IK chain behavior
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IKSettings {
    /// Whether the chain supports stretching
    pub allow_stretch: bool,
    /// Pole vector control joint ID if any
    pub pole_vector_joint: Option<String>,
    /// Whether to maintain foot contact
    pub foot_contact: bool,
    /// Blend weight with FK (0.0 - 1.0)
    pub fk_blend: f32,
}

/// Complete skeleton definition
#[derive(Serialize, Deserialize, Debug)]
pub struct Skeleton {
    /// Unique identifier for the skeleton
    pub id: String,
    /// Display name of the skeleton
    pub name: String,
    /// Version of the skeleton data format
    pub version: String,
    /// All joints in the skeleton
    pub joints: Vec<Joint>,
    /// IK chain definitions
    pub ik_chains: Vec<IKChain>,
    /// Level of detail configurations
    pub lod_configs: Vec<LodConfig>,
    /// Engine-specific metadata
    pub metadata: HashMap<String, String>,
}

/// Configuration for a specific level of detail
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LodConfig {
    /// LOD level (0 is highest detail)
    pub level: u32,
    /// Distance at which this LOD becomes active
    pub distance: f32,
    /// Joints to disable at this LOD level
    pub disabled_joints: Vec<String>,
    /// Maximum influences per vertex at this LOD
    pub max_influences: u32,
}

/// Represents connection points where skeleton parts can be attached
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AttachPoint {
    /// Unique identifier for this attachment point
    pub id: String,
    /// The type of joint that can connect here
    pub compatible_types: Vec<String>,
    /// Local position relative to the parent joint
    pub local_position: [f32; 3],
    /// Local rotation as quaternion
    pub local_rotation: [f32; 4],
    /// Optional constraints for the attachment
    pub constraints: Option<JointConstraints>,
    /// Metadata for special handling
    pub metadata: HashMap<String, String>,
}

/// A discrete part of a skeleton that can be assembled with others
#[derive(Serialize, Deserialize, Debug)]
pub struct SkeletonPart {
    /// Unique identifier for this part
    pub id: String,
    /// Display name of the part
    pub name: String,
    /// Category of this part (e.g., "arm", "leg", "head")
    pub part_type: String,
    /// Version of this part
    pub version: String,
    /// The joints that make up this part
    pub joints: Vec<Joint>,
    /// IK chains contained in this part
    pub ik_chains: Vec<IKChain>,
    /// Points where this part can be attached to others
    pub attach_points: Vec<AttachPoint>,
    /// Points where other parts can be attached to this one
    pub accept_points: Vec<AttachPoint>,
    /// Metadata for the part
    pub metadata: HashMap<String, String>,
}

/// Configuration for assembling skeleton parts
#[derive(Serialize, Deserialize, Debug)]
pub struct SkeletonAssemblyConfig {
    /// Unique identifier for this assembly
    pub id: String,
    /// Display name of the assembled skeleton
    pub name: String,
    /// The root part to start assembly from
    pub root_part_id: String,
    /// Connections between parts
    pub connections: Vec<PartConnection>,
    /// Global settings for the assembled skeleton
    pub settings: AssemblySettings,
}

/// Defines how two skeleton parts connect
#[derive(Serialize, Deserialize, Debug)]
pub struct PartConnection {
    /// ID of the parent part
    pub parent_part_id: String,
    /// ID of the child part
    pub child_part_id: String,
    /// ID of the attachment point on the parent
    pub parent_attach_point: String,
    /// ID of the attachment point on the child
    pub child_attach_point: String,
    /// Optional transform adjustments
    pub transform_offset: Option<TransformOffset>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TransformOffset {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub scale: [f32; 3],
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AssemblySettings {
    /// How to handle naming conflicts
    pub naming_strategy: NamingStrategy,
    /// Whether to validate connections during assembly
    pub validate_connections: bool,
    /// Whether to auto-generate missing IK chains
    pub generate_missing_ik: bool,
    /// LOD settings for the assembled skeleton
    pub lod_settings: LodConfig,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum NamingStrategy {
    /// Add prefix based on part ID
    Prefix,
    /// Add suffix based on part ID
    Suffix,
    /// Use GUID-style unique identifiers
    Guid,
    /// Keep original names, error on conflicts
    Strict,
}
