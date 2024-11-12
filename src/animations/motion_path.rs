// use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Duration;

/// Example usage:
/// ```rust
/// let leg_motion = SkeletonMotionPath {
///     id: "walk_right_leg".to_string(),
///     target: SkeletonTarget {
///         skeleton_id: "98c9bb55-91de-472a-8279-5f86391c4993".to_string(),
///         part_id: "right_leg".to_string(),
///         ik_chain_id: Some("right_leg_ik".to_string()),
///         joint_id: None,
///     },
///     constraints: MotionConstraints {
///         rotation_limits: [[-45.0, -120.0, -45.0], [120.0, 45.0, 45.0]],
///         preserve_volume: true,
///         max_stretch: 1.0,
///     },
///     // ... other fields ...
/// };
/// ```
///
/// Represents a single keyframe in a motion path
#[serde_with::serde_as]
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct Keyframe {
    /// Time offset from the start of the animation
    #[serde_as(as = "serde_with::DurationSecondsWithFrac<String>")]
    pub time: Duration,
    /// Position in 3D space
    pub position: [f32; 3],
    /// Rotation as quaternion
    pub rotation: [f32; 4],
    /// Velocity at this keyframe for momentum calculation
    pub velocity: [f32; 3],
    /// Optional easing function identifier
    pub easing: Option<EasingType>,
}

/// Types of easing functions available for interpolation
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub enum EasingType {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
}

// /// Complete motion path containing keyframes and metadata
// #[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
// pub struct MotionPath {
//     /// Unique identifier for this path
//     pub id: String,
//     /// Ordered list of keyframes
//     pub keyframes: Vec<Keyframe>,
//     /// Total duration of the path
//     pub duration: Duration,
//     /// Whether this path loops
//     pub is_looping: bool,
//     /// Blending settings for transitions
//     pub blend_settings: BlendSettings,
// }

/// Settings for blending between different motion paths
#[serde_with::serde_as]
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct BlendSettings {
    /// Minimum blend time required
    #[serde_as(as = "serde_with::DurationSecondsWithFrac<String>")]
    pub min_blend_time: Duration,
    /// Maximum blend time allowed
    #[serde_as(as = "serde_with::DurationSecondsWithFrac<String>")]
    pub max_blend_time: Duration,
    /// Blend curve type
    pub blend_curve: EasingType,
    /// Which attributes should be blended
    pub blend_mask: SkeletonBlendMask,
}

/// Mask for controlling which attributes should be blended
// #[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
// pub struct BlendMask {
//     pub blend_position: bool,
//     pub blend_rotation: bool,
//     pub blend_velocity: bool,
//     pub preserve_events: bool,
// }

/// Categories of motion paths
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub enum PathCategory {
    Locomotion,
    Action,
    Cinematic,
    Reaction,
    Custom(String),
}

/// Identifies which part of the skeleton this motion affects
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct SkeletonTarget {
    /// ID of the skeleton this motion is for
    pub skeleton_id: String,
    /// ID of the specific part within the skeleton
    pub part_id: Option<String>,
    /// Optional IK chain ID if this motion controls an IK target
    pub ik_chain_id: Option<String>,
    /// Optional joint ID if this motion controls a specific joint
    pub joint_id: Option<String>,
}

/// Motion constraints based on skeleton definition
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct MotionConstraints {
    /// Rotation limits [min, max] for each axis
    pub rotation_limits: [[f32; 3]; 2],
    /// Whether to preserve volume during motion
    pub preserve_volume: bool,
    /// Maximum allowed stretch factor
    pub max_stretch: f32,
}

/// Extended keyframe with skeleton-specific properties
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct SkeletonKeyframe {
    /// Base keyframe properties
    pub base: Keyframe,
    /// IK target position if this keyframe controls an IK chain
    pub ik_target_position: Option<[f32; 3]>,
    /// IK pole vector position for controlling chain orientation
    pub ik_pole_position: Option<[f32; 3]>,
    /// Joint-specific rotations if controlling individual joints
    pub joint_rotations: Option<HashMap<String, [f32; 4]>>,
    /// Whether this keyframe requires foot contact
    pub foot_contact: bool,
    /// Blend between FK and IK (0.0 = full FK, 1.0 = full IK)
    pub fk_ik_blend: f32,
}

/// Complete motion path for skeleton animation
#[serde_with::serde_as]
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct SkeletonMotionPath {
    /// Unique identifier for this path
    pub id: String,
    /// Target skeleton part and optional IK/joint targets
    pub target: SkeletonTarget,
    /// Motion constraints derived from skeleton
    pub constraints: MotionConstraints,
    /// Ordered list of skeleton-specific keyframes
    pub keyframes: Vec<SkeletonKeyframe>,
    /// Total duration of the path
    #[serde_as(as = "serde_with::DurationSecondsWithFrac<String>")]
    pub duration: Duration,
    /// Whether this path loops
    pub is_looping: bool,
    /// Blending settings for transitions
    pub blend_settings: BlendSettings,
    /// Category of motion
    pub category: PathCategory,
    /// IK solver settings if using IK
    pub ik_settings: Option<IKSolverSettings>,
}

/// Settings for IK solving during motion
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct IKSolverSettings {
    /// Number of solver iterations
    pub solver_iterations: u32,
    /// Whether to allow stretching
    pub allow_stretch: bool,
    /// Whether to maintain foot contact
    pub foot_contact: bool,
    /// Default FK/IK blend value
    pub default_fk_blend: f32,
}

// /// Extended blend settings for skeleton motion
// #[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
// pub struct BlendSettings {
//     /// Minimum blend time required
//     pub min_blend_time: Duration,
//     /// Maximum blend time allowed
//     pub max_blend_time: Duration,
//     /// Blend curve type
//     pub blend_curve: EasingType,
//     /// Which attributes should be blended
//     pub blend_mask: SkeletonBlendMask,
// }

/// Extended blend mask for skeleton-specific properties
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct SkeletonBlendMask {
    pub blend_position: bool,
    pub blend_rotation: bool,
    pub blend_velocity: bool,
    pub blend_ik_targets: bool,
    pub blend_joint_rotations: bool,
    pub preserve_foot_contact: bool,
    pub preserve_volume: bool,
}

// Generate test motion path data
// pub fn create_test_motion_paths() -> Vec<MotionPath> {
//     vec![
//         // Jump animation path
//         MotionPath {
//             id: "jump_basic".to_string(),
//             keyframes: vec![
//                 // Starting pose
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.0),
//                     position: [0.0, 0.0, 0.0],
//                     rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
//                     velocity: [0.0, 0.0, 0.0],
//                     easing: Some(EasingType::EaseOut),
//                     metadata: KeyframeMetadata { flags: 0 },
//                 },
//                 // Crouch preparation
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.15),
//                     position: [0.0, -0.2, 0.0],
//                     rotation: [0.05, 0.0, 0.0, 0.9987], // Slight forward tilt
//                     velocity: [0.0, -1.0, 0.0],
//                     easing: Some(EasingType::EaseIn),
//                     metadata: KeyframeMetadata { flags: 1 }, // Preparation flag
//                 },
//                 // Jump initiation
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.3),
//                     position: [0.0, 0.5, 0.0],
//                     rotation: [-0.05, 0.0, 0.0, 0.9987], // Slight backward tilt
//                     velocity: [0.0, 4.0, 0.0],
//                     easing: Some(EasingType::EaseOut),
//                     metadata: KeyframeMetadata { flags: 2 }, // Jump start flag
//                 },
//                 // Peak of jump
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.6),
//                     position: [0.0, 1.2, 0.0],
//                     rotation: [0.0, 0.0, 0.0, 1.0],
//                     velocity: [0.0, 0.0, 0.0],
//                     easing: Some(EasingType::EaseInOut),
//                     metadata: KeyframeMetadata { flags: 4 }, // Peak flag
//                 },
//                 // Landing preparation
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.9),
//                     position: [0.0, 0.3, 0.0],
//                     rotation: [0.05, 0.0, 0.0, 0.9987],
//                     velocity: [0.0, -3.0, 0.0],
//                     easing: Some(EasingType::EaseIn),
//                     metadata: KeyframeMetadata { flags: 8 }, // Landing prep flag
//                 },
//                 // Landing impact
//                 Keyframe {
//                     time: Duration::from_secs_f64(1.0),
//                     position: [0.0, 0.0, 0.0],
//                     rotation: [0.1, 0.0, 0.0, 0.9950],
//                     velocity: [0.0, -2.0, 0.0],
//                     easing: Some(EasingType::EaseOut),
//                     metadata: KeyframeMetadata { flags: 16 }, // Landing impact flag
//                 },
//             ],
//             duration: Duration::from_secs_f64(1.0),
//             metadata: PathMetadata {
//                 category: PathCategory::Action,
//                 priority: 2,
//             },
//             is_looping: false,
//             blend_settings: BlendSettings {
//                 min_blend_time: Duration::from_secs_f64(0.1),
//                 max_blend_time: Duration::from_secs_f64(0.3),
//                 blend_curve: EasingType::EaseInOut,
//                 blend_mask: BlendMask {
//                     blend_position: true,
//                     blend_rotation: true,
//                     blend_velocity: true,
//                     preserve_events: true,
//                 },
//             },
//         },
//         // Walk cycle animation
//         MotionPath {
//             id: "walk_cycle".to_string(),
//             keyframes: vec![
//                 // Start of cycle
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.0),
//                     position: [0.0, 0.0, 0.0],
//                     rotation: [0.0, 0.0, 0.0, 1.0],
//                     velocity: [1.0, 0.0, 0.0],
//                     easing: Some(EasingType::Linear),
//                     metadata: KeyframeMetadata { flags: 1 },
//                 },
//                 // Mid step up
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.2),
//                     position: [0.2, 0.1, 0.0],
//                     rotation: [0.0, 0.0, 0.02, 0.9998],
//                     velocity: [1.0, 0.2, 0.0],
//                     easing: Some(EasingType::EaseInOut),
//                     metadata: KeyframeMetadata { flags: 2 },
//                 },
//                 // Full step
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.4),
//                     position: [0.4, 0.0, 0.0],
//                     rotation: [0.0, 0.0, -0.02, 0.9998],
//                     velocity: [1.0, 0.0, 0.0],
//                     easing: Some(EasingType::EaseInOut),
//                     metadata: KeyframeMetadata { flags: 4 },
//                 },
//                 // Mid step up (opposite)
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.6),
//                     position: [0.6, 0.1, 0.0],
//                     rotation: [0.0, 0.0, 0.02, 0.9998],
//                     velocity: [1.0, 0.2, 0.0],
//                     easing: Some(EasingType::EaseInOut),
//                     metadata: KeyframeMetadata { flags: 2 },
//                 },
//                 // Cycle complete
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.8),
//                     position: [0.8, 0.0, 0.0],
//                     rotation: [0.0, 0.0, 0.0, 1.0],
//                     velocity: [1.0, 0.0, 0.0],
//                     easing: Some(EasingType::Linear),
//                     metadata: KeyframeMetadata { flags: 1 },
//                 },
//             ],
//             duration: Duration::from_secs_f64(0.8),
//             metadata: PathMetadata {
//                 category: PathCategory::Locomotion,
//                 priority: 1,
//             },
//             is_looping: true,
//             blend_settings: BlendSettings {
//                 min_blend_time: Duration::from_secs_f64(0.2),
//                 max_blend_time: Duration::from_secs_f64(0.4),
//                 blend_curve: EasingType::EaseInOut,
//                 blend_mask: BlendMask {
//                     blend_position: true,
//                     blend_rotation: true,
//                     blend_velocity: true,
//                     preserve_events: false,
//                 },
//             },
//         },
//         // Attack animation
//         MotionPath {
//             id: "attack_swing".to_string(),
//             keyframes: vec![
//                 // Wind up
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.0),
//                     position: [0.0, 0.0, 0.0],
//                     rotation: [0.0, -0.2, 0.0, 0.9798], // Turned slightly right
//                     velocity: [0.0, 0.0, 0.0],
//                     easing: Some(EasingType::EaseIn),
//                     metadata: KeyframeMetadata { flags: 1 },
//                 },
//                 // Attack start
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.15),
//                     position: [0.1, 0.0, 0.1],
//                     rotation: [0.0, -0.3827, 0.0, 0.9239], // 45 degrees right
//                     velocity: [1.0, 0.0, 1.0],
//                     easing: Some(EasingType::Linear),
//                     metadata: KeyframeMetadata { flags: 2 },
//                 },
//                 // Strike
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.3),
//                     position: [0.2, 0.0, -0.1],
//                     rotation: [0.0, 0.3827, 0.0, 0.9239], // 45 degrees left
//                     velocity: [2.0, 0.0, -3.0],
//                     easing: Some(EasingType::EaseOut),
//                     metadata: KeyframeMetadata { flags: 4 },
//                 },
//                 // Follow through
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.45),
//                     position: [0.1, 0.0, -0.2],
//                     rotation: [0.0, 0.2, 0.0, 0.9798],
//                     velocity: [0.5, 0.0, -1.0],
//                     easing: Some(EasingType::EaseInOut),
//                     metadata: KeyframeMetadata { flags: 8 },
//                 },
//                 // Return to neutral
//                 Keyframe {
//                     time: Duration::from_secs_f64(0.6),
//                     position: [0.0, 0.0, 0.0],
//                     rotation: [0.0, 0.0, 0.0, 1.0],
//                     velocity: [0.0, 0.0, 0.0],
//                     easing: Some(EasingType::EaseIn),
//                     metadata: KeyframeMetadata { flags: 16 },
//                 },
//             ],
//             duration: Duration::from_secs_f64(0.6),
//             metadata: PathMetadata {
//                 category: PathCategory::Action,
//                 priority: 3,
//             },
//             is_looping: false,
//             blend_settings: BlendSettings {
//                 min_blend_time: Duration::from_secs_f64(0.1),
//                 max_blend_time: Duration::from_secs_f64(0.2),
//                 blend_curve: EasingType::EaseIn,
//                 blend_mask: BlendMask {
//                     blend_position: true,
//                     blend_rotation: true,
//                     blend_velocity: true,
//                     preserve_events: true,
//                 },
//             },
//         },
//     ]
// }
