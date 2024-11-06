// use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Duration;

/// Represents a single keyframe in a motion path
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct Keyframe {
    /// Time offset from the start of the animation
    pub time: Duration,
    /// Position in 3D space
    pub position: [f32; 3],
    /// Rotation as quaternion
    pub rotation: [f32; 4],
    /// Velocity at this keyframe for momentum calculation
    pub velocity: [f32; 3],
    /// Optional easing function identifier
    pub easing: Option<EasingType>,
    /// Metadata for gameplay/animation events
    pub metadata: KeyframeMetadata,
}

/// Metadata that can be attached to keyframes for gameplay or animation purposes
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct KeyframeMetadata {
    // Animation events (footsteps, effects, etc)
    // pub events: Vec<AnimationEvent>,
    /// Gameplay-specific flags
    pub flags: u32,
    // Optional collision handling override
    // pub collision_behavior: Option<CollisionResponse>,
    // Custom properties for extensibility
    // pub properties: HashMap<String, PropertyValue>,
}

/// Types of easing functions available for interpolation
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub enum EasingType {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
    // Custom(Box<dyn Fn(f32) -> f32>),
}

/// Complete motion path containing keyframes and metadata
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct MotionPath {
    /// Unique identifier for this path
    pub id: String,
    /// Ordered list of keyframes
    pub keyframes: Vec<Keyframe>,
    /// Total duration of the path
    pub duration: Duration,
    /// Path-level metadata
    pub metadata: PathMetadata,
    /// Whether this path loops
    pub is_looping: bool,
    /// Blending settings for transitions
    pub blend_settings: BlendSettings,
}

/// Metadata for the entire motion path
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct PathMetadata {
    /// Path category (walk, run, jump, etc)
    pub category: PathCategory,
    /// Priority for blending/interruption
    pub priority: u32,
    // /// Required character capabilities
    // pub required_capabilities: Vec<Capability>,
    // /// Environmental constraints
    // pub constraints: Vec<EnvironmentConstraint>,
}

/// Settings for blending between different motion paths
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct BlendSettings {
    /// Minimum blend time required
    pub min_blend_time: Duration,
    /// Maximum blend time allowed
    pub max_blend_time: Duration,
    /// Blend curve type
    pub blend_curve: EasingType,
    /// Which attributes should be blended
    pub blend_mask: BlendMask,
}

/// Mask for controlling which attributes should be blended
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub struct BlendMask {
    pub blend_position: bool,
    pub blend_rotation: bool,
    pub blend_velocity: bool,
    pub preserve_events: bool,
}

/// Categories of motion paths
#[derive(PartialEq, Serialize, Deserialize, Clone, Debug)]
pub enum PathCategory {
    Locomotion,
    Action,
    Cinematic,
    Reaction,
    Custom(String),
}

/// Runtime manager for working with motion paths
pub struct MotionPathManager {
    /// Currently active motion paths
    active_paths: Vec<ActivePath>,
    /// Path library
    path_library: HashMap<String, MotionPath>,
    /// Current blend operations
    active_blends: Vec<BlendOperation>,
}

// impl MotionPathManager {
//     pub fn new() -> Self {
//         Self {
//             active_paths: Vec::new(),
//             path_library: HashMap::new(),
//             active_blends: Vec::new(),
//         }
//     }

//     /// Start playing a motion path
//     pub fn play_path(&mut self, path_id: &str, start_time: Duration) -> Result<(), PathError> {
//         // Implementation details...
//         todo!()
//     }

//     /// Update all active paths
//     pub fn update(&mut self, delta_time: Duration) {
//         // Implementation details...
//         todo!()
//     }

//     /// Blend between two paths
//     pub fn blend_to(&mut self, target_path: &str, blend_time: Duration) -> Result<(), PathError> {
//         // Implementation details...
//         todo!()
//     }

//     /// Sample a specific point in time on a path
//     pub fn sample_path(&self, path_id: &str, time: Duration) -> Option<Keyframe> {
//         // Implementation details...
//         todo!()
//     }
// }

/// Custom error type for motion path operations
#[derive(PartialEq, Clone, Serialize, Deserialize, Debug)]
pub enum PathError {
    PathNotFound,
    InvalidBlendOperation,
    InvalidTime,
    IncompatiblePaths,
}

/// Represents a currently playing motion path
#[derive(PartialEq, Clone, Serialize, Deserialize, Debug)]
struct ActivePath {
    path: MotionPath,
    current_time: Duration,
    is_paused: bool,
    playback_speed: f32,
}

/// Represents an active blend between two paths
#[derive(PartialEq, Clone, Serialize, Deserialize, Debug)]
struct BlendOperation {
    source_path: String,
    target_path: String,
    blend_progress: f32,
    blend_duration: Duration,
    blend_settings: BlendSettings,
}

/// Generate test motion path data
pub fn create_test_motion_paths() -> Vec<MotionPath> {
    vec![
        // Jump animation path
        MotionPath {
            id: "jump_basic".to_string(),
            keyframes: vec![
                // Starting pose
                Keyframe {
                    time: Duration::from_secs_f64(0.0),
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
                    velocity: [0.0, 0.0, 0.0],
                    easing: Some(EasingType::EaseOut),
                    metadata: KeyframeMetadata { flags: 0 },
                },
                // Crouch preparation
                Keyframe {
                    time: Duration::from_secs_f64(0.15),
                    position: [0.0, -0.2, 0.0],
                    rotation: [0.05, 0.0, 0.0, 0.9987], // Slight forward tilt
                    velocity: [0.0, -1.0, 0.0],
                    easing: Some(EasingType::EaseIn),
                    metadata: KeyframeMetadata { flags: 1 }, // Preparation flag
                },
                // Jump initiation
                Keyframe {
                    time: Duration::from_secs_f64(0.3),
                    position: [0.0, 0.5, 0.0],
                    rotation: [-0.05, 0.0, 0.0, 0.9987], // Slight backward tilt
                    velocity: [0.0, 4.0, 0.0],
                    easing: Some(EasingType::EaseOut),
                    metadata: KeyframeMetadata { flags: 2 }, // Jump start flag
                },
                // Peak of jump
                Keyframe {
                    time: Duration::from_secs_f64(0.6),
                    position: [0.0, 1.2, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    velocity: [0.0, 0.0, 0.0],
                    easing: Some(EasingType::EaseInOut),
                    metadata: KeyframeMetadata { flags: 4 }, // Peak flag
                },
                // Landing preparation
                Keyframe {
                    time: Duration::from_secs_f64(0.9),
                    position: [0.0, 0.3, 0.0],
                    rotation: [0.05, 0.0, 0.0, 0.9987],
                    velocity: [0.0, -3.0, 0.0],
                    easing: Some(EasingType::EaseIn),
                    metadata: KeyframeMetadata { flags: 8 }, // Landing prep flag
                },
                // Landing impact
                Keyframe {
                    time: Duration::from_secs_f64(1.0),
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.1, 0.0, 0.0, 0.9950],
                    velocity: [0.0, -2.0, 0.0],
                    easing: Some(EasingType::EaseOut),
                    metadata: KeyframeMetadata { flags: 16 }, // Landing impact flag
                },
            ],
            duration: Duration::from_secs_f64(1.0),
            metadata: PathMetadata {
                category: PathCategory::Action,
                priority: 2,
            },
            is_looping: false,
            blend_settings: BlendSettings {
                min_blend_time: Duration::from_secs_f64(0.1),
                max_blend_time: Duration::from_secs_f64(0.3),
                blend_curve: EasingType::EaseInOut,
                blend_mask: BlendMask {
                    blend_position: true,
                    blend_rotation: true,
                    blend_velocity: true,
                    preserve_events: true,
                },
            },
        },
        // Walk cycle animation
        MotionPath {
            id: "walk_cycle".to_string(),
            keyframes: vec![
                // Start of cycle
                Keyframe {
                    time: Duration::from_secs_f64(0.0),
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    velocity: [1.0, 0.0, 0.0],
                    easing: Some(EasingType::Linear),
                    metadata: KeyframeMetadata { flags: 1 },
                },
                // Mid step up
                Keyframe {
                    time: Duration::from_secs_f64(0.2),
                    position: [0.2, 0.1, 0.0],
                    rotation: [0.0, 0.0, 0.02, 0.9998],
                    velocity: [1.0, 0.2, 0.0],
                    easing: Some(EasingType::EaseInOut),
                    metadata: KeyframeMetadata { flags: 2 },
                },
                // Full step
                Keyframe {
                    time: Duration::from_secs_f64(0.4),
                    position: [0.4, 0.0, 0.0],
                    rotation: [0.0, 0.0, -0.02, 0.9998],
                    velocity: [1.0, 0.0, 0.0],
                    easing: Some(EasingType::EaseInOut),
                    metadata: KeyframeMetadata { flags: 4 },
                },
                // Mid step up (opposite)
                Keyframe {
                    time: Duration::from_secs_f64(0.6),
                    position: [0.6, 0.1, 0.0],
                    rotation: [0.0, 0.0, 0.02, 0.9998],
                    velocity: [1.0, 0.2, 0.0],
                    easing: Some(EasingType::EaseInOut),
                    metadata: KeyframeMetadata { flags: 2 },
                },
                // Cycle complete
                Keyframe {
                    time: Duration::from_secs_f64(0.8),
                    position: [0.8, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    velocity: [1.0, 0.0, 0.0],
                    easing: Some(EasingType::Linear),
                    metadata: KeyframeMetadata { flags: 1 },
                },
            ],
            duration: Duration::from_secs_f64(0.8),
            metadata: PathMetadata {
                category: PathCategory::Locomotion,
                priority: 1,
            },
            is_looping: true,
            blend_settings: BlendSettings {
                min_blend_time: Duration::from_secs_f64(0.2),
                max_blend_time: Duration::from_secs_f64(0.4),
                blend_curve: EasingType::EaseInOut,
                blend_mask: BlendMask {
                    blend_position: true,
                    blend_rotation: true,
                    blend_velocity: true,
                    preserve_events: false,
                },
            },
        },
        // Attack animation
        MotionPath {
            id: "attack_swing".to_string(),
            keyframes: vec![
                // Wind up
                Keyframe {
                    time: Duration::from_secs_f64(0.0),
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.0, -0.2, 0.0, 0.9798], // Turned slightly right
                    velocity: [0.0, 0.0, 0.0],
                    easing: Some(EasingType::EaseIn),
                    metadata: KeyframeMetadata { flags: 1 },
                },
                // Attack start
                Keyframe {
                    time: Duration::from_secs_f64(0.15),
                    position: [0.1, 0.0, 0.1],
                    rotation: [0.0, -0.3827, 0.0, 0.9239], // 45 degrees right
                    velocity: [1.0, 0.0, 1.0],
                    easing: Some(EasingType::Linear),
                    metadata: KeyframeMetadata { flags: 2 },
                },
                // Strike
                Keyframe {
                    time: Duration::from_secs_f64(0.3),
                    position: [0.2, 0.0, -0.1],
                    rotation: [0.0, 0.3827, 0.0, 0.9239], // 45 degrees left
                    velocity: [2.0, 0.0, -3.0],
                    easing: Some(EasingType::EaseOut),
                    metadata: KeyframeMetadata { flags: 4 },
                },
                // Follow through
                Keyframe {
                    time: Duration::from_secs_f64(0.45),
                    position: [0.1, 0.0, -0.2],
                    rotation: [0.0, 0.2, 0.0, 0.9798],
                    velocity: [0.5, 0.0, -1.0],
                    easing: Some(EasingType::EaseInOut),
                    metadata: KeyframeMetadata { flags: 8 },
                },
                // Return to neutral
                Keyframe {
                    time: Duration::from_secs_f64(0.6),
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    velocity: [0.0, 0.0, 0.0],
                    easing: Some(EasingType::EaseIn),
                    metadata: KeyframeMetadata { flags: 16 },
                },
            ],
            duration: Duration::from_secs_f64(0.6),
            metadata: PathMetadata {
                category: PathCategory::Action,
                priority: 3,
            },
            is_looping: false,
            blend_settings: BlendSettings {
                min_blend_time: Duration::from_secs_f64(0.1),
                max_blend_time: Duration::from_secs_f64(0.2),
                blend_curve: EasingType::EaseIn,
                blend_mask: BlendMask {
                    blend_position: true,
                    blend_rotation: true,
                    blend_velocity: true,
                    preserve_events: true,
                },
            },
        },
    ]
}
