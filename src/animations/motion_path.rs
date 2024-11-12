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

use crate::animations::render_skeleton::SkeletonRenderPart;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::time::Instant;
use uuid::Uuid;

/// Tracks the playback state of an animation
pub struct AnimationPlayback {
    /// Identifier to associate RendererState with animations
    pub id: String,
    /// Current time position in the animation
    pub current_time: Duration,
    /// When playback started
    pub start_time: Instant,
    /// Whether animation is currently playing
    pub is_playing: bool,
    /// Current motion paths being played
    pub active_motion_paths: Vec<SkeletonMotionPath>,
    /// Playback speed multiplier
    pub speed_multiplier: f32,
}

impl AnimationPlayback {
    pub fn new(motion_paths: Vec<SkeletonMotionPath>) -> Self {
        let id = Uuid::new_v4();

        Self {
            id: id.to_string(),
            current_time: Duration::from_secs(0),
            start_time: Instant::now(),
            is_playing: true, // for now playing if in the RendererState vector
            active_motion_paths: motion_paths,
            speed_multiplier: 1.0,
        }
    }

    pub fn play(&mut self) {
        self.is_playing = true;
        self.start_time = Instant::now();
    }

    pub fn pause(&mut self) {
        self.is_playing = false;
    }

    pub fn reset(&mut self) {
        self.current_time = Duration::from_secs(0);
        self.start_time = Instant::now();
    }

    /// Updates animation state and returns joint positions for the current frame
    pub fn update(&mut self) -> HashMap<String, (Point3<f32>, UnitQuaternion<f32>)> {
        if !self.is_playing {
            return HashMap::new();
        }

        // // Update current time
        // let elapsed = Instant::now().duration_since(self.start_time);
        // self.current_time = Duration::from_secs_f32(
        //     (elapsed.as_secs_f32() * self.speed_multiplier)
        //         % self.active_motion_paths[0].duration.as_secs_f32(),
        // );

        // Update current time
        let elapsed = Instant::now().duration_since(self.start_time);
        let raw_time = elapsed.as_secs_f32() * self.speed_multiplier;

        let duration = self.active_motion_paths[0].duration.as_secs_f32();

        // Check if we should loop or stop
        if raw_time >= duration {
            if self.active_motion_paths[0].is_looping {
                // Loop by using modulo
                self.current_time = Duration::from_secs_f32(raw_time % duration);
            } else {
                // Stop at the end
                self.is_playing = false;
                self.current_time = Duration::from_secs_f32(duration);
            }
        } else {
            self.current_time = Duration::from_secs_f32(raw_time);
        }

        // Calculate current joint positions and rotations
        let mut joint_transforms = HashMap::new();

        for motion_path in &self.active_motion_paths {
            let transforms = self.evaluate_motion_path(motion_path);
            joint_transforms.extend(transforms);
        }

        joint_transforms
    }

    /// Evaluates a motion path at the current time
    fn evaluate_motion_path(
        &self,
        motion_path: &SkeletonMotionPath,
    ) -> HashMap<String, (Point3<f32>, UnitQuaternion<f32>)> {
        let mut transforms = HashMap::new();
        let current_secs = self.current_time.as_secs_f32();

        // Find the surrounding keyframes
        let mut prev_keyframe = &motion_path.keyframes[0];
        let mut next_keyframe = &motion_path.keyframes[0];

        for window in motion_path.keyframes.windows(2) {
            let start_time = window[0].base.time.as_secs_f32();
            let end_time = window[1].base.time.as_secs_f32();

            if current_secs >= start_time && current_secs <= end_time {
                prev_keyframe = &window[0];
                next_keyframe = &window[1];
                break;
            }
        }

        // Calculate interpolation factor
        let start_time = prev_keyframe.base.time.as_secs_f32();
        let end_time = next_keyframe.base.time.as_secs_f32();
        let factor = (current_secs - start_time) / (end_time - start_time);

        // Apply easing if specified
        let eased_factor = match prev_keyframe.base.easing {
            Some(EasingType::EaseIn) => ease_in(factor),
            Some(EasingType::EaseOut) => ease_out(factor),
            Some(EasingType::EaseInOut) => ease_in_out(factor),
            _ => factor,
        };

        // Interpolate position
        let position = interpolate_vector3(
            &prev_keyframe.base.position.into(),
            &next_keyframe.base.position.into(),
            eased_factor,
        );

        // Interpolate rotation (using SLERP for quaternions)
        let rotation = interpolate_rotation(
            &prev_keyframe.base.rotation,
            &next_keyframe.base.rotation,
            eased_factor,
        );

        // If we have IK targets, handle those
        if let (Some(prev_ik), Some(next_ik)) = (
            &prev_keyframe.ik_target_position,
            &next_keyframe.ik_target_position,
        ) {
            let ik_target =
                interpolate_vector3(&(*prev_ik).into(), &(*next_ik).into(), eased_factor);

            // Store IK target transforms
            if let Some(chain_id) = &motion_path.target.ik_chain_id {
                transforms.insert(chain_id.clone(), (Point3::from(ik_target), rotation));
            }
        }

        // Handle joint-specific rotations if present
        if let (Some(prev_joints), Some(next_joints)) = (
            &prev_keyframe.joint_rotations,
            &next_keyframe.joint_rotations,
        ) {
            for (joint_id, prev_rot) in prev_joints {
                if let Some(next_rot) = next_joints.get(joint_id) {
                    let interpolated_rotation =
                        interpolate_rotation(prev_rot, next_rot, eased_factor);
                    transforms.insert(
                        joint_id.clone(),
                        (Point3::from(position), interpolated_rotation),
                    );
                }
            }
        }

        transforms
    }
}

/// Updates skeleton render parts based on animation state
pub fn update_skeleton_animation(
    render_parts: &mut Vec<SkeletonRenderPart>,
    animation: &mut AnimationPlayback,
    queue: &wgpu::Queue,
) {
    // Get current frame transforms
    let joint_transforms = animation.update();

    // Update each render part
    for part in render_parts {
        // Update bones based on new joint positions
        for bone in &mut part.bones {
            if let (Some((start_pos, start_rot)), Some((end_pos, end_rot))) = (
                joint_transforms.get(&bone.start_joint_id),
                joint_transforms.get(&bone.end_joint_id),
            ) {
                // double check this
                let euler = start_rot.euler_angles();

                // Update bone transform
                bone.update_from_joint_positions(
                    *start_pos,
                    *end_pos,
                    Some(Vector3::new(euler.0, euler.1, euler.2)),
                );

                // Update GPU buffer
                bone.transform.update_uniform_buffer(queue);

                // Update joint sphere position
                bone.joint_sphere.transform.position = start_pos.coords;
                bone.joint_sphere.transform.update_uniform_buffer(queue);
            }
        }
    }
}

// Helper functions for interpolation and easing

fn interpolate_vector3(start: &Vector3<f32>, end: &Vector3<f32>, t: f32) -> Vector3<f32> {
    start + (end - start) * t
}

fn interpolate_rotation(start: &[f32; 4], end: &[f32; 4], t: f32) -> UnitQuaternion<f32> {
    let q1 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        start[3], start[0], start[1], start[2],
    ));
    let q2 =
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(end[3], end[0], end[1], end[2]));
    q1.slerp(&q2, t)
}

fn ease_in(t: f32) -> f32 {
    t * t
}

fn ease_out(t: f32) -> f32 {
    1.0 - (1.0 - t) * (1.0 - t)
}

fn ease_in_out(t: f32) -> f32 {
    if t < 0.5 {
        2.0 * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(2) / 2.0
    }
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
