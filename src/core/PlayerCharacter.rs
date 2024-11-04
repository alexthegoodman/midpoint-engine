use std::sync::MutexGuard;

use nalgebra::{Isometry3, Point3, Vector3};
use rapier3d::{
    control::{CharacterAutostep, KinematicCharacterController},
    prelude::{
        ActiveCollisionTypes, Collider, ColliderBuilder, ColliderHandle, QueryFilter, RigidBody,
        RigidBodyBuilder, RigidBodyHandle,
    },
};
use uuid::Uuid;

use crate::{handlers::get_camera, models::Model::Model};

use super::{RendererState::RendererState, SimpleCamera::SimpleCamera};

pub struct PlayerCharacter {
    pub id: Uuid,
    // Transform components
    // position: Vec3,
    // rotation: Quat,
    // camera_height: f32,
    // camera: SimpleCamera, // always the one camera
    pub model: Option<Model>,

    // Physics components
    pub character_controller: KinematicCharacterController,
    pub movement_collider: Collider,
    pub collider_handle: Option<ColliderHandle>,
    pub movement_rigid_body: RigidBody,
    pub movement_rigid_body_handle: Option<RigidBodyHandle>,
    // hit_collider: Collider,

    // Movement properties
    pub movement_speed: f32,
    pub mouse_sensitivity: f32,
}

impl PlayerCharacter {
    pub fn new() -> Self {
        let id = Uuid::new_v4();

        let movement_collider = ColliderBuilder::capsule_y(0.5, 1.0)
            .friction(0.0)
            .restitution(0.0)
            .density(1.0) // Add density to give it mass
            .user_data(id.as_u128())
            .active_collision_types(ActiveCollisionTypes::all()) // Make sure ALL collision types are enabled
            .build();

        let dynamic_body = RigidBodyBuilder::dynamic()
            .additional_mass(70.0) // Explicitly set mass (e.g., 70kg for a person)
            .linear_damping(0.1)
            .ccd_enabled(true)
            .user_data(id.as_u128())
            .build();

        Self {
            id,
            model: None,
            character_controller: KinematicCharacterController {
                autostep: Some(CharacterAutostep {
                    max_height: rapier3d::control::CharacterLength::Relative((0.5)),
                    min_width: rapier3d::control::CharacterLength::Relative((0.2)),
                    include_dynamic_bodies: true,
                }),
                slide: true,
                ..KinematicCharacterController::default()
            },
            movement_collider,
            collider_handle: None,
            movement_rigid_body: dynamic_body,
            movement_rigid_body_handle: None,
            // hit_collider: Collider::convex_hull(&player_model_verts).unwrap(), // this is on the optional Model
            movement_speed: 5.0,
            mouse_sensitivity: 0.003,
        }
    }

    pub fn update_rotation(dx: f32, dy: f32) {
        // the movement_collider and thus characte controller needn't rotate, only the Model's hit collider
        let camera = get_camera();
        let sensitivity = 0.005;

        let dx = -dx * sensitivity;
        let dy = dy * sensitivity;

        camera.rotate(dx, dy);

        // camera.update(); // should be called in render loop or here?
    }
}