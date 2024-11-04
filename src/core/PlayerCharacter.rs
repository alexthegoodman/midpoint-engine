use std::sync::MutexGuard;

use nalgebra::{Isometry3, Point3, Vector3};
use rapier3d::{
    control::{CharacterAutostep, KinematicCharacterController},
    prelude::{
        Collider, ColliderBuilder, ColliderHandle, QueryFilter, RigidBody, RigidBodyBuilder,
        RigidBodyHandle,
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
            .user_data(id.as_u128())
            .build();

        let dynamic_body = RigidBodyBuilder::dynamic().user_data(id.as_u128()).build();

        Self {
            id,
            // position: Vec3::new(0.0, 1.0, 0.0),
            // rotation: Quat::IDENTITY,
            // camera_height: 1.6, // Typical eye height
            // camera: simple_camera,
            model: None,
            character_controller: KinematicCharacterController {
                // translation: Some(Vector3::identity()),
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

    pub fn update_position(
        &mut self,
        renderer_state: MutexGuard<RendererState>,
        translation: Vector3<f32>,
        delta_time: f32,
    ) {
        let mut camera = get_camera();
        // Collision filter (typically you want to collide with everything except other characters)
        let filter = QueryFilter::default()
            .exclude_rigid_body(
                self.movement_rigid_body_handle
                    .expect("Couldn't get rigid body handle"),
            ) // If you have an entity ID
            .exclude_collider(self.collider_handle.expect("Couldn't get collider handle"))
            .exclude_sensors(); // Typically don't collide with trigger volumes

        // Current character position
        let character_pos = Isometry3::translation(
            camera.position.x,
            camera.position.y - 0.9, // Offset by half height to put camera at top
            camera.position.z,
        );

        self.character_controller.move_shape(
            delta_time,
            &renderer_state.rigid_body_set,
            &renderer_state.collider_set,
            &renderer_state.query_pipeline,
            self.movement_collider.shape(),
            &character_pos,
            translation,
            filter,
            |collision| { /* Handle or collect the collision in this closure. */ },
        );

        camera.position = Point3::new(
            camera.position.x + translation.x,
            camera.position.y - 0.9 + translation.y,
            camera.position.z + translation.z,
        );

        // TODO: update collider with handle?
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
