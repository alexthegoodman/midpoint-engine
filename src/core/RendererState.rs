use floem::keyboard::ModifiersState;
use nalgebra::{Isometry3, Point3, Vector3};
use rapier3d::math::Point as RapierPoint;
use rapier3d::prelude::*;
use rapier3d::prelude::{ColliderSet, QueryPipeline, RigidBodySet};
use uuid::Uuid;
use wgpu::BindGroupLayout;

use crate::animations::render_skeleton::SkeletonRenderPart;
use crate::animations::skeleton::Joint;
use crate::handlers::get_camera;
use crate::{
    core::Texture::Texture,
    helpers::saved_data::{ComponentData, ComponentKind},
};
use std::collections::HashMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::Instant;

use crate::{
    helpers::{landscapes::LandscapePixelData, saved_data::LandscapeTextureKinds},
    landscapes::Landscape::Landscape,
    models::Model::Model,
    shapes::{Cube::Cube, Pyramid::Pyramid},
};

use super::Grid::GridConfig;
use super::PlayerCharacter::{PlayerCharacter, NPC};
use super::SimpleGizmo::AxisArrow;
use super::{
    Grid::Grid,
    Rays::{cast_ray_at_components, create_ray_from_mouse},
    SimpleCamera::SimpleCamera,
    SimpleGizmo::SimpleGizmo,
    Viewport::Viewport,
};

#[derive(Debug, Clone)]
pub struct MouseState {
    pub is_first_mouse: bool,
    pub last_mouse_x: f64,
    pub last_mouse_y: f64,
    pub right_mouse_pressed: bool,
    pub drag_started: bool,
    pub is_dragging: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct WindowSize {
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

// Define all possible edit operations
#[derive(Debug)]
pub enum ObjectProperty {
    Width(f32),
}

#[derive(Debug)]
pub struct ObjectEditConfig {
    pub object_id: Uuid,
    pub field_name: String,
    pub old_value: ObjectProperty,
    pub new_value: ObjectProperty,
    // pub signal: RwSignal<String>,
}

#[derive(Clone, Debug)]
pub struct ObjectConfig {
    pub id: Uuid,
    pub name: String,
    pub position: (f32, f32, f32),
}

// #[derive(std::ops::DerefMut)]
pub struct RendererState {
    pub viewport: Arc<Mutex<Viewport>>,
    pub cubes: Vec<Cube>,
    pub pyramids: Vec<Pyramid>,
    pub grids: Vec<Grid>,
    pub models: Vec<Model>,
    pub landscapes: Vec<Landscape>,
    pub skeleton_parts: Vec<SkeletonRenderPart>, // will contain buffers and the like

    // wgpu
    pub model_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_render_mode_buffer: Arc<wgpu::Buffer>,
    pub color_render_mode_buffer: Arc<wgpu::Buffer>,
    pub camera_uniform_buffer: Arc<wgpu::Buffer>,
    pub camera_bind_group: Arc<wgpu::BindGroup>,

    // state
    pub project_selected: Option<Uuid>,
    pub current_view: String,
    pub object_selected: Option<Uuid>,
    pub object_selected_kind: Option<ComponentKind>,
    pub object_selected_data: Option<ComponentData>,

    pub gizmo: SimpleGizmo,

    // physics
    pub gravity: Vector<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhaseMultiSap,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,

    // characters
    pub player_character: PlayerCharacter,
    pub npcs: Vec<NPC>,

    pub current_modifiers: ModifiersState,
    pub mouse_state: MouseState,
    pub last_ray: Option<Ray>,
    pub ray_intersecting: bool,
    pub ray_intersection: Option<RapierPoint<f32>>,
    pub ray_component_id: Option<Uuid>,
    pub dragging_gizmo: bool,

    pub last_movement_time: Option<Instant>,
    pub last_frame_time: Option<Instant>,
}

// impl<'a> RendererState<'a> {
impl RendererState {
    pub async fn new(
        // device: Arc<wgpu::Device>,
        // queue: Arc<wgpu::Queue>,
        viewport: Arc<Mutex<Viewport>>,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        model_bind_group_layout: Arc<wgpu::BindGroupLayout>,
        texture_bind_group_layout: Arc<wgpu::BindGroupLayout>,
        texture_render_mode_buffer: Arc<wgpu::Buffer>,
        color_render_mode_buffer: Arc<wgpu::Buffer>,
        camera_uniform_buffer: Arc<wgpu::Buffer>,
        camera_bind_group: Arc<wgpu::BindGroup>,
        camera: &SimpleCamera,
        window_width: u32,
        window_height: u32,
        camera_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    ) -> Self {
        // create the utility grid(s)
        let mut grids = Vec::new();
        grids.push(Grid::new(
            &device,
            &model_bind_group_layout,
            &texture_bind_group_layout,
            &color_render_mode_buffer,
            GridConfig {
                width: 200.0,
                depth: 200.0,
                spacing: 4.0,
                line_thickness: 0.1,
            },
        ));
        grids.push(Grid::new(
            &device,
            &model_bind_group_layout,
            &texture_bind_group_layout,
            &color_render_mode_buffer,
            GridConfig {
                width: 200.0,
                depth: 200.0,
                spacing: 1.0,
                line_thickness: 0.025,
            },
        ));

        let mut cubes = Vec::new();
        cubes.push(Cube::new(&device, &model_bind_group_layout));

        let mut pyramids = Vec::new();
        // pyramids.push(Pyramid::new(device, bind_group_layout, color_render_mode_buffer));
        // add more pyramids as needed

        let mut models = Vec::new();

        let mut landscapes = Vec::new();

        let mut skeleton_parts = Vec::new();

        // let gizmo = TestTransformGizmo::new(
        //     &device,
        //     camera,
        //     WindowSize {
        //         width: window_width,
        //         height: window_height,
        //     },
        //     camera_bind_group_layout.clone(), // TODO: check if right layout
        //     color_render_mode_buffer.clone(),
        //     texture_bind_group_layout.clone(),
        // );

        let gizmo = SimpleGizmo::new(
            &device,
            camera,
            WindowSize {
                width: window_width,
                height: window_height,
            },
            camera_bind_group_layout.clone(), // TODO: check if right layout
            color_render_mode_buffer.clone(),
            texture_bind_group_layout.clone(),
        );

        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let mut player_character = PlayerCharacter::new();

        let rigid_body_handle = rigid_body_set.insert(player_character.movement_rigid_body.clone());
        player_character.movement_rigid_body_handle = Some(rigid_body_handle);

        // now associate rigidbody with collider
        let collider_handle = collider_set.insert_with_parent(
            player_character.movement_collider.clone(),
            rigid_body_handle,
            &mut rigid_body_set,
        );
        player_character.collider_handle = Some(collider_handle);

        Self {
            cubes,
            pyramids,
            grids,
            models,
            landscapes,
            skeleton_parts,

            // device,
            // queue,
            viewport,
            model_bind_group_layout,
            texture_bind_group_layout,
            texture_render_mode_buffer,
            color_render_mode_buffer,
            camera_uniform_buffer,
            camera_bind_group,

            project_selected: None,
            current_view: "welcome".to_string(),
            object_selected: None,
            object_selected_kind: None,
            object_selected_data: None,

            gizmo,
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            query_pipeline,
            rigid_body_set,
            collider_set,
            player_character,

            current_modifiers: ModifiersState::empty(),
            mouse_state: MouseState {
                last_mouse_x: 0.0,
                last_mouse_y: 0.0,
                is_first_mouse: true,
                right_mouse_pressed: false,
                drag_started: false,
                is_dragging: false,
            },
            last_ray: None,
            ray_intersecting: false,
            ray_component_id: None,
            ray_intersection: None,
            dragging_gizmo: false,
            last_movement_time: None,
            last_frame_time: None,
            npcs: Vec::new(),
        }
    }

    // pub fn step_physics_pipeline(&mut self) {
    //     // Calculate delta time
    //     let now = std::time::Instant::now();
    //     let dt = if self.last_frame_time.is_some() {
    //         (now - self.last_frame_time.expect("Couldn't get time")).as_secs_f32()
    //     } else {
    //         0.0
    //     };
    //     self.last_frame_time = Some(now);

    //     // println!("processing physics...");
    //     let physics_hooks = ();
    //     let event_handler = ();

    //     self.physics_pipeline.step(
    //         &self.gravity,
    //         &self.integration_parameters,
    //         &mut self.island_manager,
    //         &mut self.broad_phase,
    //         &mut self.narrow_phase,
    //         &mut self.rigid_body_set,
    //         &mut self.collider_set,
    //         &mut self.impulse_joint_set,
    //         &mut self.multibody_joint_set,
    //         &mut self.ccd_solver,
    //         Some(&mut self.query_pipeline),
    //         &physics_hooks,
    //         &event_handler,
    //     );

    //     // update visuals accordingly
    //     for (rigid_body_handle, rigid_body) in self.rigid_body_set.iter() {
    //         // println!("rigid body");
    //         // Get the physics position
    //         let physics_position = rigid_body.position();

    //         // Convert Rapier's Isometry as needed
    //         let position = physics_position.translation.vector;
    //         let rotation = physics_position.rotation;
    //         let euler = rotation.euler_angles(); // Returns (roll, pitch, yaw)
    //         let component_id = Uuid::from_u128(rigid_body.user_data);

    //         let instance_model_data = self
    //             .models
    //             .iter_mut()
    //             .find(|m| m.id == component_id.to_string());

    //         let instance_npc_data = self
    //             .npcs
    //             .iter_mut()
    //             .find(|m| m.model_id == component_id.to_string());

    //         // Update camera to follow physics body
    //         if let Some(rb_handle) = self.player_character.movement_rigid_body_handle {
    //             if let Some(rb) = self.rigid_body_set.get(rb_handle) {
    //                 let pos = rb.translation();

    //                 let mut camera = get_camera();
    //                 camera.position = Point3::new(pos.x, pos.y + 0.9, pos.z); // Add eye height
    //             }
    //         }

    //         if instance_model_data.is_some() {
    //             // println!("Processing model physics!");
    //             let instance_model_data =
    //                 instance_model_data.expect("Couldn't get instance_model_data");

    //             instance_model_data.meshes.iter_mut().for_each(|mesh| {
    //                 mesh.transform
    //                     .update_position([position.x, position.y, position.z]);
    //                 mesh.transform.update_rotation([euler.0, euler.1, euler.2]);
    //             });

    //             if instance_npc_data.is_some() {
    //                 let instance_npc_data =
    //                     instance_npc_data.expect("Couldn't get instance_npc_data");

    //                 let mut first_mesh = instance_model_data.meshes.get_mut(0).expect("Test");

    //                 instance_npc_data.test_behavior.update(
    //                     // renderer_state,
    //                     &mut self.rigid_body_set,
    //                     &self.collider_set,
    //                     &self.query_pipeline,
    //                     first_mesh.rigid_body_handle.expect("Couldn't get it"), // TODO / FIX assumes 1
    //                     &first_mesh.rapier_collider,
    //                     &mut first_mesh.transform,
    //                     dt,
    //                 );
    //             }
    //         }

    //         // landscapes are static anyway?
    //         let instance_landscape_data = self
    //             .landscapes
    //             .iter_mut()
    //             .find(|m| m.id == component_id.to_string());

    //         if instance_landscape_data.is_some() {
    //             let mut instance_landscape_data =
    //                 instance_landscape_data.expect("Couldn't get instance_landscape_data");

    //             instance_landscape_data
    //                 .transform
    //                 .update_position([position.x, position.y, position.z]);
    //             instance_landscape_data
    //                 .transform
    //                 .update_rotation([euler.0, euler.1, euler.2]);
    //         }
    //     }
    // }

    pub fn step_physics_pipeline(&mut self) {
        // Calculate delta time
        let now = std::time::Instant::now();
        let dt = if let Some(last_time) = self.last_frame_time {
            (now - last_time).as_secs_f32()
        } else {
            0.0
        };
        self.last_frame_time = Some(now);

        // Step the physics pipeline
        let physics_hooks = ();
        let event_handler = ();

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        // Collect all the necessary data first
        let physics_updates: Vec<(Uuid, nalgebra::Vector3<f32>, (f32, f32, f32))> = self
            .rigid_body_set
            .iter()
            .map(|(_, rigid_body)| {
                let physics_position = rigid_body.position();
                let position = physics_position.translation.vector;
                let rotation = physics_position.rotation;
                let euler = rotation.euler_angles();
                let component_id = Uuid::from_u128(rigid_body.user_data);
                (component_id, position, euler)
            })
            .collect();

        // Update camera position if needed
        if let Some(rb_handle) = self.player_character.movement_rigid_body_handle {
            if let Some(rb) = self.rigid_body_set.get(rb_handle) {
                let pos = rb.translation();
                let mut camera = get_camera();
                camera.position = Point3::new(pos.x, pos.y + 0.9, pos.z);
            }
        }

        // Now process all updates without borrowing rigid_body_set
        for (component_id, position, euler) in physics_updates {
            // Update models
            if let Some(instance_model_data) = self
                .models
                .iter_mut()
                .find(|m| m.id == component_id.to_string())
            {
                instance_model_data.meshes.iter_mut().for_each(|mesh| {
                    mesh.transform
                        .update_position([position.x, position.y, position.z]);
                    mesh.transform.update_rotation([euler.0, euler.1, euler.2]);
                });

                // Handle NPC updates
                if let Some(instance_npc_data) = self
                    .npcs
                    .iter_mut()
                    .find(|m| m.model_id == component_id.to_string())
                {
                    if let Some(first_mesh) = instance_model_data.meshes.get_mut(0) {
                        let current_stamina = 100.0;
                        instance_npc_data.test_behavior.update(
                            &mut self.rigid_body_set,
                            &self.collider_set,
                            &self.query_pipeline,
                            first_mesh
                                .rigid_body_handle
                                .expect("Couldn't get rigid body handle"),
                            self.player_character
                                .movement_rigid_body_handle
                                .expect("Couldn't get rigid body handle"),
                            &first_mesh.rapier_collider,
                            &mut first_mesh.transform,
                            current_stamina,
                            dt,
                        );
                    }
                }
            }

            // Update landscapes
            if let Some(instance_landscape_data) = self
                .landscapes
                .iter_mut()
                .find(|m| m.id == component_id.to_string())
            {
                instance_landscape_data
                    .transform
                    .update_position([position.x, position.y, position.z]);
                instance_landscape_data
                    .transform
                    .update_rotation([euler.0, euler.1, euler.2]);
            }
        }
    }

    // Usage in your main update/render loop:
    pub fn update_rays(
        &mut self,
        mouse_pos: (f32, f32),
        camera: &SimpleCamera,
        screen_width: u32,
        screen_height: u32,
    ) -> Ray {
        // Create ray from mouse position
        let ray = create_ray_from_mouse(mouse_pos, camera, screen_width, screen_height);

        // println!("collider set {:?}", self.collider_set.len());

        // Cast ray and check for intersection
        if let Some((collider_handle, toi)) = cast_ray_at_components(
            &ray,
            &self.query_pipeline,
            &self.rigid_body_set,
            &self.collider_set,
        ) {
            // println!("Colliding!");
            // Get the collider
            let collider = &self.collider_set[collider_handle];

            // Get intersection point in world space
            let intersection_point = ray.point_at(toi);

            let component_id = Uuid::from_u128(collider.user_data);

            self.ray_intersecting = true;
            self.ray_intersection = Some(intersection_point);
            self.ray_component_id = Some(component_id);
        } else {
            self.ray_intersecting = false;
            // keep stale data for sticky translation
            // self.ray_intersection = None;
            // self.ray_component_id = None;
        }

        ray
    }

    pub fn update_rapier(&mut self) {
        self.query_pipeline.update(&self.collider_set);
    }

    pub fn add_arrow_colliders(&mut self) {
        self.gizmo.arrows.iter_mut().for_each(|arrow| {
            println!("adding arrow collider");
            let collider_handle = self.collider_set.insert(arrow.rapier_collider.clone());
            arrow.collider_handle = Some(collider_handle);
        });
    }

    pub fn update_arrow_collider_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        self.gizmo.arrows.iter().for_each(|arrow| {
            // Create translation vector based on the arrow's axis
            let translation = match arrow.axis {
                0 => vector![position[0], position[1], position[2]], // X axis
                1 => vector![position[0], position[1], position[2]], // Y axis
                _ => vector![position[0], position[1], position[2]], // Z axis
            };

            let isometry =
                nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

            if let Some(collider) = self
                .collider_set
                .get_mut(arrow.collider_handle.expect("Couldn't get collider handle"))
            {
                collider.set_position(isometry);
                println!(
                    "Updated collider for axis {}: pos={:?}",
                    arrow.axis, translation
                );
            }
        });
    }

    pub fn update_player_character_position(&mut self, translation: Vector3<f32>, delta_time: f32) {
        let mut camera = get_camera();
        // Collision filter (typically you want to collide with everything except other characters)
        let filter = QueryFilter::default()
            .exclude_rigid_body(
                self.player_character
                    .movement_rigid_body_handle
                    .expect("Couldn't get rigid body handle"),
            )
            .exclude_collider(
                self.player_character
                    .collider_handle
                    .expect("Couldn't get collider handle"),
            )
            .exclude_sensors(); // Typically don't collide with trigger volumes

        // Current character position
        let character_pos = Isometry3::translation(
            camera.position.x,
            camera.position.y - 0.9, // Offset by half height to put camera at top
            camera.position.z,
        );

        self.player_character.character_controller.move_shape(
            delta_time,
            &self.rigid_body_set,
            &self.collider_set,
            &self.query_pipeline,
            self.player_character.movement_collider.shape(),
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

    pub fn update_player_collider_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        // Create translation vector based on the arrow's axis
        let translation = vector![position[0], position[1], position[2]];

        let isometry =
            nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

        if let Some(collider) = self.collider_set.get_mut(
            self.player_character
                .collider_handle
                .expect("Couldn't get mesh collider handle"),
        ) {
            collider.set_position(isometry);
        }
    }

    pub fn update_model_collider_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        self.models.iter().for_each(|model| {
            model.meshes.iter().for_each(|mesh| {
                // Create translation vector based on the arrow's axis
                let translation = vector![position[0], position[1], position[2]];

                let isometry =
                    nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

                if let Some(collider) = self.collider_set.get_mut(
                    mesh.collider_handle
                        .expect("Couldn't get mesh collider handle"),
                ) {
                    collider.set_position(isometry);
                }
            });
        });
    }

    pub fn update_player_rigidbody_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        // Create translation vector based on the arrow's axis
        let translation = vector![position[0], position[1], position[2]];

        let isometry =
            nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

        if let Some(rigidbody) = self.rigid_body_set.get_mut(
            self.player_character
                .movement_rigid_body_handle
                .expect("Couldn't get mesh rigidbody handle"),
        ) {
            rigidbody.set_position(isometry, true);
        }
    }

    pub fn update_model_rigidbody_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        self.models.iter().for_each(|model| {
            model.meshes.iter().for_each(|mesh| {
                // Create translation vector based on the arrow's axis
                let translation = vector![position[0], position[1], position[2]];

                let isometry =
                    nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

                if let Some(rigidbody) = self.rigid_body_set.get_mut(
                    mesh.rigid_body_handle
                        .expect("Couldn't get mesh collider handle"),
                ) {
                    rigidbody.set_position(isometry, true);
                }
            });
        });
    }

    pub fn update_landscape_collider_position(
        &mut self,
        //arrows: &[AxisArrow; 3],
        position: [f32; 3],
    ) {
        self.landscapes.iter().for_each(|landscape| {
            // Create translation vector based on the arrow's axis
            let translation = vector![position[0], position[1], position[2]];

            let isometry =
                nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

            if let Some(collider) = self.collider_set.get_mut(
                landscape
                    .collider_handle
                    .expect("Couldn't get landscape collider handle"),
            ) {
                collider.set_position(isometry);
            }
        });
    }

    pub fn add_collider(&mut self, component_id: String, component_kind: ComponentKind) {
        match component_kind {
            ComponentKind::Landscape => {
                let renderer_landscape = self
                    .landscapes
                    .iter_mut()
                    .find(|l| l.id == component_id.clone())
                    .expect("Couldn't get Renderer Landscape");

                // let collider_handle = self
                //     .collider_set
                //     .insert(renderer_landscape.rapier_heightfield.clone());
                // renderer_landscape.collider_handle = Some(collider_handle);

                let rigid_body_handle = self
                    .rigid_body_set
                    .insert(renderer_landscape.rapier_rigidbody.clone());
                renderer_landscape.rigid_body_handle = Some(rigid_body_handle);

                // now associate rigidbody with collider
                let collider_handle = self.collider_set.insert_with_parent(
                    renderer_landscape.rapier_heightfield.clone(),
                    rigid_body_handle,
                    &mut self.rigid_body_set,
                );
                renderer_landscape.collider_handle = Some(collider_handle);
            }
            ComponentKind::Model => {
                let renderer_model = self
                    .models
                    .iter_mut()
                    .find(|l| l.id == component_id.clone())
                    .expect("Couldn't get Renderer Model");

                renderer_model.meshes.iter_mut().for_each(|mesh| {
                    // let collider_handle = self.collider_set.insert(mesh.rapier_collider.clone());
                    // mesh.collider_handle = Some(collider_handle);

                    let rigid_body_handle =
                        self.rigid_body_set.insert(mesh.rapier_rigidbody.clone());
                    mesh.rigid_body_handle = Some(rigid_body_handle);

                    // now associate rigidbody with collider
                    let collider_handle = self.collider_set.insert_with_parent(
                        mesh.rapier_collider.clone(),
                        rigid_body_handle,
                        &mut self.rigid_body_set,
                    );
                    mesh.collider_handle = Some(collider_handle);
                });
            }
        }
    }

    pub fn add_model(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        model_component_id: &String,
        bytes: &Vec<u8>,
        isometry: Isometry3<f32>,
    ) {
        let model = Model::from_glb(
            model_component_id,
            bytes,
            device,
            queue,
            &self.model_bind_group_layout,
            &self.texture_bind_group_layout,
            &self.texture_render_mode_buffer,
            &self.color_render_mode_buffer,
            isometry,
        );

        // test npcs
        self.npcs.push(NPC::new(model.id.clone()));

        self.models.push(model);
    }

    pub fn add_landscape(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        landscapeComponentId: &String,
        data: &LandscapePixelData,
        position: [f32; 3],
    ) {
        let landscape = Landscape::new(
            landscapeComponentId,
            data,
            device,
            queue,
            &self.model_bind_group_layout,
            &self.texture_bind_group_layout,
            // &self.texture_render_mode_buffer,
            &self.color_render_mode_buffer,
            position,
        );

        self.landscapes.push(landscape);
    }

    pub fn update_landscape_texture(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        landscape_id: String,
        kind: LandscapeTextureKinds,
        texture: Texture,
        maskKind: LandscapeTextureKinds,
        mask: Texture,
    ) {
        if let Some(landscape) = self.landscapes.iter_mut().find(|l| l.id == landscape_id) {
            landscape.update_texture(
                device,
                queue,
                &self.texture_bind_group_layout,
                &self.texture_render_mode_buffer,
                &self.color_render_mode_buffer,
                kind,
                &texture,
            );
            landscape.update_texture(
                device,
                queue,
                &self.texture_bind_group_layout,
                &self.texture_render_mode_buffer,
                &self.color_render_mode_buffer,
                maskKind,
                &mask,
            );
        }
    }

    pub fn add_skeleton_part(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        partComponentId: &String,
        position: [f32; 3],
        joints: Vec<Joint>,
        joint_positions: &HashMap<String, Point3<f32>>,
    ) {
        let mut skeleton_part = SkeletonRenderPart::new(partComponentId.to_string());
        skeleton_part.create_bone_segments(
            device,
            &self.model_bind_group_layout,
            joints,
            joint_positions,
        );

        self.skeleton_parts.push(skeleton_part);
    }
}

static RENDERING_PAUSED: AtomicBool = AtomicBool::new(false);

// Pause rendering
pub fn pause_rendering() {
    RENDERING_PAUSED.store(true, Ordering::SeqCst);
}

// Resume rendering
pub fn resume_rendering() {
    RENDERING_PAUSED.store(false, Ordering::SeqCst);
}

// Check if rendering is paused
pub fn is_rendering_paused() -> bool {
    RENDERING_PAUSED.load(Ordering::SeqCst)
}

// mutex approach

// // Global mutable static variable for RendererState protected by a Mutex
// pub static mut RENDERER_STATE: Option<Mutex<RendererState>> = None;

// thread_local! {
//     pub static RENDERER_STATE_INIT: std::cell::Cell<bool> = std::cell::Cell::new(false);
// }

// // Function to initialize the RendererState
// pub fn initialize_renderer_state(state: RendererState) {
//     unsafe {
//         RENDERER_STATE = Some(Mutex::new(state));
//     }
//     RENDERER_STATE_INIT.with(|init| {
//         init.set(true);
//     });
// }

// // Function to get a mutable reference to the RendererState
// pub fn get_renderer_state() -> Arc<&'static Mutex<RendererState>> {
//     RENDERER_STATE_INIT.with(|init| {
//         if !init.get() {
//             panic!("RendererState not initialized");
//         }
//     });

//     unsafe { Arc::new(RENDERER_STATE.as_ref().unwrap()) }
// }
