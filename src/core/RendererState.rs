use floem::keyboard::ModifiersState;
use gltf::json::camera;
use nalgebra::{Isometry3, Point3, Vector3};
use rapier3d::math::Point as RapierPoint;
use rapier3d::prelude::*;
use rapier3d::prelude::{ColliderSet, QueryPipeline, RigidBodySet};
use uuid::Uuid;
use wgpu::BindGroupLayout;

use crate::animations::motion_path::AnimationPlayback;
use crate::animations::render_skeleton::SkeletonRenderPart;
use crate::animations::skeleton::{AttachPoint, Joint, KinematicChain, PartConnection};
use crate::handlers::get_camera;
use crate::landscapes::QuadNode::QuadNode;
use crate::landscapes::TerrainManager::TerrainManager;
use crate::lighting::LightState::LightState;
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
use super::RotationGizmo::RotationGizmo;
use super::ScaleGizmo::ScaleGizmo;
use super::TranslationGizmo::AxisArrow;
use super::{
    Grid::Grid,
    Rays::{cast_ray_at_components, create_ray_from_mouse},
    SimpleCamera::SimpleCamera,
    TranslationGizmo::TranslationGizmo,
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
    pub skeleton_parts: Vec<SkeletonRenderPart>, // will contain buffers and the like
    pub terrain_managers: Vec<TerrainManager>,
    pub light_state: LightState,

    // animations
    pub active_animations: Vec<AnimationPlayback>,

    // wgpu
    pub model_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_render_mode_buffer: Arc<wgpu::Buffer>,
    pub color_render_mode_buffer: Arc<wgpu::Buffer>,
    pub camera_uniform_buffer: Arc<wgpu::Buffer>,
    pub camera_bind_group: Arc<wgpu::BindGroup>,
    pub light_bind_group_layout: Arc<wgpu::BindGroupLayout>,

    // state
    pub project_selected: Option<Uuid>,
    pub current_view: String,
    pub object_selected: Option<Uuid>,
    pub object_selected_kind: Option<ComponentKind>,
    pub object_selected_data: Option<ComponentData>,

    pub translation_gizmo: TranslationGizmo,
    pub rotation_gizmo: RotationGizmo,
    pub scale_gizmo: ScaleGizmo,
    pub active_gizmo: String,

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
    pub dragging_translation_gizmo: bool,
    pub gizmo_drag_axis: Option<u8>,

    pub last_movement_time: Option<Instant>,
    pub last_frame_time: Option<Instant>,

    pub navigation_speed: f32,
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
        light_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    ) -> Self {
        // let there be light!
        let light_state = LightState::new(device, &light_bind_group_layout);

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

        // let mut landscapes = Vec::new();

        let mut terrain_managers = Vec::new();

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

        let translation_gizmo = TranslationGizmo::new(
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

        let rotation_gizmo = RotationGizmo::new(
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

        let scale_gizmo = ScaleGizmo::new(
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
            // landscapes,
            skeleton_parts,
            terrain_managers,
            active_animations: Vec::new(),
            light_state,

            // device,
            // queue,
            viewport,
            model_bind_group_layout,
            texture_bind_group_layout,
            texture_render_mode_buffer,
            color_render_mode_buffer,
            camera_uniform_buffer,
            camera_bind_group,
            light_bind_group_layout,

            project_selected: None,
            current_view: "welcome".to_string(),
            object_selected: None,
            object_selected_kind: None,
            object_selected_data: None,

            translation_gizmo,
            rotation_gizmo,
            scale_gizmo,
            active_gizmo: "translate".to_string(),

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
            dragging_translation_gizmo: false,
            last_movement_time: None,
            last_frame_time: None,
            npcs: Vec::new(),
            gizmo_drag_axis: None,
            navigation_speed: 5.0,
        }
    }

    pub fn step_animations_pipeline(&mut self, queue: &wgpu::Queue) {
        for animation in &mut self.active_animations {
            // TODO: pass only relevant parts
            // update_skeleton_animation(&mut self.skeleton_parts, animation, queue);
            animation.update(&mut self.skeleton_parts, queue);
        }
    }

    pub fn step_physics_pipeline(&mut self, device: &wgpu::Device) {
        // Calculate delta time
        let now = std::time::Instant::now();
        let dt = if let Some(last_time) = self.last_frame_time {
            (now - last_time).as_secs_f32()
        } else {
            0.0
        };
        self.last_frame_time = Some(now);

        self.update_terrain_managers(device, dt);

        // println!("Before step:");
        // println!("  Bodies: {}", self.rigid_body_set.len());
        // println!("  Colliders: {}", self.collider_set.len());
        // println!("  Impulse Joints: {}", self.impulse_joint_set.len());
        // // println!("  Multibody Joints: {}", self.multibody_joint_set.len());
        // println!(
        //     "  Islands: {} {}",
        //     self.island_manager.active_dynamic_bodies().len(),
        //     self.island_manager.active_kinematic_bodies().len()
        // );

        let step_time = Instant::now();

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

        let step_duration = step_time.elapsed();
        // println!("  step_duration: {:?}", step_duration);

        let physics_update_time = Instant::now();

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

        let physics_update_duration = physics_update_time.elapsed();
        // println!(
        //     "  physics_update collect _duration: {:?}",
        //     physics_update_duration
        // );

        let physics_update_time = Instant::now();

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
            // just helps knowing terrain is where the physics are
            if let Some(terrain_manager) = self
                .terrain_managers
                .iter_mut()
                .find(|m| m.id == component_id.to_string())
            {
                terrain_manager
                    .transform
                    .update_position([position.x, position.y, position.z]);
                terrain_manager
                    .transform
                    .update_rotation([euler.0, euler.1, euler.2]);
            }
        }

        let physics_update_duration = physics_update_time.elapsed();
        // println!("  physics_update_duration: {:?}", physics_update_duration);
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

    // pub fn update_gizmo_state(&mut self, dragging: bool, axis: u8) {
    //     self.dragging_gizmo = true;
    //     self.gizmo_drag_axis = Some(axis);
    // }

    pub fn update_rapier(&mut self) {
        self.query_pipeline.update(&self.collider_set);
    }

    pub fn add_arrow_colliders(&mut self) {
        self.translation_gizmo.arrows.iter_mut().for_each(|arrow| {
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
        self.translation_gizmo.arrows.iter().for_each(|arrow| {
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
                // println!(
                //     "Updated collider for axis {}: pos={:?}",
                //     arrow.axis, translation
                // );
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
        self.terrain_managers.iter().for_each(|landscape| {
            // Create translation vector based on the arrow's axis
            let translation = vector![position[0], position[1], position[2]];

            let isometry =
                nalgebra::Isometry3::translation(translation.x, translation.y, translation.z);

            // if let Some(collider) = self.collider_set.get_mut(
            //     landscape
            //         .collider_handle
            //         .expect("Couldn't get landscape collider handle"),
            // ) {
            //     collider.set_position(isometry);
            // }

            // TODO: try this:
            // landscape.terrain_position = isometry
        });
    }

    pub fn add_collider(&mut self, component_id: String, component_kind: ComponentKind) {
        match component_kind {
            ComponentKind::Landscape => {
                // should be added as part of terrain manager
                // let terrain_manager = self
                //     .terrain_managers
                //     .iter_mut()
                //     .find(|l| l.id == component_id.clone())
                //     .expect("Couldn't get Renderer Landscape");

                // // TODO: need to collect current rigid bodies on nodes to add to rigidbodyset
                // // I should probably clear the set somehow before resetting it here

                // let rigid_body_handle = self
                //     .rigid_body_set
                //     .insert(current_node.rapier_rigidbody.clone());
                // current_node.rigid_body_handle = Some(rigid_body_handle);

                // // now associate rigidbody with collider
                // let collider_handle = self.collider_set.insert_with_parent(
                //     current_node.rapier_heightfield.clone(),
                //     rigid_body_handle,
                //     &mut self.rigid_body_set,
                // );
                // current_node.collider_handle = Some(collider_handle);
            }
            ComponentKind::Model => {
                let renderer_model = self
                    .models
                    .iter_mut()
                    .find(|l| l.id == component_id.clone())
                    .expect("Couldn't get Renderer Model");

                renderer_model.meshes.iter_mut().for_each(|mesh| {
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

    // pub fn add_landscape(
    //     &mut self,
    //     device: &wgpu::Device,
    //     queue: &wgpu::Queue,
    //     landscapeComponentId: &String,
    //     data: &LandscapePixelData,
    //     position: [f32; 3],
    // ) {
    //     let landscape = Landscape::new(
    //         landscapeComponentId,
    //         data,
    //         device,
    //         queue,
    //         &self.model_bind_group_layout,
    //         &self.texture_bind_group_layout,
    //         // &self.texture_render_mode_buffer,
    //         &self.color_render_mode_buffer,
    //         position,
    //     );

    //     self.landscapes.push(landscape);
    // }

    pub fn update_terrain_managers(&mut self, device: &wgpu::Device, dt: f32) {
        if self.terrain_managers.len() > 0 {
            let camera = get_camera();
            let terrain_manager = self
                .terrain_managers
                .get_mut(0)
                .expect("Couldn't get first terrain manager");

            // keep for debugging:
            // if let Some(rb_handle) = self.player_character.movement_rigid_body_handle {
            //     if let Some(rb) = self.rigid_body_set.get(rb_handle) {
            //         let character_pos = rb.position();

            //         // let camera = get_camera();
            //         // let character_pos = camera.position;

            //         // Cast slightly above character's feet
            //         let ray_start = character_pos * Point3::new(0.0, 0.1, 0.0);
            //         let ray_dir = Vector3::new(0.0, -1.0, 0.0);

            //         let collider_handle = find_first_collider_handle(&terrain_manager.root);

            //         println!(
            //             "Check collider handle {:?} {:?}",
            //             character_pos,
            //             collider_handle.is_some()
            //         );

            //         if let Some(handle) = collider_handle {
            //             // Use QueryPipeline for ray casting
            //             let hit = self.query_pipeline.cast_ray(
            //                 &self.rigid_body_set,
            //                 &self.collider_set,
            //                 &Ray::new(ray_start, ray_dir),
            //                 f32::MAX,
            //                 true,
            //                 QueryFilter::default().exclude_rigid_body(rb_handle), // Exclude the character's own collider
            //             );

            //             if let Some((_, intersection)) = hit {
            //                 let hit_point: nalgebra::OPoint<f32, nalgebra::Const<3>> =
            //                     ray_start + ray_dir * intersection;
            //                 println!("Ground intersection at: {:?}", hit_point);
            //                 println!("Character position: {:?}", character_pos);
            //                 println!("Distance to ground: {:?}", intersection);
            //             } else {
            //                 println!("no intersect!");
            //             }
            //         }
            //     }
            // }

            terrain_manager.update(
                [camera.position.x, camera.position.y, camera.position.z],
                device,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.island_manager,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set, // terrain_manager.terrain_position,
                // terrain_manager.id.clone(),
                dt,
                // &mut self.query_pipeline,
            );
        }
    }

    pub fn add_terrain_manager(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        projectId: String,
        landscapeAssetId: String,
        landscapeComponentId: String,
        landscapeFilename: String,
        position: [f32; 3],
    ) {
        let terrain_manager = TerrainManager::new(
            projectId,
            landscapeComponentId,
            landscapeAssetId,
            landscapeFilename,
            device,
            &self.model_bind_group_layout,
            position,
        );

        self.terrain_managers.push(terrain_manager);
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
        if let Some(terrain_manager) = self
            .terrain_managers
            .iter_mut()
            .find(|l| l.id == landscape_id)
        {
            terrain_manager.update_texture(
                device,
                queue,
                &self.texture_bind_group_layout,
                &self.texture_render_mode_buffer,
                &self.color_render_mode_buffer,
                kind,
                &texture,
            );
            terrain_manager.update_texture(
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
        k_chains: Vec<KinematicChain>,
        attach_points: Vec<AttachPoint>,
        joint_positions: &HashMap<String, Point3<f32>>,
        // joint_rotations: &HashMap<String, Vector3<f32>>,
        connection: Option<PartConnection>,
    ) {
        let mut skeleton_part = SkeletonRenderPart::new(partComponentId.to_string());
        skeleton_part.create_bone_segments(
            device,
            &self.model_bind_group_layout,
            joints,
            k_chains,
            attach_points,
            joint_positions,
            // joint_rotations,
            position,
            connection,
        );

        self.skeleton_parts.push(skeleton_part);
    }
}

fn find_first_collider_handle(node: &QuadNode) -> Option<ColliderHandle> {
    // Check if current node has a collider
    if let Some(handle) = node.collider_handle {
        return Some(handle);
    }

    // If not, recursively check children
    if let Some(ref children) = node.children {
        for child in children.iter() {
            if let Some(handle) = find_first_collider_handle(child) {
                return Some(handle);
            }
        }
    }

    None
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
