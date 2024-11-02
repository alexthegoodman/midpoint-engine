use floem::keyboard::ModifiersState;
use rapier3d::prelude::*;
use rapier3d::prelude::{ColliderSet, QueryPipeline, RigidBodySet};
use uuid::Uuid;

use crate::{
    core::Texture::Texture,
    helpers::saved_data::{ComponentData, ComponentKind},
};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};

use crate::{
    helpers::{landscapes::LandscapePixelData, saved_data::LandscapeTextureKinds},
    landscapes::Landscape::Landscape,
    models::Model::Model,
    shapes::{Cube::Cube, Pyramid::Pyramid},
};

use super::{
    Grid::Grid,
    Rays::{cast_ray_at_components, create_ray_from_mouse},
    SimpleCamera::SimpleCamera,
    SimpleGizmo::SimpleGizmo,
    TestTransformGizmo::TestTransformGizmo,
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

    // pub device: Arc<wgpu::Device>,
    // pub queue: Arc<wgpu::Queue>,
    pub model_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    pub texture_render_mode_buffer: Arc<wgpu::Buffer>,
    pub color_render_mode_buffer: Arc<wgpu::Buffer>,
    pub camera_uniform_buffer: Arc<wgpu::Buffer>,
    pub camera_bind_group: Arc<wgpu::BindGroup>,

    pub project_selected: Option<Uuid>,
    pub current_view: String,
    pub object_selected: Option<Uuid>,
    pub object_selected_kind: Option<ComponentKind>,
    pub object_selected_data: Option<ComponentData>,

    // pub gizmo: TestTransformGizmo,
    pub gizmo: SimpleGizmo,
    pub query_pipeline: QueryPipeline,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,

    pub current_modifiers: ModifiersState,
    pub mouse_state: MouseState,
    pub last_ray: Option<Ray>,
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
        ));

        let mut cubes = Vec::new();
        cubes.push(Cube::new(&device, &model_bind_group_layout));

        let mut pyramids = Vec::new();
        // pyramids.push(Pyramid::new(device, bind_group_layout, color_render_mode_buffer));
        // add more pyramids as needed

        let mut models = Vec::new();

        let mut landscapes = Vec::new();

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

        Self {
            cubes,
            pyramids,
            grids,
            models,
            landscapes,

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
            query_pipeline: QueryPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),

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
        }
    }

    // Usage in your main update/render loop:
    pub fn update_rays(
        &mut self,
        mouse_pos: (f32, f32),
        camera: &SimpleCamera,
        screen_width: u32,
        screen_height: u32,
        // query_pipeline: &mut QueryPipeline,
        // rigid_body_set: &RigidBodySet,
        // collider_set: &ColliderSet,
        // gizmo: &mut SimpleGizmo,
    ) -> Ray {
        // Create ray from mouse position
        let ray = create_ray_from_mouse(mouse_pos, camera, screen_width, screen_height);

        // Cast ray and check for intersection
        if let Some((collider_handle, toi)) = cast_ray_at_components(
            &ray,
            &self.query_pipeline,
            &self.rigid_body_set,
            &self.collider_set,
        ) {
            println!("Colliding!");
            // Get the collider
            let collider = &self.collider_set[collider_handle];

            // Get intersection point in world space
            // let intersection_point = ray.point_at(intersection.toi);
            let intersection_point = ray.point_at(toi);

            let component_id = Uuid::from_u128(collider.user_data);

            // Position the gizmo at intersection point
            self.gizmo.transform.update_position([
                intersection_point.x,
                intersection_point.y,
                intersection_point.z,
            ]);

            // // Optionally, you might want to orient the gizmo based on the surface normal
            // if let Some(normal) = intersection.normal {
            //     // Create rotation from normal...
            // }
        } else {
            println!("not colliding...");
        }

        ray
    }

    pub fn update_rapier(&mut self) {
        self.query_pipeline.update(&self.collider_set);
    }

    pub fn add_collider(&mut self, component_id: String, component_kind: ComponentKind) {
        match component_kind {
            ComponentKind::Landscape => {
                let renderer_landscape = self
                    .landscapes
                    .iter()
                    .find(|l| l.id == component_id.clone())
                    .expect("Couldn't get Renderer Landscape");

                // TODO: expensive clone?
                self.collider_set
                    .insert(renderer_landscape.rapier_heightfield.clone());
            }
            ComponentKind::Model => {
                let renderer_model = self
                    .models
                    .iter()
                    .find(|l| l.id == component_id.clone())
                    .expect("Couldn't get Renderer Model");

                renderer_model.meshes.iter().for_each(|mesh| {
                    // TODO: expensive clone?
                    self.collider_set.insert(mesh.rapier_collider.clone());
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
        );

        self.models.push(model);
    }

    pub fn add_landscape(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        landscapeComponentId: &String,
        data: &LandscapePixelData,
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
