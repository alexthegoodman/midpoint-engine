use uuid::Uuid;

use crate::core::Texture::Texture;
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

use super::{Grid::Grid, Viewport::Viewport};

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
        }
    }

    pub fn add_model(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, bytes: &Vec<u8>) {
        let model = Model::from_glb(
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
