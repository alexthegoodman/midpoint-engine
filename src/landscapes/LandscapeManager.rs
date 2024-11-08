use std::{
    collections::HashMap,
    sync::{Arc, Mutex, MutexGuard},
};

use nalgebra::{Matrix4, Vector, Vector2, Vector3};
use tokio::sync::mpsc::error::TrySendError;
use wgpu::util::DeviceExt;

use crate::{
    core::{
        RendererState::RendererState,
        Transform::{matrix4_to_raw_array, Transform},
    },
    helpers::{
        landscapes::{get_landscape_pixels, get_upscaled_landscape_pixels, LandscapePixelData},
        utilities::get_common_os_dir,
    },
};

use super::Landscape::Landscape;

// Represents a single landscape tile0
pub struct LandscapeTile {
    pub landscape: Landscape,
    pub position: Vector2<f32>, // Grid position (x, z)
    pub is_loaded: bool,
}

// Manages the landscape streaming system
pub struct LandscapeManager {
    pub id: String,
    pub asset_id: String,
    pub project_id: String,
    pub tiles: HashMap<(i32, i32), LandscapeTile>,
    pub grid_size: i32,               // tiles per side
    pub tile_size: f32,               // Size of each tile in world units (1024)
    pub heightmap_resolution: u32,    // Resolution of each heightmap (1024)
    pub current_center: Vector2<f32>, // Current center position in grid coordinates
    // device: wgpu::Device,
    // queue: wgpu::Queue,
    // pub initial_position: [f32; 3],
    pub transform: Transform,
    pub bind_group: wgpu::BindGroup,
    pub load_manager: LandscapeLoadManager, // Add the loader
}

impl LandscapeManager {
    pub fn new(
        device: &wgpu::Device,
        bind_group_layout: Arc<wgpu::BindGroupLayout>,
        project_id: String,
        asset_id: String,
        component_id: &String,
        initial_position: [f32; 3],
        upscaled_count: &usize,
    ) -> Self {
        let upscaled_count = *upscaled_count as f32;
        let grid_size = if upscaled_count > 0.0 {
            upscaled_count.sqrt()
        } else {
            1.0
        };

        let load_manager =
            LandscapeLoadManager::new(9, grid_size, project_id.clone(), asset_id.clone());

        // set uniform buffer for transforms
        let empty_buffer = Matrix4::<f32>::identity();
        let raw_matrix = matrix4_to_raw_array(&empty_buffer);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Model GLB Uniform Buffer"),
            contents: bytemuck::cast_slice(&raw_matrix),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
            label: None,
        });

        Self {
            id: component_id.to_string(),
            asset_id: asset_id.clone(),
            project_id: project_id.clone(),
            tiles: HashMap::new(),
            grid_size: grid_size as i32, // 256 heightmaps is 16 grid size, 16 heightmaps is 4 grid size, 1 is 1
            tile_size: 1024.0,
            heightmap_resolution: 1024,
            current_center: Vector2::identity(),
            // device,
            // queue,
            transform: Transform::new(
                Vector3::new(
                    initial_position[0],
                    initial_position[1],
                    initial_position[2],
                ),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 1.0, 1.0),
                uniform_buffer,
            ),
            bind_group,
            load_manager,
        }
    }

    // Convert world position to grid coordinates
    fn world_to_grid(&self, position: [f32; 3]) -> (i32, i32) {
        let x = (position[0] / self.tile_size).floor() as i32;
        let z = (position[2] / self.tile_size).floor() as i32;
        (x, z)
    }

    // Get the required tiles based on camera position
    fn get_required_tiles(&self, camera_pos: [f32; 3]) -> Vec<(i32, i32)> {
        let (center_x, center_z) = self.world_to_grid(camera_pos);
        let mut required = Vec::with_capacity(9);

        // Generate 3x3 grid centered on camera
        for x in -1..=1 {
            for z in -1..=1 {
                required.push((center_x + x, center_z + z));
            }
        }
        required
    }

    pub fn update(
        &mut self,
        // renderer_state: &mut RendererState,
        model_bind_group_layout: &wgpu::BindGroupLayout,
        texture_bind_group_layout: &wgpu::BindGroupLayout,
        color_render_mode_buffer: &wgpu::Buffer,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        project_id: String,
        camera_pos: [f32; 3],
        unscaled_filename: Option<String>,
    ) {
        // let renderer_state = renderer_state.lock().unwrap();

        let required_tiles = self.get_required_tiles(camera_pos);

        println!("required_tiles {:?}", required_tiles);

        // Request loads for missing tiles
        for (x, z) in &required_tiles {
            if !self.tiles.contains_key(&(*x, *z)) {
                // Normalize coordinates for file name (assuming 16x16 grid)
                let file_x = ((*x % self.grid_size) + self.grid_size) % self.grid_size; // This will wrap negative numbers to positive
                let file_z = ((*z % self.grid_size) + self.grid_size) % self.grid_size; // Same for z coordinate

                let file_name = format!("tile_{}_{}.tiff", file_x, file_z);
                self.load_manager.request_load(
                    (*x, *z),
                    file_name,
                    unscaled_filename.clone(),
                    self.asset_id.clone(),
                    project_id.clone(),
                );
            }
        }

        // Process completed loads
        for message in self.load_manager.update() {
            match message {
                LandscapeLoadMessage::HeightmapLoaded { grid_pos, data } => {
                    let position = Vector2::new(
                        self.transform.position[0] + (grid_pos.0 as f32 * self.tile_size),
                        self.transform.position[2] + (grid_pos.1 as f32 * self.tile_size),
                    );

                    let landscape = Landscape::new(
                        &self.id,
                        &data,
                        device,
                        queue,
                        &model_bind_group_layout,
                        &texture_bind_group_layout,
                        // &self.texture_render_mode_buffer,
                        &color_render_mode_buffer,
                        [position.x, self.transform.position[1], position.y], // y is z here
                    );

                    // Create the landscape tile
                    let tile = LandscapeTile {
                        landscape,
                        position: Vector2::new(grid_pos.0 as f32, grid_pos.1 as f32),
                        is_loaded: true,
                    };

                    println!("Inserting new tile!");

                    // Store the tile
                    self.tiles.insert(grid_pos, tile);
                }
                LandscapeLoadMessage::LoadError { grid_pos, error } => {
                    eprintln!("Failed to load landscape tile at {:?}: {}", grid_pos, error);
                }
            }
        }

        // Unload distant tiles
        let (center_x, center_z) = self.world_to_grid(camera_pos);
        self.tiles.retain(|&pos, _| {
            let (x, z) = pos;
            let distance = ((x - center_x).pow(2) + (z - center_z).pow(2)) as f32;

            distance <= 4.0
        });

        // drop(renderer_state);
    }
}

use std::collections::VecDeque;
use tokio::sync::mpsc;

// Message sent from loader to renderer
pub enum LandscapeLoadMessage {
    HeightmapLoaded {
        grid_pos: (i32, i32),
        // heightmap: Vec<f32>,
        data: LandscapePixelData,
    },
    LoadError {
        grid_pos: (i32, i32),
        error: String,
    },
}

// Message sent to loader
#[derive(Debug)]
pub struct LoadRequest {
    grid_pos: (i32, i32),
    file_name: String,
    unscaled_filename: Option<String>,
    project_id: String,
    asset_id: String,
}

pub struct LandscapeLoadManager {
    loader_tx: mpsc::Sender<LoadRequest>,
    receiver_rx: mpsc::Receiver<LandscapeLoadMessage>,
    // Track requested but not yet loaded tiles
    pending_loads: VecDeque<(
        i32, // grid pos
        i32,
        String,         // file path
        Option<String>, // unscaled_filename:
        String,         // asset_id:
        String,         // project_id:
    )>,
    max_concurrent_loads: usize,
    current_loads: usize,
}

impl LandscapeLoadManager {
    pub fn new(
        max_concurrent_loads: usize,
        grid_size: f32,
        project_id: String,
        asset_id: String,
    ) -> Self {
        let (loader_tx, mut loader_rx) = mpsc::channel::<LoadRequest>(32); // Channel for load requests
        let (receiver_tx, receiver_rx) = mpsc::channel::<LandscapeLoadMessage>(32); // Channel for completed loads

        // Spawn the loader task
        tokio::spawn(async move {
            while let Some(request) = loader_rx.recv().await {
                let tx = receiver_tx.clone();

                // Spawn a new task for each load request
                tokio::spawn(async move {
                    // let result = load_heightmap_file(&request.file_path).await;
                    // let message = match result {
                    //     Ok(heightmap) => LandscapeLoadMessage::HeightmapLoaded {
                    //         grid_pos: request.grid_pos,
                    //         heightmap,
                    //     },
                    //     Err(e) => LandscapeLoadMessage::LoadError {
                    //         grid_pos: request.grid_pos,
                    //         error: e.to_string(),
                    //     },
                    // };

                    let data = if grid_size == 1.0 {
                        let unscaled_filename = request
                            .unscaled_filename
                            .expect("Couldn't get unscaled landscape filename");
                        get_landscape_pixels(
                            request.project_id,
                            request.asset_id,
                            unscaled_filename,
                        )
                    } else {
                        // let upscaled_filename =
                        //     format!("tile_{}_{}.tiff", request.grid_pos.0, request.grid_pos.1);
                        get_upscaled_landscape_pixels(
                            request.project_id,
                            request.asset_id,
                            request.file_name,
                        )
                    };

                    let message = LandscapeLoadMessage::HeightmapLoaded {
                        grid_pos: (request.grid_pos.0, request.grid_pos.1),
                        data,
                    };

                    // Ignore send errors - receiver might be gone
                    let _ = tx.send(message).await;
                });
            }
        });

        Self {
            loader_tx,
            receiver_rx,
            pending_loads: VecDeque::new(),
            max_concurrent_loads,
            current_loads: 0,
        }
    }

    // // Request a tile to be loaded
    // pub async fn request_load(&mut self, grid_pos: (i32, i32), file_path: String) {
    //     self.pending_loads.push_back(grid_pos);
    //     self.process_pending_loads().await;
    // }

    // Non-async version of request_load
    pub fn request_load(
        &mut self,
        grid_pos: (i32, i32),
        file_name: String,
        unscaled_filename: Option<String>,
        asset_id: String,
        project_id: String,
    ) -> bool {
        self.pending_loads.push_back((
            grid_pos.0,
            grid_pos.1,
            file_name.clone(),
            unscaled_filename.clone(),
            asset_id.clone(),
            project_id.clone(),
        ));

        match self.loader_tx.try_send(LoadRequest {
            grid_pos,
            file_name: file_name.clone(),
            unscaled_filename: unscaled_filename.clone(),
            asset_id: asset_id.clone(),
            project_id: project_id.clone(),
        }) {
            Ok(_) => {
                self.current_loads += 1;
                true
            }
            Err(TrySendError::Full(_)) => {
                // Channel is full, but request is queued in pending_loads
                true
            }
            Err(TrySendError::Closed(_)) => {
                // Channel is closed - system is shutting down
                false
            }
        }
    }

    // Process any pending load requests if we have capacity
    async fn process_pending_loads(&mut self) {
        while self.current_loads < self.max_concurrent_loads && !self.pending_loads.is_empty() {
            if let Some(grid_info) = self.pending_loads.pop_front() {
                // let file_path =
                //     format!("path/to/heightmaps/tile_{}_{}.tiff", grid_pos.0, grid_pos.1);

                if let Err(e) = self
                    .loader_tx
                    .send(LoadRequest {
                        grid_pos: (grid_info.0, grid_info.1),
                        file_name: grid_info.2,
                        unscaled_filename: grid_info.3,
                        asset_id: grid_info.4,
                        project_id: grid_info.5,
                    })
                    .await
                {
                    eprintln!("Failed to send load request: {}", e);
                    continue;
                }

                self.current_loads += 1;
            }
        }
    }

    // // Check for completed loads - call this in your render loop
    // pub fn update(&mut self) -> Vec<LandscapeLoadMessage> {
    //     let mut completed_loads = Vec::new();

    //     while let Ok(message) = self.receiver_rx.try_recv() {
    //         self.current_loads -= 1;
    //         completed_loads.push(message);
    //     }

    //     // If we processed any loads, check if we can start more
    //     if !completed_loads.is_empty() {
    //         tokio::spawn(async move {
    //             self.process_pending_loads().await;
    //         });
    //     }

    //     completed_loads
    // }

    // Check for completed loads - call this in your render loop
    pub fn update(&mut self) -> Vec<LandscapeLoadMessage> {
        let mut completed_loads = Vec::new();

        while let Ok(message) = self.receiver_rx.try_recv() {
            self.current_loads -= 1;
            completed_loads.push(message);
        }

        // If we processed any loads, check if we can start more
        if !completed_loads.is_empty() {
            // Process directly without spawning a new task
            while self.current_loads < self.max_concurrent_loads && !self.pending_loads.is_empty() {
                if let Some(grid_info) = self.pending_loads.pop_front() {
                    // Using try_send instead of send to avoid async
                    match self.loader_tx.try_send(LoadRequest {
                        grid_pos: (grid_info.0, grid_info.1),
                        file_name: grid_info.2,
                        unscaled_filename: grid_info.3,
                        asset_id: grid_info.4,
                        project_id: grid_info.5,
                    }) {
                        Ok(_) => self.current_loads += 1,
                        Err(e) => eprintln!("Failed to send load request: {}", e),
                    }
                }
            }
        }

        completed_loads
    }
}
