use nalgebra::{Isometry3, Matrix4, Point3, Vector3};
use rapier3d::math::{Point, Vector};
use rapier3d::parry::query::point;
use rapier3d::prelude::{point, ActiveCollisionTypes};
use rapier3d::prelude::{
    Collider, ColliderBuilder, ColliderHandle, InteractionGroups, RigidBody, RigidBodyBuilder,
    RigidBodyHandle,
};
use std::collections::HashMap;
use std::str::FromStr;
use uuid::Uuid;
use wgpu::util::{DeviceExt, TextureDataOrder};

use crate::core::Texture::Texture;
use crate::core::Transform::{matrix4_to_raw_array, Transform};
use crate::handlers::{get_camera, Vertex};
use crate::helpers::landscapes::LandscapePixelData;
use crate::helpers::saved_data::LandscapeTextureKinds;
use crate::helpers::utilities::get_common_os_dir;
use crate::landscapes::ChunkedTerrainGenerator::ChunkedTerrainGenerator;

pub struct Landscape {
    pub id: String,
    pub transform: Transform,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    pub bind_group: wgpu::BindGroup,
    // pub texture_bind_group: wgpu::BindGroup,
    pub texture_array: Option<wgpu::Texture>,
    pub texture_array_view: Option<wgpu::TextureView>,
    pub texture_bind_group: Option<wgpu::BindGroup>,
    pub rapier_heightfield: Collider,
    pub rapier_rigidbody: RigidBody,
    pub collider_handle: Option<ColliderHandle>,
    pub rigid_body_handle: Option<RigidBodyHandle>,
    pub heights: nalgebra::DMatrix<f32>,
    pub buffers: TerrainBuffers,
    pub lod_data: [LodData; 4], // hardcode 4 for now
    pub lod_distances: Vec<f32>,
    pub lod_sample_stride: usize,
}

const MAX_VERTICES: usize = 20000000;

impl Landscape {
    pub fn new(
        landscapeComponentId: &String,
        data: &LandscapePixelData,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bind_group_layout: &wgpu::BindGroupLayout,
        texture_bind_group_layout: &wgpu::BindGroupLayout,
        color_render_mode_buffer: &wgpu::Buffer,
        position: [f32; 3],
        project_id: String,
        asset_id: String,
    ) -> Self {
        // load actual vertices and indices (most important for now)
        let scale = 1.0;
        // let (vertices, indices) = Self::generate_terrain(data, scale);
        let lod_levels = vec![
            LodLevel { vertex_stride: 1 },  // Full detail
            LodLevel { vertex_stride: 4 },  // Quarter detail
            LodLevel { vertex_stride: 8 },  // 1/8th detail
            LodLevel { vertex_stride: 16 }, // 1/16th detail
        ];

        let lod_distances = vec![100.0, 200.0, 400.0, 800.0]; // Distances for each LOD level

        let camera = get_camera();

        // let (vertices, indices) = Self::generate_terrain_with_lod(
        //     data,
        //     [camera.position.x, camera.position.y, camera.position.z],
        //     lod_levels,
        //     lod_distances,
        //     scale,
        // );

        println!("Loading Landscape LOD levels...");

        let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
        let landscapes_dir = sync_dir.join(format!(
            "midpoint/projects/{}/landscapes/{}/heightmaps/upscaled",
            project_id, asset_id
        ));

        let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
            landscapes_dir
                .join("lod_1.mdbf")
                .to_str()
                .expect("Couldn't make string")
                .to_string(),
            1,
        );
        let (vertices_1, indices_1, stride_1) = chunked_terrain_generator
            .load_terrain()
            .expect("Couldn't load LOD 1");

        let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
            landscapes_dir
                .join("lod_4.mdbf")
                .to_str()
                .expect("Couldn't make string")
                .to_string(),
            4,
        );
        let (vertices_4, indices_4, stride_4) = chunked_terrain_generator
            .load_terrain()
            .expect("Couldn't load LOD 4");

        let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
            landscapes_dir
                .join("lod_8.mdbf")
                .to_str()
                .expect("Couldn't make string")
                .to_string(),
            8,
        );
        let (vertices_8, indices_8, stride_8) = chunked_terrain_generator
            .load_terrain()
            .expect("Couldn't load LOD 8");

        let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
            landscapes_dir
                .join("lod_16.mdbf")
                .to_str()
                .expect("Couldn't make string")
                .to_string(),
            16,
        );
        let (vertices_16, indices_16, stride_16) = chunked_terrain_generator
            .load_terrain()
            .expect("Couldn't load LOD 16");

        // DEBUG:
        // let vertices_1 = Vec::new();
        // let indices_1 = Vec::new();
        // let vertices_4 = Vec::new();
        // let indices_4 = Vec::new();
        // let vertices_8 = Vec::new();
        // let indices_8 = Vec::new();
        // let vertices_16 = Vec::new();
        // let indices_16 = Vec::new();

        let lod_distances = vec![2.0, 10.0, 20.0, 1024.0]; // Close, medium, far
        let lod_sample_stride = 10_000;
        let lod_strides = vec![1, 4, 8, 16];

        let mut buffers = TerrainBuffers::new(MAX_VERTICES);
        let lod_data = [
            LodData {
                vertices: vertices_1,
                indices: indices_1,
                stride: 1,
            },
            LodData {
                vertices: vertices_4,
                indices: indices_4,
                stride: 4,
            },
            LodData {
                vertices: vertices_8,
                indices: indices_8,
                stride: 8,
            },
            LodData {
                vertices: vertices_16,
                indices: indices_16.clone(),
                stride: 16,
            },
        ];

        // run just once in update_buffers?
        buffers.update(
            &lod_data,
            [camera.position.x, camera.position.y, camera.position.z],
            lod_distances.clone(),
            lod_sample_stride, // e.g., 100_000 for sampling every 100k vertices
        );

        println!("landscape buffers loaded {:?}", indices_16.clone().len());

        // Get the actual dimensions of your heightmap data
        let heightmap_width = data.rapier_heights.ncols() as f32;
        let heightmap_height = data.rapier_heights.nrows() as f32;

        // Print some debug info
        println!(
            "Heightmap dimensions: {} x {}",
            heightmap_width, heightmap_height
        );
        println!(
            "Sample heights min/max: {:?}/{:?}",
            data.rapier_heights
                .iter()
                .fold(f32::INFINITY, |a, &b| a.min(b)),
            data.rapier_heights
                .iter()
                .fold(f32::NEG_INFINITY, |a, &b| a.max(b))
        );

        let square_size = 1024.0 * 100.0;
        // let square_height = 1858.0;

        // Create terrain size that matches your actual terrain dimensions
        let terrain_size = Vector::new(
            square_size, // Total width in world units
            // 250.0,  // Total height in world units
            1.0,         // already specified when loading
            square_size, // Total depth in world units
        );

        let isometry = Isometry3::translation(position[0], position[1], position[2]);

        // let isometry = Isometry3::translation(-500.0, -500.0, -500.0);

        println!(
            "vertices length: {:?} heights length: {:?}",
            buffers.vertices.len(),
            data.rapier_heights.clone().len()
        );

        let terrain_collider =
            ColliderBuilder::heightfield(data.rapier_heights.clone(), terrain_size)
                .friction(0.9)
                .restitution(0.1)
                // .position(isometry)
                .user_data(
                    Uuid::from_str(landscapeComponentId)
                        .expect("Couldn't extract uuid")
                        .as_u128(),
                )
                .build();

        // Create the ground as a fixed rigid body

        println!("insert landscape position {:?}", position);

        let ground_rigid_body = RigidBodyBuilder::fixed()
            .position(isometry)
            .user_data(
                Uuid::from_str(&landscapeComponentId)
                    .expect("Couldn't extract uuid")
                    .as_u128(),
            )
            .sleeping(false)
            .build();

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Landscape Vertex Buffer"),
            contents: bytemuck::cast_slice(&buffers.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer: wgpu::Buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Landscape Index Buffer"),
                contents: bytemuck::cast_slice(&buffers.indices),
                usage: wgpu::BufferUsages::INDEX,
            });

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
            id: landscapeComponentId.to_owned(),
            index_count: buffers.indices.len() as u32,
            vertex_buffer,
            index_buffer,
            bind_group,
            // texture_bind_group,
            transform: Transform::new(
                Vector3::new(0.0, -100.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 1.0, 1.0),
                uniform_buffer,
            ),
            texture_array: None,
            texture_array_view: None,
            texture_bind_group: None,
            rapier_heightfield: terrain_collider,
            rapier_rigidbody: ground_rigid_body,
            collider_handle: None,
            rigid_body_handle: None,
            heights: data.rapier_heights.clone(),
            buffers,
            lod_data,
            lod_distances: lod_distances.clone(),
            lod_sample_stride,
        }
    }

    pub fn update_buffers(&mut self, queue: &wgpu::Queue) {
        println!("update landscape buffers!");
        // let camera = get_camera();
        // self.buffers.update(
        //     &self.lod_data,
        //     [camera.position.x, camera.position.y, camera.position.z],
        //     self.lod_distances.clone(),
        //     self.lod_sample_stride,
        // );

        // // Write new data to existing buffers
        // queue.write_buffer(
        //     &self.vertex_buffer,
        //     0, // offset
        //     bytemuck::cast_slice(&self.buffers.vertices),
        // );

        // queue.write_buffer(
        //     &self.index_buffer,
        //     0, // offset
        //     bytemuck::cast_slice(&self.buffers.indices),
        // );
    }

    pub fn update_texture(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture_bind_group_layout: &wgpu::BindGroupLayout,
        texture_render_mode_buffer: &wgpu::Buffer,
        color_render_mode_buffer: &wgpu::Buffer,
        kind: LandscapeTextureKinds,
        new_texture: &Texture,
    ) {
        let layer = match kind {
            LandscapeTextureKinds::Primary => 0,
            LandscapeTextureKinds::PrimaryMask => 1,
            LandscapeTextureKinds::Rockmap => 2,
            LandscapeTextureKinds::RockmapMask => 3,
            LandscapeTextureKinds::Soil => 4,
            LandscapeTextureKinds::SoilMask => 5,
        };

        if self.texture_array.is_none() {
            self.create_texture_array(device, new_texture.size());
        }

        if let Some(texture_array) = &self.texture_array {
            queue.write_texture(
                wgpu::ImageCopyTexture {
                    texture: texture_array,
                    mip_level: 0,
                    origin: wgpu::Origin3d {
                        x: 0,
                        y: 0,
                        z: layer,
                    },
                    aspect: wgpu::TextureAspect::All,
                },
                &new_texture.data,
                wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(4 * new_texture.size().width),
                    rows_per_image: Some(new_texture.size().height),
                },
                new_texture.size(),
            );

            self.update_bind_group(
                device,
                texture_bind_group_layout,
                texture_render_mode_buffer,
                color_render_mode_buffer,
            );
        }
    }

    fn create_texture_array(&mut self, device: &wgpu::Device, size: wgpu::Extent3d) {
        let texture_array = device.create_texture(&wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: size.width,
                height: size.height,
                depth_or_array_layers: 6, // Primary, Rockmap, Soil and associated masks
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: Some("landscape_texture_array"),
            view_formats: &[],
        });

        let texture_array_view = texture_array.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });

        self.texture_array = Some(texture_array);
        self.texture_array_view = Some(texture_array_view);
    }

    fn update_bind_group(
        &mut self,
        device: &wgpu::Device,
        texture_bind_group_layout: &wgpu::BindGroupLayout,
        texture_render_mode_buffer: &wgpu::Buffer,
        color_render_mode_buffer: &wgpu::Buffer,
    ) {
        if let Some(texture_array_view) = &self.texture_array_view {
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor::default());

            self.texture_bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                layout: texture_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(texture_array_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                            buffer: texture_render_mode_buffer,
                            offset: 0,
                            size: None,
                        }),
                    },
                ],
                label: Some("landscape_texture_bind_group"),
            }));
        }
    }

    // Generate vertex buffer from heightmap data
    // pub fn generate_terrain(data: &LandscapePixelData, scale: f32) -> (Vec<Vertex>, Vec<u32>) {
    //     println!("generating terrain vertices...");
    //     let mut vertices = Vec::with_capacity(data.width * data.height);
    //     // let mut rapier_vertices = Vec::with_capacity(data.width * data.height);
    //     let mut indices = Vec::new();

    //     for y in 0..data.height {
    //         for x in 0..data.width {
    //             vertices.push(Vertex {
    //                 position: data.pixel_data[y][x].position,
    //                 normal: [0.0, 0.0, 0.0],
    //                 tex_coords: data.pixel_data[y][x].tex_coords,
    //                 color: [1.0, 1.0, 1.0],
    //             });
    //         }
    //     }

    //     println!("generating indices...");

    //     // Generate indices with additional connections
    //     for y in 0..(data.height - 1) {
    //         for x in 0..(data.width - 1) {
    //             let top_left = (y * data.width + x) as u32;
    //             let top_right = top_left + 1;
    //             let bottom_left = ((y + 1) * data.width + x) as u32;
    //             let bottom_right = bottom_left + 1;

    //             // Main triangle
    //             indices.extend_from_slice(&[top_left, bottom_left, top_right]);
    //             indices.extend_from_slice(&[top_right, bottom_left, bottom_right]);

    //             // Additional connections
    //             if x < data.width - 2 {
    //                 // Connect to the next column
    //                 indices.extend_from_slice(&[top_right, bottom_right, top_right + 1]);
    //                 indices.extend_from_slice(&[bottom_right, bottom_right + 1, top_right + 1]);
    //             }

    //             if y < data.height - 2 {
    //                 // Connect to the next row
    //                 indices.extend_from_slice(&[
    //                     bottom_left,
    //                     bottom_left + data.width as u32,
    //                     bottom_right,
    //                 ]);
    //                 indices.extend_from_slice(&[
    //                     bottom_right,
    //                     bottom_left + data.width as u32,
    //                     bottom_right + data.width as u32,
    //                 ]);
    //             }
    //         }
    //     }

    //     println!("Terrain ready!");

    //     (vertices, indices)
    // }

    // TODO: return to this
    pub fn generate_terrain_with_lod(
        data: &LandscapePixelData,
        camera_pos: [f32; 3],
        lod_levels: Vec<LodLevel>,
        lod_distances: Vec<f32>, // Distance thresholds for each LOD level
        scale: f32,
    ) -> (Vec<Vertex>, Vec<u32>) {
        println!("generating terrain vertices with LOD...");
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // Helper function to determine LOD level based on distance from camera
        fn get_lod_level(pos: [f32; 3], camera_pos: [f32; 3], lod_distances: &Vec<f32>) -> usize {
            let distance = ((pos[0] - camera_pos[0]).powi(2)
                + (pos[1] - camera_pos[1]).powi(2)
                + (pos[2] - camera_pos[2]).powi(2))
            .sqrt();

            for (i, &threshold) in lod_distances.iter().enumerate() {
                if distance < threshold {
                    return i;
                }
            }
            lod_distances.len() - 1
        }

        // Create a grid of LOD levels
        let mut grid_lod_levels = vec![vec![0; data.width]; data.height];
        for y in 0..data.height {
            for x in 0..data.width {
                let pos = data.pixel_data[y][x].position;
                grid_lod_levels[y][x] = get_lod_level(pos, camera_pos, &lod_distances);
            }
        }

        // Smooth LOD transitions to prevent cracks
        fn smooth_lod_transitions(grid: &mut Vec<Vec<usize>>) {
            let height = grid.len();
            let width = grid[0].len();
            let mut changes = true;
            while changes {
                changes = false;
                for y in 1..height - 1 {
                    for x in 1..width - 1 {
                        let current_lod = grid[y][x];
                        // Check neighboring cells
                        for dy in -1..=1 {
                            for dx in -1..=1 {
                                let ny = (y as i32 + dy) as usize;
                                let nx = (x as i32 + dx) as usize;
                                let neighbor_lod = grid[ny][nx];
                                // If neighbor has much higher detail, increase our detail
                                if current_lod > neighbor_lod + 1 {
                                    grid[y][x] = neighbor_lod + 1;
                                    changes = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        smooth_lod_transitions(&mut grid_lod_levels);

        // Generate vertices based on LOD
        let mut vertex_map = HashMap::new(); // Maps (x,y) to vertex index
        for y in 0..data.height {
            for x in 0..data.width {
                let lod_level = grid_lod_levels[y][x];
                let stride = lod_levels[lod_level].vertex_stride;

                // Only create vertex if it's needed for this LOD level
                if x % stride == 0 && y % stride == 0 {
                    let vertex_idx = vertices.len() as u32;
                    vertex_map.insert((x, y), vertex_idx);

                    vertices.push(Vertex {
                        position: data.pixel_data[y][x].position,
                        normal: [0.0, 0.0, 0.0],
                        tex_coords: data.pixel_data[y][x].tex_coords,
                        color: [1.0, 1.0, 1.0],
                    });
                }
            }
        }

        // Generate indices with LOD-aware triangulation
        for y in 0..data.height - 1 {
            for x in 0..data.width - 1 {
                let current_lod = grid_lod_levels[y][x];
                let stride = lod_levels[current_lod].vertex_stride;

                if x % stride == 0 && y % stride == 0 {
                    // Get vertices for this quad
                    if let (
                        Some(&top_left),
                        Some(&bottom_left),
                        Some(&top_right),
                        Some(&bottom_right),
                    ) = (
                        vertex_map.get(&(x, y)),
                        vertex_map.get(&(x, y + stride)),
                        vertex_map.get(&(x + stride, y)),
                        vertex_map.get(&(x + stride, y + stride)),
                    ) {
                        // Add triangles
                        indices.extend_from_slice(&[top_left, bottom_left, top_right]);
                        indices.extend_from_slice(&[top_right, bottom_left, bottom_right]);
                    }
                }
            }
        }

        println!("LOD Terrain ready!");
        (vertices, indices)
    }
}

pub struct LodLevel {
    vertex_stride: usize, // How many vertices to skip (1 for full detail, 2 for half, 4 for quarter, etc.)
}

pub struct LodData {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    stride: usize, // 1 for full detail, 2 for half, 4 for quarter, etc.
}

pub struct TerrainBuffers {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

impl TerrainBuffers {
    pub fn new(capacity: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(capacity),
            indices: Vec::with_capacity(capacity * 6),
        }
    }

    // pub fn update(
    //     &mut self,
    //     lod_data: &[LodData],
    //     camera_pos: [f32; 3],
    //     lod_distances: Vec<f32>,
    //     sample_stride: usize,
    // ) {
    //     self.vertices.clear();
    //     self.indices.clear();
    //     let mut current_vertex_offset = 0;

    //     // For each LOD level
    //     for (lod_idx, lod) in lod_data.iter().enumerate() {
    //         let min_distance = if lod_idx == 0 {
    //             0.0
    //         } else {
    //             lod_distances[lod_idx - 1]
    //         };
    //         let max_distance = lod_distances[lod_idx];

    //         let mut start_idx = 0;
    //         let mut end_idx = 0;

    //         // Find both start and end indices based on distance thresholds

    //         // Is this expensive on a million vertices?
    //         // I want to use it to go in each direction out from center, baed on min_distance and max_distance, to collect vertices
    //         let center_camera_vertex_index = lod.vertices.iter().position(|v| {
    //             v.position[0] < camera_pos[0] + 100.0 && v.position[0] > camera_pos[0] - 100.0
    //         });

    //         for i in (0..lod.vertices.len()).step_by(sample_stride) {
    //             let vertex = &lod.vertices[i];
    //             let distance = ((vertex.position[0] - camera_pos[0]).powi(2)
    //                 + (vertex.position[2] - camera_pos[2]).powi(2))
    //             .sqrt();

    //             // println!("check {:?}", vertex.position);

    //             if distance >= min_distance && distance < max_distance {
    //                 if start_idx == 0 {
    //                     println!(
    //                         "mark start {:?} {:?} {:?} {:?} {:?}",
    //                         min_distance, max_distance, distance, vertex.position, camera_pos
    //                     );
    //                     start_idx = i;
    //                 }
    //                 end_idx = i + sample_stride;
    //             }
    //         }

    //         if end_idx > lod.vertices.len() {
    //             end_idx = lod.vertices.len();
    //         }

    //         println!("verify buffers {:?} {:?}", start_idx, end_idx);

    //         // If we found vertices within this LOD's range
    //         if start_idx < end_idx && end_idx <= lod.vertices.len() {
    //             // Add only the vertices within the distance range
    //             self.vertices
    //                 .extend_from_slice(&lod.vertices[start_idx..end_idx]);

    //             // println!(
    //             //     "verify slice {:?} {:?} {:?} {:?} {:?} {:?}",
    //             //     self.vertices[2].position,
    //             //     self.vertices[101].position,
    //             //     self.vertices[201].position,
    //             //     self.vertices[301].position,
    //             //     self.vertices[401].position,
    //             //     self.vertices[self.vertices.len() - 1].position
    //             // );

    //             // Calculate corresponding index range and adjust indices for vertex offset
    //             let index_start = (start_idx / lod.stride) * 6;
    //             let index_end = (end_idx / lod.stride) * 6;
    //             self.indices.extend(
    //                 lod.indices[index_start..index_end]
    //                     .iter()
    //                     .map(|&i| i + current_vertex_offset as u32),
    //             );

    //             current_vertex_offset += end_idx - start_idx;
    //         }
    //     }

    //     println!(
    //         "TerrainBuffer update - vertices: {}, indices: {}",
    //         self.vertices.len(),
    //         self.indices.len()
    //     );
    // }

    // approach 2
    // pub fn update(
    //     &mut self,
    //     lod_data: &[LodData],
    //     camera_pos: [f32; 3],
    //     lod_distances: Vec<f32>,
    //     sample_stride: usize,
    // ) {
    //     self.vertices.clear();
    //     self.indices.clear();
    //     let mut current_vertex_offset = 0;

    //     // For each LOD level
    //     for (lod_idx, lod) in lod_data.iter().enumerate() {
    //         let min_distance = if lod_idx == 0 {
    //             0.0
    //         } else {
    //             lod_distances[lod_idx - 1]
    //         };
    //         let max_distance = lod_distances[lod_idx];

    //         // Simple linear search for center point, but break when we find a close enough match
    //         let mut center_idx = 0;
    //         let threshold_distance = 5.0; // Adjust based on your terrain scale

    //         for (i, vertex) in lod.vertices.iter().enumerate() {
    //             let dist_to_camera = ((vertex.position[0] - camera_pos[0]).powi(2)
    //                 + (vertex.position[2] - camera_pos[2]).powi(2))
    //             .sqrt();

    //             // If we find a vertex close enough to the camera, use it as our center point
    //             if dist_to_camera < threshold_distance {
    //                 println!("found center {:?} {:?}", i, vertex.position);
    //                 center_idx = i;
    //                 break;
    //             }
    //         }

    //         // Expand outward from center point
    //         let mut included_vertices = Vec::new();

    //         // Helper function to check if vertex is in range
    //         let is_in_range =
    //             |vertex: &Vertex, camera_pos: [f32; 3], min_d: f32, max_d: f32| -> bool {
    //                 let distance = ((vertex.position[0] - camera_pos[0]).powi(2)
    //                     + (vertex.position[2] - camera_pos[2]).powi(2))
    //                 .sqrt();
    //                 distance >= min_d && distance < max_d
    //             };

    //         // Expand forward
    //         let mut current_idx = center_idx;
    //         while current_idx < lod.vertices.len()
    //             && is_in_range(
    //                 &lod.vertices[current_idx],
    //                 camera_pos,
    //                 min_distance,
    //                 max_distance,
    //             )
    //         {
    //             included_vertices.push(current_idx);
    //             current_idx += sample_stride;
    //         }

    //         // Expand backward
    //         let mut current_idx = center_idx.saturating_sub(sample_stride);
    //         while current_idx > 0
    //             && is_in_range(
    //                 &lod.vertices[current_idx],
    //                 camera_pos,
    //                 min_distance,
    //                 max_distance,
    //             )
    //         {
    //             included_vertices.push(current_idx);
    //             current_idx = current_idx.saturating_sub(sample_stride);
    //         }

    //         // Sort indices to maintain proper order
    //         included_vertices.sort_unstable();

    //         // If we found vertices within this LOD's range
    //         if !included_vertices.is_empty() {
    //             // Add vertices
    //             for &idx in &included_vertices {
    //                 self.vertices.push(lod.vertices[idx].clone());
    //             }

    //             // Calculate and add corresponding indices
    //             let first_vertex_idx = included_vertices[0];
    //             let last_vertex_idx = included_vertices[included_vertices.len() - 1];

    //             let index_start = (first_vertex_idx / lod.stride) * 6;
    //             let index_end = (last_vertex_idx / lod.stride) * 6;

    //             self.indices.extend(
    //                 lod.indices[index_start..=index_end]
    //                     .iter()
    //                     .map(|&i| i + current_vertex_offset as u32),
    //             );

    //             current_vertex_offset += included_vertices.len();
    //         }
    //     }

    //     println!(
    //         "TerrainBuffer update - vertices: {}, indices: {}",
    //         self.vertices.len(),
    //         self.indices.len()
    //     );
    // }

    // approach 3
    pub fn update(
        &mut self,
        lod_data: &[LodData],
        camera_pos: [f32; 3],
        lod_distances: Vec<f32>,
        sample_stride: usize,
    ) {
        self.vertices.clear();
        self.indices.clear();
        let mut current_vertex_offset = 0;

        // For each LOD level
        for (lod_idx, lod) in lod_data.iter().enumerate() {
            let min_distance = if lod_idx == 0 {
                0.0
            } else {
                lod_distances[lod_idx - 1]
            };
            let max_distance = lod_distances[lod_idx];

            // Calculate grid dimensions (assuming square grid, adjust if different)
            let grid_size = (lod.vertices.len() as f32).sqrt() as i32;

            // Find center point
            let mut center_grid_x = 0;
            let mut center_grid_y = 0;
            let threshold_distance = 5.0;

            // Find center point in grid coordinates
            'outer: for y in 0..grid_size {
                for x in 0..grid_size {
                    let idx = y * grid_size + x;
                    if idx >= lod.vertices.len() as i32 {
                        break;
                    }

                    let vertex = &lod.vertices[idx as usize];
                    let dist_to_camera = ((vertex.position[0] - camera_pos[0]).powi(2)
                        + (vertex.position[2] - camera_pos[2]).powi(2))
                    .sqrt();

                    if dist_to_camera < threshold_distance {
                        println!("center found {:?} {:?} {:?}", x, y, vertex.position);
                        center_grid_x = x;
                        center_grid_y = y;
                        break 'outer;
                    }
                }
            }

            let mut included_vertices = Vec::new();

            // Helper function to check if vertex is in range
            let is_in_range =
                |vertex: &Vertex, camera_pos: [f32; 3], min_d: f32, max_d: f32| -> bool {
                    let distance = ((vertex.position[0] - camera_pos[0]).powi(2)
                        + (vertex.position[2] - camera_pos[2]).powi(2))
                    .sqrt();
                    distance >= min_d && distance < max_d
                };

            // Expand outward in concentric squares
            let mut radius: i32 = 0;
            let max_radius: i32 = grid_size / 2; // Limit search radius

            'expansion: while radius < max_radius {
                // Check all cells in the current "ring"
                for dy in -radius..=radius {
                    for dx in -radius..=radius {
                        let dy = dy as f32;
                        let dx = dx as f32;
                        let radius = radius as f32;
                        let max_radius = max_radius as f32;

                        // Skip cells that aren't on the current ring's perimeter
                        if radius > 0.0 && dx.abs() != radius && dy.abs() != radius {
                            continue;
                        }

                        // Calculate grid coordinates
                        let grid_x = center_grid_x as f32 + dx;
                        let grid_y = center_grid_y as f32 + dy;

                        // Skip if outside grid bounds
                        if grid_x < 0.0
                            || grid_x >= grid_size as f32
                            || grid_y < 0.0
                            || grid_y >= grid_size as f32
                        {
                            continue;
                        }

                        // Convert to vertex index
                        let idx = (grid_y * grid_size as f32 + grid_x) as usize;
                        if idx >= lod.vertices.len() {
                            continue;
                        }

                        // Check if vertex is in range
                        if is_in_range(&lod.vertices[idx], camera_pos, min_distance, max_distance) {
                            included_vertices.push(idx);
                        } else {
                            // If we find vertices out of range, we might want to stop expanding in this direction
                            // This is a simple heuristic - you might want to make it more sophisticated
                            if included_vertices.len() > 0
                                && dx.abs() == radius
                                && dy.abs() == radius
                            {
                                break 'expansion;
                            }
                        }
                    }
                }
                radius += sample_stride as i32;
            }

            // Sort indices to maintain proper order
            included_vertices.sort_unstable();

            // If we found vertices within this LOD's range
            if !included_vertices.is_empty() {
                // Add vertices
                for &idx in &included_vertices {
                    self.vertices.push(lod.vertices[idx].clone());
                }

                // Calculate and add corresponding indices
                let first_vertex_idx = included_vertices[0];
                let last_vertex_idx = included_vertices[included_vertices.len() - 1];

                let index_start = (first_vertex_idx / lod.stride) * 6;
                let index_end = (last_vertex_idx / lod.stride) * 6;

                self.indices.extend(
                    lod.indices[index_start..=index_end]
                        .iter()
                        .map(|&i| i + current_vertex_offset as u32),
                );

                current_vertex_offset += included_vertices.len();
            }
        }

        println!(
            "TerrainBuffer update - vertices: {}, indices: {}",
            self.vertices.len(),
            self.indices.len()
        );
    }
}
