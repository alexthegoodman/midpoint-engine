use image::DynamicImage;
use nalgebra::{Isometry3, Matrix4, Point3, Vector3};
use rapier3d::prelude::*;
use rapier3d::prelude::{Collider, ColliderBuilder, RigidBody, RigidBodyBuilder};
use std::num::NonZeroU32;
use std::str::FromStr;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Instant;
use util::BufferInitDescriptor;
use uuid::Uuid;
use wgpu::util::DeviceExt;
use wgpu::*;

use crate::core::Texture::Texture;
use crate::core::Transform::{matrix4_to_raw_array, Transform};
use crate::handlers::{get_camera, Vertex};
use crate::helpers::landscapes::get_landscape_pixels;
use crate::helpers::saved_data::LandscapeTextureKinds;
use crate::landscapes::LandscapeLOD::{add_physics_components_mini, Rect};

use super::LandscapeLOD::{
    ColliderMessage, BASE_LOD_DISTANCE, LOD_DISTANCE_MULTIPLIER, MAX_LOD_LEVELS,
};
use super::QuadNode::QuadNode;

// Main terrain manager
pub struct TerrainManager {
    pub id: String,
    pub terrain_position: [f32; 3],
    pub root: QuadNode,
    pub height_data: Vec<f32>,
    pub transform: Transform,

    pub bind_group: wgpu::BindGroup,
    pub texture_array: Option<wgpu::Texture>,
    pub texture_array_view: Option<wgpu::TextureView>,
    pub texture_bind_group: Option<wgpu::BindGroup>,

    // Track time since last LOD update
    pub lod_update_timer: f32,
    pub lod_update_interval: f32, // e.g., 0.5 seconds

    pub collider_sender: Sender<ColliderMessage>,
    pub collider_receiver: Receiver<ColliderMessage>,
}

impl TerrainManager {
    pub fn new(
        projectId: String,
        landscapeComponentId: String,
        landscapeAssetId: String,
        landscapeFilename: String,
        device: &Device,
        bind_group_layout: &wgpu::BindGroupLayout,
        terrain_position: [f32; 3],
    ) -> Self {
        // Load height data from TIFF
        // let square_size = 1024.0 * 100.0;
        // let square_height = 1858.0 * 10.0;
        let square_size = 1024.0 * 4.0;
        let square_height = 150.0 * 4.0;
        let data = get_landscape_pixels(projectId, landscapeAssetId, landscapeFilename);

        println!("loaded heights... creating root quad...");

        let (sender, receiver) = channel();

        // Create root node
        let root = QuadNode::new(
            Rect {
                // x: -500.0, // Adjust these values based on world scale
                // z: -500.0,
                // width: 1000.0,
                // height: 1000.0,
                width: square_size,
                height: square_size,
                x: -(square_size / 2.0) + terrain_position[0],
                z: -(square_size / 2.0) + terrain_position[2],
            },
            &data.raw_heights,
            16, // Base resolution
            device,
            terrain_position,
            landscapeComponentId.clone(),
            sender.clone(),
        );

        // set uniform buffer for transforms
        let empty_buffer = Matrix4::<f32>::identity();
        let raw_matrix = matrix4_to_raw_array(&empty_buffer);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("TerrainManager Uniform Buffer"),
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
            id: landscapeComponentId.clone(),
            terrain_position,
            root,
            height_data: data.raw_heights,
            transform: Transform::new(
                Vector3::new(
                    terrain_position[0],
                    terrain_position[1],
                    terrain_position[2],
                ),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 1.0, 1.0),
                uniform_buffer,
            ),
            bind_group,
            texture_array: None,
            texture_array_view: None,
            texture_bind_group: None,
            lod_update_timer: 0.0,
            lod_update_interval: 2.0, // Configure as needed
            collider_sender: sender.clone(),
            collider_receiver: receiver,
        }
    }

    pub fn render<'a>(
        &'a self,
        render_pass: &mut RenderPass<'a>,
        camera_bind_group: &'a BindGroup,
        queue: &wgpu::Queue,
    ) {
        if let Some(ref texture_bind_group) = &self.texture_bind_group {
            let render_time = Instant::now();

            // Update any per-terrain transform uniforms if needed
            self.transform.update_uniform_buffer(&queue);

            // Render the entire quadtree
            self.root.render(
                render_pass,
                camera_bind_group,
                &self.bind_group,
                texture_bind_group,
            );

            let render_duration = render_time.elapsed();
            // println!("  render_duration: {:?}", render_duration);
        }
    }

    pub fn find_chunk_by_id(&mut self, chunk_id: &String) -> Option<&mut QuadNode> {
        fn recurse_children<'a>(
            node: &'a mut QuadNode,
            chunk_id: &str,
        ) -> Option<&'a mut QuadNode> {
            if let Some(ref mesh) = node.mesh {
                if mesh.mesh_id == chunk_id {
                    return Some(node);
                }
            }

            if let Some(ref mut children) = node.children {
                for child in children.iter_mut() {
                    if let Some(found) = recurse_children(child, &chunk_id) {
                        return Some(found);
                    }
                }
            }

            None
        }

        recurse_children(&mut self.root, &chunk_id)
    }

    pub fn update(
        &mut self,
        camera_pos: [f32; 3],
        device: &Device,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joint_set: &mut ImpulseJointSet,
        multibody_joint_set: &mut MultibodyJointSet,
        delta_time: f32,
    ) {
        self.lod_update_timer += delta_time;

        // Check for completed colliders
        while let Ok((chunk_id, collider)) = self.collider_receiver.try_recv() {
            // Find the relevant chunk and add the collider
            if let Some(chunk) = self.find_chunk_by_id(&chunk_id) {
                // Add collider to chunk's mesh
                if let Some(ref mut mesh) = chunk.mesh {
                    println!("attaching collider");
                    // mesh.collider = Some(collider);
                    // Now you can add it to physics world if needed
                    add_physics_components_mini(
                        rigid_body_set,
                        collider_set,
                        device,
                        chunk,
                        collider,
                    );
                }
            }
        }

        // Only update LOD and physics at specified intervals
        if self.lod_update_timer >= self.lod_update_interval {
            let update_time = Instant::now();

            let lod_distances = self.calculate_lod_distances();

            println!("update lods? {:?} {:?}", lod_distances, camera_pos);

            // Update LOD structure
            let lod_changed = self.root.update_lod(
                camera_pos,
                &self.height_data,
                &lod_distances,
                MAX_LOD_LEVELS as u32,
                device,
                self.terrain_position,
                self.id.clone(),
                self.collider_sender.clone(),
            );

            let update_duration = update_time.elapsed();
            println!("  update_duration: {:?}", update_duration);
            let physics_time = Instant::now();

            // Only update physics if LOD changed
            // if lod_changed {
            println!("update_physics_if_needed");
            self.root.update_physics_if_needed(
                rigid_body_set,
                collider_set,
                island_manager,
                impulse_joint_set,
                multibody_joint_set,
                device,
                self.terrain_position,
            );
            // }

            self.lod_update_timer = 0.0;

            let physics_duration = physics_time.elapsed();
            println!("  physics_duration: {:?}", physics_duration);

            println!(
                "get_active_physics_count {:?} {:?} {:?}",
                self.get_active_physics_count(),
                rigid_body_set.len(),
                collider_set.len()
            );
        }
    }

    // For debugging/profiling
    pub fn get_active_physics_count(&self) -> (usize, usize) {
        // (bodies, colliders)
        let mut body_count = 0;
        let mut collider_count = 0;

        fn count_physics(node: &QuadNode) -> (usize, usize) {
            let mut bodies = node.rigid_body_handle.is_some() as usize;
            let mut colliders = node.collider_handle.is_some() as usize;

            if let Some(ref children) = node.children {
                for child in children.iter() {
                    let (b, c) = count_physics(child);
                    bodies += b;
                    colliders += c;
                }
            }
            (bodies, colliders)
        }

        count_physics(&self.root)
    }

    pub fn calculate_lod_distances(&self) -> Vec<f32> {
        let mut distances = Vec::with_capacity(MAX_LOD_LEVELS);
        let mut current_distance = BASE_LOD_DISTANCE;

        for _ in 0..MAX_LOD_LEVELS {
            distances.push(current_distance);
            current_distance *= LOD_DISTANCE_MULTIPLIER;
        }

        distances
    }

    // may move to nodes management
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
}
