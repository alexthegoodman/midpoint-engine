use image::DynamicImage;
use nalgebra::{Isometry3, Matrix4, Vector3};
use rapier3d::prelude::*;
use rapier3d::prelude::{Collider, ColliderBuilder, RigidBody, RigidBodyBuilder};
use std::num::NonZeroU32;
use std::str::FromStr;
use util::BufferInitDescriptor;
use uuid::Uuid;
use wgpu::util::DeviceExt;
use wgpu::*;

use crate::core::Texture::Texture;
use crate::core::Transform::{matrix4_to_raw_array, Transform};
use crate::handlers::Vertex;
use crate::helpers::landscapes::get_landscape_pixels;
use crate::helpers::saved_data::LandscapeTextureKinds;

// Represents a node in our quadtree
#[derive(Debug)]
pub struct QuadNode {
    pub bounds: Rect,
    pub depth: u32,
    pub children: Option<Box<[QuadNode; 4]>>,
    pub mesh: Option<TerrainMesh>,
    // for conveneience?
    // pub terrain_position: [f32; 3],
    // pub landscape_component_id: String,
    pub rigid_body_handle: Option<RigidBodyHandle>,
    pub collider_handle: Option<ColliderHandle>,
    // Track if this node's LOD state has changed
    pub lod_dirty: bool,
}

#[derive(Debug, Clone)]
pub struct Rect {
    pub x: f32,
    pub z: f32,
    pub width: f32,
    pub height: f32,
}

#[derive(Debug)]
pub struct TerrainMesh {
    pub vertex_buffer: Buffer,
    pub index_buffer: Buffer,
    pub index_count: u32,
    pub collider: Collider,
    pub rigid_body: RigidBody,
}

impl QuadNode {
    pub fn new(
        bounds: Rect,
        height_data: &[f32],
        resolution: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
    ) -> Self {
        let depth = 0;
        Self {
            bounds: bounds.clone(),
            depth,
            children: None,
            mesh: Some(Self::create_mesh(
                &bounds.clone(),
                height_data,
                resolution,
                device,
                terrain_position,
                landscape_component_id,
            )),
            rigid_body_handle: None,
            collider_handle: None,
            lod_dirty: true, // Start as dirty to ensure initial physics setup
        }
    }

    fn cleanup_physics(
        &mut self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joint_set: &mut ImpulseJointSet,
        multibody_joint_set: &mut MultibodyJointSet,
    ) {
        // Remove collider first
        if let Some(handle) = self.collider_handle.take() {
            collider_set.remove(handle, island_manager, rigid_body_set, true);
        }

        // Then remove rigid body
        if let Some(handle) = self.rigid_body_handle.take() {
            rigid_body_set.remove(
                handle,
                island_manager,
                collider_set,
                impulse_joint_set,
                multibody_joint_set,
                true,
            );
        }

        // Recursively cleanup children
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                child.cleanup_physics(
                    rigid_body_set,
                    collider_set,
                    island_manager,
                    impulse_joint_set,
                    multibody_joint_set,
                );
            }
        }
    }

    fn add_physics_components(
        &mut self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) {
        if let Some(ref mesh) = self.mesh {
            // Create rigid body
            // let rigid_body = RigidBodyBuilder::fixed().build();
            let rigid_body_handle = rigid_body_set.insert(mesh.rigid_body.clone()); // TODO: bad clone?
            self.rigid_body_handle = Some(rigid_body_handle);

            // Create and attach collider if we have one
            // if let Some(ref collider) = mesh.collider {
            let collider_handle = collider_set.insert_with_parent(
                mesh.collider.clone(), // bad clone?
                rigid_body_handle,
                rigid_body_set,
            );
            self.collider_handle = Some(collider_handle);
            // }
        }

        // Recursively add physics components to children
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                child.add_physics_components(rigid_body_set, collider_set);
            }
        }
    }

    pub fn split(
        &mut self,
        height_data: &[f32],
        max_depth: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
    ) {
        if self.depth >= max_depth {
            return;
        }

        let half_width = self.bounds.width / 2.0;
        let half_height = self.bounds.height / 2.0;

        // Create four child nodes
        let children = Box::new([
            // Top-left
            QuadNode {
                bounds: Rect {
                    x: self.bounds.x,
                    z: self.bounds.z,
                    width: half_width,
                    height: half_height,
                },
                depth: self.depth + 1,
                children: None,
                mesh: Some(Self::create_mesh(
                    &self.bounds,
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                )),
                // terrain_position: self.terrain_position,
                // landscape_component_id: self.landscape_component_id,
                rigid_body_handle: None,
                collider_handle: None,
                lod_dirty: true,
            },
            // Top-right
            QuadNode {
                bounds: Rect {
                    x: self.bounds.x + half_width,
                    z: self.bounds.z,
                    width: half_width,
                    height: half_height,
                },
                depth: self.depth + 1,
                children: None,
                mesh: Some(Self::create_mesh(
                    &self.bounds,
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                )),
                // terrain_position: self.terrain_position,
                // landscape_component_id: self.landscape_component_id,
                rigid_body_handle: None,
                collider_handle: None,
                lod_dirty: true,
            },
            // Bottom-left
            QuadNode {
                bounds: Rect {
                    x: self.bounds.x,
                    z: self.bounds.z + half_height,
                    width: half_width,
                    height: half_height,
                },
                depth: self.depth + 1,
                children: None,
                mesh: Some(Self::create_mesh(
                    &self.bounds,
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                )),
                // terrain_position: self.terrain_position,
                // landscape_component_id: self.landscape_component_id,
                rigid_body_handle: None,
                collider_handle: None,
                lod_dirty: true,
            },
            // Bottom-right
            QuadNode {
                bounds: Rect {
                    x: self.bounds.x + half_width,
                    z: self.bounds.z + half_height,
                    width: half_width,
                    height: half_height,
                },
                depth: self.depth + 1,
                children: None,
                mesh: Some(Self::create_mesh(
                    &self.bounds,
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                )),
                // terrain_position: self.terrain_position,
                // landscape_component_id: self.landscape_component_id,
                rigid_body_handle: None,
                collider_handle: None,
                lod_dirty: true,
            },
        ]);

        self.children = Some(children);
    }

    pub fn should_split(&self, camera_pos: [f32; 3], lod_distances: &[f32]) -> bool {
        // Calculate distance from camera to node center
        let node_center = [
            self.bounds.x + self.bounds.width / 2.0,
            0.0, // Use average height here if needed
            self.bounds.z + self.bounds.height / 2.0,
        ];

        let distance = distance_squared(camera_pos, node_center);

        // Check if we're within the LOD distance threshold for this level
        distance < lod_distances[self.depth as usize].powi(2)
    }

    fn update_lod(
        &mut self,
        camera_pos: [f32; 3],
        height_data: &[f32],
        lod_distances: &[f32],
        max_depth: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
    ) -> bool {
        // Returns true if LOD state changed
        if self.depth >= max_depth {
            return false;
        }

        let should_split = self.should_split(camera_pos, lod_distances);
        let had_children = self.children.is_some();
        let mut state_changed = false;

        if should_split {
            if self.children.is_none() {
                self.split(
                    height_data,
                    max_depth,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                );
                state_changed = true;
            }

            // Recursively update children
            if let Some(ref mut children) = self.children {
                for child in children.iter_mut() {
                    if child.update_lod(
                        camera_pos,
                        height_data,
                        lod_distances,
                        max_depth,
                        device,
                        terrain_position,
                        landscape_component_id.clone(),
                    ) {
                        state_changed = true;
                    }
                }
            }
        } else if had_children {
            self.children = None;
            state_changed = true;
        }

        if state_changed {
            self.lod_dirty = true;
        }

        state_changed
    }

    fn update_physics_if_needed(
        &mut self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joint_set: &mut ImpulseJointSet,
        multibody_joint_set: &mut MultibodyJointSet,
    ) -> bool {
        // Returns true if any physics were updated
        if !self.lod_dirty {
            // Recursively check children
            if let Some(ref mut children) = self.children {
                return children.iter_mut().any(|child| {
                    child.update_physics_if_needed(
                        rigid_body_set,
                        collider_set,
                        island_manager,
                        impulse_joint_set,
                        multibody_joint_set,
                    )
                });
            }
            return false;
        }

        // Clean up existing physics components for this node
        self.cleanup_physics(
            rigid_body_set,
            collider_set,
            island_manager,
            impulse_joint_set,
            multibody_joint_set,
        );

        // Add new physics components if needed
        if self.children.is_none() {
            // Only leaf nodes get physics components
            self.add_physics_components(rigid_body_set, collider_set);
        }

        // Recursively update children's physics
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                child.update_physics_if_needed(
                    rigid_body_set,
                    collider_set,
                    island_manager,
                    impulse_joint_set,
                    multibody_joint_set,
                );
            }
        }

        self.lod_dirty = false;
        true
    }

    pub fn render<'a>(
        &'a self,
        render_pass: &mut RenderPass<'a>,
        camera_bind_group: &'a BindGroup,
        landscape_bind_group: &'a BindGroup,
        texture_bind_group: &'a BindGroup,
    ) {
        // If this node has children, render them instead
        if let Some(ref children) = self.children {
            for child in children.iter() {
                child.render(
                    render_pass,
                    camera_bind_group,
                    landscape_bind_group,
                    texture_bind_group,
                );
            }
            return;
        }

        // If this is a leaf node with a mesh, render it
        if let Some(ref mesh) = self.mesh {
            render_pass.set_bind_group(0, camera_bind_group, &[]);
            render_pass.set_bind_group(1, landscape_bind_group, &[]);
            render_pass.set_bind_group(2, texture_bind_group, &[]);

            render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
            render_pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);

            render_pass.draw_indexed(0..mesh.index_count, 0, 0..1);
        }
    }
}

impl QuadNode {
    fn create_mesh(
        bounds: &Rect,
        height_data: &[f32],
        resolution: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
    ) -> TerrainMesh {
        let mut vertices = Vec::with_capacity((resolution * resolution) as usize);
        let mut indices = Vec::with_capacity(((resolution - 1) * (resolution - 1) * 6) as usize);

        // Steps between vertices
        let step_x = bounds.width / (resolution - 1) as f32;
        let step_z = bounds.height / (resolution - 1) as f32;

        // Create a matrix to store heights for Rapier
        let mut height_matrix = nalgebra::DMatrix::zeros(resolution as usize, resolution as usize);

        // Generate vertices in a regular grid
        for z in 0..resolution {
            for x in 0..resolution {
                let pos_x = bounds.x + x as f32 * step_x;
                let pos_z = bounds.z + z as f32 * step_z;

                // Get normalized coordinates for height sampling
                let norm_x = x as f32 / (resolution - 1) as f32;
                let norm_z = z as f32 / (resolution - 1) as f32;

                // Sample height and store in matrix
                let height = sample_height_normalized(norm_x, norm_z, height_data);
                height_matrix[(z as usize, x as usize)] = height;

                // Calculate texture coordinates
                let tex_u = norm_x;
                let tex_v = norm_z;

                // Store vertex with position and texture coordinates
                vertices.push(Vertex {
                    position: [pos_x, height, pos_z],
                    normal: [0.0, 1.0, 0.0], // might want to calculate proper normals
                    tex_coords: [tex_u, tex_v],
                    color: [1.0, 0.0, 0.0],
                });
            }
        }

        // Generate indices for triangle strips
        for z in 0..resolution - 1 {
            for x in 0..resolution - 1 {
                let top_left = z * resolution + x;
                let top_right = top_left + 1;
                let bottom_left = (z + 1) * resolution + x;
                let bottom_right = bottom_left + 1;

                // First triangle (top-left, bottom-left, top-right)
                indices.push(top_left);
                indices.push(bottom_left);
                indices.push(top_right);

                // Second triangle (top-right, bottom-left, bottom-right)
                indices.push(top_right);
                indices.push(bottom_left);
                indices.push(bottom_right);
            }
        }

        // Calculate normals if needed
        if vertices.len() > 0 {
            calculate_normals(&mut vertices, &indices);
        }

        // Create vertex buffer
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create index buffer
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Terrain Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // Create Rapier heightfield
        let heights = height_matrix;
        let scaling = vector![
            bounds.width / (resolution - 1) as f32,
            1.0, // Height scale
            bounds.height / (resolution - 1) as f32
        ];
        let translation = vector![bounds.x, 0.0, bounds.z];

        let terrain_collider = ColliderBuilder::heightfield(heights.clone(), scaling)
            .friction(0.9)
            .restitution(0.1)
            // .position(isometry)
            .user_data(
                Uuid::from_str(&landscape_component_id)
                    .expect("Couldn't extract uuid")
                    .as_u128(),
            )
            .build();

        // Create the ground as a fixed rigid body

        println!(
            "insert landscape rigidbody position {:?} {:?}",
            terrain_position, translation
        );

        let isometry = Isometry3::translation(
            terrain_position[0],
            terrain_position[1],
            terrain_position[2],
        );

        let ground_rigid_body = RigidBodyBuilder::fixed()
            .position(isometry)
            // .translation(translation)
            .user_data(
                Uuid::from_str(&landscape_component_id)
                    .expect("Couldn't extract uuid")
                    .as_u128(),
            )
            .sleeping(false)
            .build();

        TerrainMesh {
            vertex_buffer,
            index_buffer,
            index_count: indices.len() as u32,
            collider: terrain_collider,
            rigid_body: ground_rigid_body,
        }
    }
}

// Helper function to calculate vertex normals
fn calculate_normals(vertices: &mut [Vertex], indices: &[u32]) {
    // Reset all normals
    for vertex in vertices.iter_mut() {
        vertex.normal = [0.0, 0.0, 0.0];
    }

    // Calculate normals for each triangle and accumulate
    for chunk in indices.chunks(3) {
        if chunk.len() == 3 {
            let v0 = vertices[chunk[0] as usize].position;
            let v1 = vertices[chunk[1] as usize].position;
            let v2 = vertices[chunk[2] as usize].position;

            // Calculate triangle normal
            let edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
            let edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

            // Cross product
            let normal = [
                edge1[1] * edge2[2] - edge1[2] * edge2[1],
                edge1[2] * edge2[0] - edge1[0] * edge2[2],
                edge1[0] * edge2[1] - edge1[1] * edge2[0],
            ];

            // Accumulate normals for each vertex of the triangle
            for &index in chunk {
                let vertex = &mut vertices[index as usize];
                vertex.normal[0] += normal[0];
                vertex.normal[1] += normal[1];
                vertex.normal[2] += normal[2];
            }
        }
    }

    // Normalize all normals
    for vertex in vertices.iter_mut() {
        let length = (vertex.normal[0] * vertex.normal[0]
            + vertex.normal[1] * vertex.normal[1]
            + vertex.normal[2] * vertex.normal[2])
            .sqrt();
        if length > 0.0 {
            vertex.normal[0] /= length;
            vertex.normal[1] /= length;
            vertex.normal[2] /= length;
        } else {
            vertex.normal = [0.0, 1.0, 0.0];
        }
    }
}

// Helper function to sample height with normalized coordinates
fn sample_height_normalized(norm_x: f32, norm_z: f32, height_data: &[f32]) -> f32 {
    let width = (height_data.len() as f32).sqrt() as usize;
    let x = (norm_x * (width - 1) as f32).round() as usize;
    let z = (norm_z * (width - 1) as f32).round() as usize;
    let index = z * width + x;
    height_data.get(index).copied().unwrap_or(0.0)
}

// Helper function to sample height from the height data
fn sample_height(x: f32, z: f32, height_data: &[f32]) -> f32 {
    // You'll need to implement proper sampling based on your height data format
    // This is a placeholder that returns 0.0
    0.0
}

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
}

impl TerrainManager {
    // Constants for LOD configuration
    const MAX_LOD_LEVELS: usize = 8;
    const BASE_LOD_DISTANCE: f32 = 1000.0; // Distance for first LOD transition
    const LOD_DISTANCE_MULTIPLIER: f32 = 0.5; // Each level shows more detail at half the distance

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
        let square_height = 150.0;
        let data = get_landscape_pixels(projectId, landscapeAssetId, landscapeFilename);

        println!("loaded heights... creating root quad...");

        // Create root node
        let root = QuadNode::new(
            Rect {
                // x: -500.0, // Adjust these values based on world scale
                // z: -500.0,
                // width: 1000.0,
                // height: 1000.0,
                width: square_size,
                height: square_size,
                x: -(square_size / 2.0),
                z: -(square_size / 2.0),
            },
            &data.raw_heights,
            32, // Base resolution
            device,
            terrain_position,
            landscapeComponentId.clone(),
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
            lod_update_interval: 0.5, // Configure as needed
        }
    }

    pub fn render<'a>(
        &'a self,
        render_pass: &mut RenderPass<'a>,
        camera_bind_group: &'a BindGroup,
        queue: &wgpu::Queue,
    ) {
        if let Some(ref texture_bind_group) = &self.texture_bind_group {
            // Update any per-terrain transform uniforms if needed
            self.transform.update_uniform_buffer(&queue);

            // Render the entire quadtree
            self.root.render(
                render_pass,
                camera_bind_group,
                &self.bind_group,
                texture_bind_group,
            );
        }
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

        // Only update LOD and physics at specified intervals
        if self.lod_update_timer >= self.lod_update_interval {
            let lod_distances = self.calculate_lod_distances();

            // Update LOD structure
            let lod_changed = self.root.update_lod(
                camera_pos,
                &self.height_data,
                &lod_distances,
                Self::MAX_LOD_LEVELS as u32,
                device,
                self.terrain_position,
                self.id.clone(),
            );

            // Only update physics if LOD changed
            if lod_changed {
                println!("update_physics_if_needed");
                self.root.update_physics_if_needed(
                    rigid_body_set,
                    collider_set,
                    island_manager,
                    impulse_joint_set,
                    multibody_joint_set,
                );
            }

            self.lod_update_timer = 0.0;
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
        let mut distances = Vec::with_capacity(Self::MAX_LOD_LEVELS);
        let mut current_distance = Self::BASE_LOD_DISTANCE;

        for _ in 0..Self::MAX_LOD_LEVELS {
            distances.push(current_distance);
            current_distance *= Self::LOD_DISTANCE_MULTIPLIER;
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

// Utility function to calculate squared distance between two 3D points
pub fn distance_squared(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    dx * dx + dy * dy + dz * dz
}

// Add this to handle hysteresis for LOD transitions
pub struct LodTransition {
    split_distance: f32,
    merge_distance: f32,
    transition_factor: f32,
}

impl LodTransition {
    pub fn new(base_distance: f32) -> Self {
        Self {
            split_distance: base_distance,
            merge_distance: base_distance * 1.2, // Add 20% hysteresis
            transition_factor: 0.0,
        }
    }

    pub fn update(&mut self, camera_distance: f32) -> bool {
        if camera_distance < self.split_distance {
            self.transition_factor = (self.split_distance - camera_distance) / self.split_distance;
            true
        } else if camera_distance > self.merge_distance {
            self.transition_factor = 0.0;
            false
        } else {
            // Keep current state to avoid flickering
            self.transition_factor > 0.5
        }
    }
}
