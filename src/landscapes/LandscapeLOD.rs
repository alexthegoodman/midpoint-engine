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

// Represents a node in our quadtree
#[derive(Debug)]
pub struct QuadNode {
    pub bounds: Rect,
    pub depth: u32,
    pub children: Option<Box<[QuadNode; 4]>>,
    pub mesh: Option<TerrainMesh>,
    pub debug_mesh: Option<TerrainMesh>,
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
    pub mesh_id: String,
    pub vertex_buffer: Buffer,
    pub index_buffer: Buffer,
    pub index_count: u32,
    pub collider: Option<Collider>,
    pub rigid_body: Option<RigidBody>,
    pub depth: u32,
}

const PHYSICS_DISTANCE: f32 = 1000.0;

impl QuadNode {
    pub fn new(
        bounds: Rect,
        height_data: &[f32],
        resolution: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
        collider_sender: Sender<ColliderMessage>,
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
                depth,
                collider_sender,
            )),
            debug_mesh: None,
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
        // println!("clean up physics!");
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
        device: &wgpu::Device,
    ) {
        // Only add physics for chunks at deeper levels (e.g., more detailed)
        // let min_physics_depth = 2; // Tune this value
        // if self.depth >= min_physics_depth {
        if let Some(ref mut mesh) = self.mesh {
            // Get rigid body
            // if let Some(rigid_body) = mesh.rigid_body.take() {
            if let Some(ref rigid_body) = mesh.rigid_body {
                let rigid_body_handle = rigid_body_set.insert(rigid_body.clone());
                self.rigid_body_handle = Some(rigid_body_handle);
                // Create and attach collider if we have one
                // if let Some(collider) = mesh.collider.take() {
                if let Some(ref collider) = mesh.collider {
                    // if let Some(debug_mesh) = create_debug_collision_mesh(&collider, device) {
                    //     self.debug_mesh = Some(debug_mesh);
                    // }

                    let collider_handle = collider_set.insert_with_parent(
                        collider.clone(),
                        rigid_body_handle,
                        rigid_body_set,
                    );
                    self.collider_handle = Some(collider_handle);
                }
            }
        }
        // }

        // Recursively add physics components to children
        if let Some(ref mut children) = self.children {
            for child in children.iter_mut() {
                child.add_physics_components(rigid_body_set, collider_set, device);
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
        collider_sender: Sender<ColliderMessage>,
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
                    &Rect {
                        x: self.bounds.x,
                        z: self.bounds.z,
                        width: half_width,
                        height: half_height,
                    },
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    self.depth + 1,
                    collider_sender.clone(),
                )),
                debug_mesh: None,
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
                    &Rect {
                        x: self.bounds.x + half_width,
                        z: self.bounds.z,
                        width: half_width,
                        height: half_height,
                    },
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    self.depth + 1,
                    collider_sender.clone(),
                )),
                debug_mesh: None,
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
                    &Rect {
                        x: self.bounds.x,
                        z: self.bounds.z + half_height,
                        width: half_width,
                        height: half_height,
                    },
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    self.depth + 1,
                    collider_sender.clone(),
                )),
                debug_mesh: None,
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
                    &Rect {
                        x: self.bounds.x + half_width,
                        z: self.bounds.z + half_height,
                        width: half_width,
                        height: half_height,
                    },
                    height_data,
                    16,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    self.depth + 1,
                    collider_sender.clone(),
                )),
                debug_mesh: None,
                // terrain_position: self.terrain_position,
                // landscape_component_id: self.landscape_component_id,
                rigid_body_handle: None,
                collider_handle: None,
                lod_dirty: true,
            },
        ]);

        self.children = Some(children);
    }

    pub fn should_split(
        &self,
        camera_pos: [f32; 3],
        lod_distances: &[f32],
        transform_position: [f32; 3],
    ) -> bool {
        // let closest_dist = get_camera_distance_from_bounds(self.bounds.clone(), transform_position);
        let closest_dist =
            get_camera_distance_from_bound_center_rel(self.bounds.clone(), transform_position);

        // Calculate node size (diagonal)
        let node_size = (self.bounds.width * self.bounds.width
            + self.bounds.height * self.bounds.height)
            .sqrt();

        // Calculate view-dependent error metric
        // Split if we're close to any part of the node relative to its size
        let error_threshold = node_size * 0.5; // Adjust this factor to control split aggressiveness
        let should_split =
            closest_dist < (lod_distances[self.depth as usize] * error_threshold).powi(2);

        // println!(
        //     "Depth: {}, Node size: {:.2}, Closest dist: {:.2}, Threshold: {:.2}, Split: {}",
        //     self.depth,
        //     node_size,
        //     closest_dist.sqrt(),
        //     lod_distances[self.depth as usize] * error_threshold,
        //     should_split
        // );

        should_split
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
        collider_sender: Sender<ColliderMessage>,
    ) -> bool {
        if self.depth >= max_depth {
            return false;
        }

        let should_split = self.should_split(camera_pos, lod_distances, terrain_position);
        let had_children = self.children.is_some();
        let mut state_changed = false;

        if should_split {
            // Create children if we don't have them
            if !had_children {
                self.split(
                    height_data,
                    max_depth,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    collider_sender.clone(),
                );
                state_changed = true;
            }

            // Find the closest child and only split that one
            if let Some(ref mut children) = self.children {
                // Calculate distances to each child's center
                let mut closest_idx = 0;
                let mut min_distance = f32::MAX;

                // TODO: split all children if should_split is true?

                for (idx, child) in children.iter().enumerate() {
                    let child_center = [
                        terrain_position[0] + child.bounds.x + child.bounds.width / 2.0,
                        terrain_position[1],
                        terrain_position[2] + child.bounds.z + child.bounds.height / 2.0,
                    ];
                    let dist = distance_squared(camera_pos, child_center);
                    if dist < min_distance {
                        min_distance = dist;
                        closest_idx = idx;
                    }
                }

                // Only update the closest child
                if children[closest_idx].update_lod(
                    camera_pos,
                    height_data,
                    lod_distances,
                    max_depth,
                    device,
                    terrain_position,
                    landscape_component_id.clone(),
                    collider_sender.clone(),
                ) {
                    state_changed = true;
                }

                // Merge other children if they have subdivisions
                for (idx, child) in children.iter_mut().enumerate() {
                    if idx != closest_idx && child.children.is_some() {
                        child.children = None;
                        state_changed = true;
                    }
                }
            }
        } else if had_children {
            self.children = None;
            state_changed = true;
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
        device: &wgpu::Device,
        transform_position: [f32; 3],
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
                        device,
                        transform_position,
                    )
                });
            }
            return false;
        }

        let closest_dist =
            get_camera_distance_from_bound_center_rel(self.bounds.clone(), transform_position)
                .sqrt();

        // TODO: set to reasonable amount
        if (closest_dist < PHYSICS_DISTANCE) {
            println!("closest_dist on physics {:?}", closest_dist);
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
                // this merely adds them to sets, does not create them
                self.add_physics_components(rigid_body_set, collider_set, device);
            }
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
                    device,
                    transform_position,
                );
            }
        }

        self.lod_dirty = false;
        true
    }

    fn render<'a>(
        &'a self,
        render_pass: &mut RenderPass<'a>,
        camera_bind_group: &'a BindGroup,
        landscape_bind_group: &'a BindGroup,
        texture_bind_group: &'a BindGroup,
    ) {
        // Skip depth 0 (root node)
        // if self.depth == 0 || self.depth == 1 || self.depth == 2 || self.depth ==  {
        //     // Only render children if they exist
        //     if let Some(ref children) = self.children {
        //         for child in children.iter() {
        //             child.render(
        //                 render_pass,
        //                 camera_bind_group,
        //                 landscape_bind_group,
        //                 texture_bind_group,
        //             );
        //         }
        //     }
        //     return;
        // }

        // Rest of the render logic for non-root nodes
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

        if let Some(ref mesh) = self.mesh {
            render_pass.set_bind_group(0, camera_bind_group, &[]);
            render_pass.set_bind_group(1, landscape_bind_group, &[]);
            render_pass.set_bind_group(2, texture_bind_group, &[]);

            render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
            render_pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);

            render_pass.draw_indexed(0..mesh.index_count, 0, 0..1);
        }

        if let Some(ref debug_mesh) = self.debug_mesh {
            render_pass.set_bind_group(0, camera_bind_group, &[]);
            render_pass.set_bind_group(1, landscape_bind_group, &[]);
            render_pass.set_bind_group(2, texture_bind_group, &[]);

            render_pass.set_vertex_buffer(0, debug_mesh.vertex_buffer.slice(..));
            render_pass
                .set_index_buffer(debug_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);

            render_pass.draw_indexed(0..debug_mesh.index_count, 0, 0..1);
        }
    }
}

impl QuadNode {
    // Maybe something like this for resolution calculation
    fn get_resolution_for_depth(depth: u32) -> u32 {
        // Example scaling:
        // depth 0 (root) -> 16
        // depth 1 -> 32
        // depth 2 -> 64
        // depth 3 -> 128
        // depth 4 -> 256
        match depth {
            0 => 16,
            1 => 32,
            2 => 64,
            3 => 128,
            4 => 256,
            5 => 512,
            6 => 1024,
            7 => 2048,
            8 => 4096,
            _ => 16,
        }
    }

    fn create_mesh(
        bounds: &Rect,
        height_data: &[f32],
        resolution: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
        depth: u32,
        collider_sender: Sender<ColliderMessage>,
    ) -> TerrainMesh {
        let mesh_id = Uuid::new_v4().to_string();
        let calc_res = Self::get_resolution_for_depth(depth);
        let mut vertices = Vec::with_capacity((resolution * resolution) as usize);
        let mut indices = Vec::with_capacity(((resolution - 1) * (resolution - 1) * 6) as usize);

        // Steps between vertices
        // let step_x = bounds.width / (resolution - 1) as f32;
        // let step_z = bounds.height / (resolution - 1) as f32;

        // Create a matrix to store heights for Rapier
        let mut height_matrix = nalgebra::DMatrix::zeros(calc_res as usize, calc_res as usize);
        let width_calc = (height_data.len() as f32).sqrt() as f32;

        println!("width calc {:?} {:?}", width_calc, calc_res);

        let camera = get_camera();
        println!("create mesh, cam pos: {:?}", camera.position);

        let mut rapier_vertices = Vec::new();

        // Generate vertices
        for z in 0..calc_res {
            for x in 0..calc_res {
                // Calculate world position
                let world_x = bounds.x + (x as f32 * ((bounds.width) / calc_res as f32));
                let world_z = bounds.z + (z as f32 * ((bounds.height) / calc_res as f32));

                // Get height at this world position
                let height = sample_height_world(world_x, world_z, height_data);
                height_matrix[(z as usize, x as usize)] = height + (terrain_position[1] / 2.0); // add if negative position?

                // Calculate texture coordinates based on world position
                let tex_x = (world_x + width_calc / 2.0) / width_calc;
                let tex_z = (world_z + width_calc / 2.0) / width_calc;

                vertices.push(Vertex {
                    position: [world_x, height, world_z],
                    normal: [0.0, 1.0, 0.0],
                    tex_coords: [tex_x, tex_z],
                    color: [1.0, 0.0, 0.0],
                });

                rapier_vertices.push(Point3::new(world_x, height, world_z));
            }
        }

        println!("height: {:?}", vertices[1].position[1]);
        println!("vertices length {:?}", vertices.len());

        // Generate indices for triangle strips
        for z in 0..calc_res - 1 {
            for x in 0..calc_res - 1 {
                let top_left = z * calc_res + x;
                let top_right = top_left + 1;
                let bottom_left = (z + 1) * calc_res + x;
                let bottom_right = bottom_left + 1;

                // First triangle (top-left, bottom-left, top-right)
                indices.push(top_left);
                indices.push(bottom_left);
                indices.push(top_right);

                // Second triangle (top-right, bottom-left, bottom-right)
                indices.push(top_right);
                indices.push(bottom_left);
                indices.push(bottom_right);

                // // Additional connections - but only within this quad
                // if x < calc_res - 2 {
                //     // Connect to next column, but only if we're not at the quad edge
                //     indices.extend_from_slice(&[
                //         top_right as u32,
                //         bottom_right as u32,
                //         top_right + 1 as u32,
                //     ]);
                //     indices.extend_from_slice(&[
                //         bottom_right as u32,
                //         bottom_right + 1 as u32,
                //         top_right + 1 as u32,
                //     ]);
                // }

                // if z < calc_res - 2 {
                //     // Connect to next row, but only if we're not at the quad edge
                //     indices.extend_from_slice(&[
                //         bottom_left as u32,
                //         bottom_left + calc_res as u32,
                //         bottom_right as u32,
                //     ]);
                //     indices.extend_from_slice(&[
                //         bottom_right as u32,
                //         bottom_left + calc_res as u32,
                //         bottom_right + calc_res as u32,
                //     ]);
                // }
            }
        }

        // Calculate normals if needed
        if vertices.len() > 0 {
            // calculate_normals(&mut vertices, &indices);
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
            bounds.width as f32,
            1.0, // Height scale
            bounds.height as f32
        ];

        // let translation = vector![bounds.x, 0.0, bounds.z];
        let translation = vector![0.0, -220.0, 0.0];

        let isometry = Isometry3::translation(
            terrain_position[0],
            terrain_position[1],
            terrain_position[2],
        );

        // let terrain_collider = ColliderBuilder::heightfield(heights.clone(), scaling)
        //     .friction(0.9)
        //     .restitution(0.1)
        //     // .position(isometry)
        //     // .translation(translation)
        //     .user_data(
        //         Uuid::from_str(&landscape_component_id)
        //             .expect("Couldn't extract uuid")
        //             .as_u128(),
        //     )
        //     .build();

        let closest_dist =
            get_camera_distance_from_bound_center_rel(bounds.clone(), terrain_position).sqrt();

        println!("closest_dist {:?}", closest_dist);

        // TODO: set to reasonable amount
        if (closest_dist < PHYSICS_DISTANCE) {
            // let collider_time = Instant::now();

            // let terrain_collider = ColliderBuilder::trimesh(
            //     rapier_vertices,
            //     indices
            //         .chunks(3)
            //         .map(|chunk| [chunk[0], chunk[1], chunk[2]])
            //         .collect::<Vec<[u32; 3]>>(),
            // )
            // .user_data(
            //     Uuid::from_str(&landscape_component_id)
            //         .expect("Couldn't extract uuid")
            //         .as_u128(),
            // )
            // .build();

            // let collider_duration = collider_time.elapsed();
            // println!("  collider_duration: {:?}", collider_duration);

            let sender = collider_sender.clone();
            let vertices = rapier_vertices.clone();
            let indices_clone = indices.clone();
            let chunk_id = mesh_id.clone();

            // Spawn the heavy computation in a separate thread
            std::thread::spawn(move || {
                let collider = ColliderBuilder::trimesh(
                    vertices,
                    indices_clone
                        .chunks(3)
                        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
                        .collect::<Vec<[u32; 3]>>(),
                )
                .user_data(Uuid::from_str(&chunk_id).unwrap().as_u128())
                .build();

                // Send the completed collider back
                sender.send((chunk_id, collider)).unwrap();
            });

            // Create the ground as a fixed rigid body

            println!(
                "insert landscape rigidbody position {:?} {:?} {:?}",
                depth, bounds, terrain_position
            );

            let rigid_time = Instant::now();

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

            let rigid_duration = rigid_time.elapsed();
            println!("  rigid_duration: {:?}", rigid_duration);

            TerrainMesh {
                mesh_id,
                vertex_buffer,
                index_buffer,
                index_count: indices.clone().len() as u32,
                collider: None,
                rigid_body: Some(ground_rigid_body),
                depth,
            }
        } else {
            println!("not addressing most physics");

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
                mesh_id,
                vertex_buffer,
                index_buffer,
                index_count: indices.clone().len() as u32,
                collider: None,
                rigid_body: Some(ground_rigid_body),
                depth,
            }
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

// // Helper function to sample height with normalized coordinates
fn sample_height_normalized(norm_x: f32, norm_z: f32, height_data: &[f32]) -> f32 {
    let width = (height_data.len() as f32).sqrt() as usize;
    let x = (norm_x * (width - 1) as f32).round() as usize;
    let z = (norm_z * (width - 1) as f32).round() as usize;
    let index = z * width + x;
    height_data.get(index).copied().unwrap_or(0.0)
}

fn sample_height_world(world_x: f32, world_z: f32, height_data: &[f32]) -> f32 {
    let width = (height_data.len() as f32).sqrt() as f32;

    // println!("normal width {:?}", width);

    // Convert from world space to 0-1 range using actual terrain width
    let norm_x = (world_x + width / 2.0) / width;
    let norm_z = (world_z + width / 2.0) / width;

    // Use these normalized coordinates directly to get the index
    let index = ((norm_z * width as f32) as f32 * width) + (norm_x * width as f32) as f32;

    // println!(
    //     "World pos ({:.1}, {:.1}) -> norm ({:.3}, {:.3}) -> index {}",
    //     world_x, world_z, norm_x, norm_z, index
    // );

    height_data.get(index as usize).copied().unwrap_or(0.0)
}

// Helper function to sample height from the height data
fn sample_height(x: f32, z: f32, height_data: &[f32]) -> f32 {
    // You'll need to implement proper sampling based on your height data format
    // This is a placeholder that returns 0.0
    0.0
}

// Create type for the message we'll send through channel
type ColliderMessage = (String, Collider); // (chunk_id, collider)

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
    // Constants for LOD configuration
    const MAX_LOD_LEVELS: usize = 4;
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
            32, // Base resolution
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
                Self::MAX_LOD_LEVELS as u32,
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

pub fn get_camera_distance_from_bounds(bounds: Rect, transform_position: [f32; 3]) -> f32 {
    let camera = get_camera();

    // Get node corners in world space
    let corners = [
        // Near corners
        [
            transform_position[0] + bounds.x,
            transform_position[1],
            transform_position[2] + bounds.z,
        ],
        [
            transform_position[0] + bounds.x + bounds.width,
            transform_position[1],
            transform_position[2] + bounds.z,
        ],
        // Far corners
        [
            transform_position[0] + bounds.x,
            transform_position[1],
            transform_position[2] + bounds.z + bounds.height,
        ],
        [
            transform_position[0] + bounds.x + bounds.width,
            transform_position[1],
            transform_position[2] + bounds.z + bounds.height,
        ],
    ];

    // Find closest point to camera
    let closest_dist = corners
        .iter()
        .map(|corner| {
            distance_squared(
                [camera.position.x, camera.position.y, camera.position.z],
                *corner,
            )
        })
        .reduce(f32::min)
        .unwrap_or(f32::MAX);

    closest_dist
}

pub fn get_camera_distance_from_bound_center(bounds: Rect, transform_position: [f32; 3]) -> f32 {
    let camera = get_camera();

    // Get node corners in world space
    let center = [
        transform_position[0] + bounds.x + (bounds.width / 2.0),
        transform_position[1],
        transform_position[2] + bounds.z + (bounds.height / 2.0),
    ];

    // Find closest point to camera
    let dist = distance_squared(
        [camera.position.x, camera.position.y, camera.position.z],
        center,
    );

    dist
}

pub fn get_camera_distance_from_bound_center_rel(
    bounds: Rect,
    transform_position: [f32; 3],
) -> f32 {
    let camera = get_camera();

    // Get node corners in world space
    let center = [
        transform_position[0] + bounds.x + (bounds.width / 2.0),
        camera.position[1],
        transform_position[2] + bounds.z + (bounds.height / 2.0),
    ];

    // Find closest point to camera
    let dist = distance_squared(
        [camera.position.x, camera.position.y, camera.position.z],
        center,
    );

    dist
}

pub fn get_camera_distance_from_bounds_rel(bounds: Rect, transform_position: [f32; 3]) -> f32 {
    let camera = get_camera();

    // Get node corners in world space
    let corners = [
        // Near corners
        [
            transform_position[0] + bounds.x,
            camera.position[1],
            transform_position[2] + bounds.z,
        ],
        [
            transform_position[0] + bounds.x + bounds.width,
            camera.position[1],
            transform_position[2] + bounds.z,
        ],
        // Far corners
        [
            transform_position[0] + bounds.x,
            camera.position[1],
            transform_position[2] + bounds.z + bounds.height,
        ],
        [
            transform_position[0] + bounds.x + bounds.width,
            camera.position[1],
            transform_position[2] + bounds.z + bounds.height,
        ],
    ];

    // Find closest point to camera
    let closest_dist = corners
        .iter()
        .map(|corner| {
            distance_squared(
                [camera.position.x, camera.position.y, camera.position.z],
                *corner,
            )
        })
        .reduce(f32::min)
        .unwrap_or(f32::MAX);

    closest_dist
}

pub fn add_physics_components_mini(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    device: &wgpu::Device,
    // mesh: &mut TerrainMesh,
    quad: &mut QuadNode,
    collider: Collider,
) {
    // Only add physics for chunks at deeper levels (e.g., more detailed)
    // let min_physics_depth = 2; // Tune this value
    // if self.depth >= min_physics_depth {
    println!("landscape collider add_physics_components");
    if let Some(ref mut mesh) = quad.mesh {
        // Get rigid body
        println!("landscape collider self.mesh");
        // if let Some(rigid_body) = mesh.rigid_body.take() {
        if let Some(ref rigid_body) = mesh.rigid_body {
            let rigid_body_handle = rigid_body_set.insert(rigid_body.clone());
            quad.rigid_body_handle = Some(rigid_body_handle);
            println!("landscape collider rigid_body");
            // Create and attach collider if we have one
            // if let Some(collider) = mesh.collider.take() {
            //     // if let Some(debug_mesh) = create_debug_collision_mesh(&collider, device) {
            //     //     self.debug_mesh = Some(debug_mesh);
            //     // }

            //     println!("landscape collider insert_with_parent");

            //     let collider_handle =
            //         collider_set.insert_with_parent(collider, rigid_body_handle, rigid_body_set);
            //     quad.collider_handle = Some(collider_handle);
            // }

            // match &mesh.collider {
            //     Some(_) => println!("Collider exists before take"),
            //     None => println!("No collider before take"),
            // }

            // // Use as_ref() or clone() instead of take()
            // if let Some(ref collider) = mesh.collider {
            println!("landscape collider insert_with_parent");

            let collider_handle =
                collider_set.insert_with_parent(collider, rigid_body_handle, rigid_body_set);
            quad.collider_handle = Some(collider_handle);
            // }
        }
    }
    // }
}

// Utility function to calculate squared distance between two 3D points
pub fn distance_squared(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    dx * dx + dy * dy + dz * dz
}

fn create_debug_collision_mesh(collider: &Collider, device: &Device) -> Option<TerrainMesh> {
    if let Some(shape) = collider.shape().as_heightfield() {
        // Debug print the heightfield properties
        println!("Heightfield properties:");
        println!("  Scale: {:?}", shape.scale());
        println!("  Dimensions: {} x {}", shape.nrows(), shape.ncols());

        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut vertex_index = 0;

        // Print some height values from the heightfield directly
        println!("Raw height samples:");
        for i in 0..3 {
            println!("  Height at {}: {}", i, shape.heights()[i]);
        }

        // Get triangles and build vertex/index buffers
        let triangles = shape.triangles();

        // Track min/max Y values to verify variation
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;

        for triangle in triangles {
            // Check Y variation in triangles
            min_y = min_y.min(triangle.a.y).min(triangle.b.y).min(triangle.c.y);
            max_y = max_y.max(triangle.a.y).max(triangle.b.y).max(triangle.c.y);

            if vertex_index < 3 {
                println!("Triangle {}:", vertex_index / 3);
                println!("  A: {:?}", triangle.a);
                println!("  B: {:?}", triangle.b);
                println!("  C: {:?}", triangle.c);
            }

            // Add vertices
            vertices.push(Vertex {
                position: [triangle.a.x, triangle.a.y, triangle.a.z],
                normal: [0.0, 1.0, 0.0], // We could calculate proper normals if needed
                tex_coords: [0.0, 0.0],  // Not needed for debug visualization
                color: [1.0, 0.0, 0.0],  // Red for debug visualization
            });
            vertices.push(Vertex {
                position: [triangle.b.x, triangle.b.y, triangle.b.z],
                normal: [0.0, 1.0, 0.0],
                tex_coords: [0.0, 0.0],
                color: [1.0, 0.0, 0.0],
            });
            vertices.push(Vertex {
                position: [triangle.c.x, triangle.c.y, triangle.c.z],
                normal: [0.0, 1.0, 0.0],
                tex_coords: [0.0, 0.0],
                color: [1.0, 0.0, 0.0],
            });

            // Add indices for this triangle
            indices.push(vertex_index);
            indices.push(vertex_index + 1);
            indices.push(vertex_index + 2);

            vertex_index += 3;
        }

        println!("Height range in debug mesh:");
        println!("  Min Y: {}", min_y);
        println!("  Max Y: {}", max_y);
        println!("  Variation: {}", max_y - min_y);

        println!("Debug mesh stats:");
        println!("  Vertices: {}", vertices.len());
        println!("  Indices: {}", indices.len());

        // Create vertex buffer
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Debug Collision Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create index buffer
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Debug Collision Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Some(TerrainMesh {
            mesh_id: "001".to_string(),
            vertex_buffer,
            index_buffer,
            index_count: indices.len() as u32,
            collider: None, // No need for physics on debug mesh
            rigid_body: Some(RigidBodyBuilder::fixed().build()), // Dummy rigid body
            depth: 1,
        })
    } else {
        None
    }
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
