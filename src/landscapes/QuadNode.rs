use image::DynamicImage;
use nalgebra::{Isometry3, Matrix4, Point3, Vector3};
use rapier3d::prelude::*;
use rapier3d::prelude::{Collider, ColliderBuilder, RigidBody, RigidBodyBuilder};
use std::collections::HashMap;
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
use crate::landscapes::LandscapeLOD::sample_height_world;

use super::LandscapeLOD::{
    distance_squared, get_camera_distance_from_bound_center_rel, ColliderMessage, Rect,
    PHYSICS_DISTANCE,
};

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

// Create a struct to track vertex info for indexing
struct VertexInfo {
    vertex: Vertex,
    is_edge: bool,
    edge_index: Option<usize>, // Position along the edge if it's an edge vertex
    grid_pos: (u32, u32),      // Position in the overall grid
}

// Represents a node in our quadtree
#[derive(Debug)]
pub struct QuadNode {
    pub bounds: Rect,
    pub depth: u32,
    pub children: Option<Box<[QuadNode; 4]>>,
    pub mesh: Option<TerrainMesh>,
    pub debug_mesh: Option<TerrainMesh>,
    pub rigid_body_handle: Option<RigidBodyHandle>,
    pub collider_handle: Option<ColliderHandle>,
    // Track if this node's LOD state has changed
    pub lod_dirty: bool,
}

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

    pub fn update_lod(
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

    pub fn update_physics_if_needed(
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

    pub fn render<'a>(
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
    pub fn get_resolution_for_depth(depth: u32) -> u32 {
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

    pub fn create_mesh(
        bounds: &Rect,
        height_data: &[f32],
        edge_resolution: u32,
        device: &Device,
        terrain_position: [f32; 3],
        landscape_component_id: String,
        depth: u32,
        collider_sender: Sender<ColliderMessage>,
    ) -> TerrainMesh {
        let mesh_id = Uuid::new_v4().to_string();
        let calc_res = Self::get_resolution_for_depth(depth);
        // let mut vertices = Vec::with_capacity((resolution * resolution) as usize);
        // let mut indices = Vec::with_capacity(((resolution - 1) * (resolution - 1) * 6) as usize);

        // Create a matrix to store heights for Rapier
        // let mut height_matrix = nalgebra::DMatrix::zeros(calc_res as usize, calc_res as usize);
        let width_calc = (height_data.len() as f32).sqrt() as f32;

        println!(
            "width calc {:?} {:?} {:?}",
            width_calc, edge_resolution, calc_res
        );

        let camera = get_camera();
        println!("create mesh, cam pos: {:?}", camera.position);

        let mut rapier_vertices = Vec::new();

        // // Generate vertices
        // for z in 0..calc_res {
        //     for x in 0..calc_res {
        //         // Calculate world position
        //         let world_x = bounds.x + (x as f32 * ((bounds.width) / calc_res as f32));
        //         let world_z = bounds.z + (z as f32 * ((bounds.height) / calc_res as f32));

        //         // Get height at this world position
        //         let height = sample_height_world(world_x, world_z, height_data);
        //         height_matrix[(z as usize, x as usize)] = height + (terrain_position[1] / 2.0); // add if negative position?

        //         // Calculate texture coordinates based on world position
        //         let tex_x = (world_x + width_calc / 2.0) / width_calc;
        //         let tex_z = (world_z + width_calc / 2.0) / width_calc;

        //         vertices.push(Vertex {
        //             position: [world_x, height, world_z],
        //             normal: [0.0, 1.0, 0.0],
        //             tex_coords: [tex_x, tex_z],
        //             color: [1.0, 0.0, 0.0],
        //         });

        //         rapier_vertices.push(Point3::new(world_x, height, world_z));
        //     }
        // }

        // Generate vertices with edge tracking
        let mut vertex_info: Vec<VertexInfo> = Vec::new();

        // Handle edges first - we'll do this exactly once per edge
        // Left edge
        for i in 0..edge_resolution {
            let normalized_z = i as f32 / (edge_resolution) as f32;
            let height = sample_height_world(
                bounds.x,
                bounds.z + (normalized_z * bounds.height),
                height_data,
            );
            let world_pos = [bounds.x, height, bounds.z + (normalized_z * bounds.height)];
            let tex_x = (world_pos[0] + width_calc / 2.0) / width_calc;
            let tex_z = (world_pos[2] + width_calc / 2.0) / width_calc;
            let scaled_pos = (i * (calc_res - 1)) / (edge_resolution - 1);

            // Update height matrix for edge position
            // height_matrix[(0, i as usize)] = height + (terrain_position[1] / 2.0);

            vertex_info.push(VertexInfo {
                vertex: Vertex {
                    position: world_pos,
                    normal: [0.0, 1.0, 0.0],
                    tex_coords: [tex_x, tex_z],
                    color: [1.0, 0.0, 0.0],
                },
                is_edge: true,
                edge_index: Some(i as usize),
                grid_pos: (0, scaled_pos),
            });

            rapier_vertices.push(Point3::new(world_pos[0], world_pos[1], world_pos[2]));
        }

        // Right edge
        for i in 0..edge_resolution {
            let normalized_z = i as f32 / (edge_resolution) as f32;
            let height = sample_height_world(
                bounds.x + bounds.width,
                bounds.z + (normalized_z * bounds.height),
                height_data,
            );
            let world_pos = [
                bounds.x + bounds.width,
                height,
                bounds.z + (normalized_z * bounds.height),
            ];
            let tex_x = (world_pos[0] + width_calc / 2.0) / width_calc;
            let tex_z = (world_pos[2] + width_calc / 2.0) / width_calc;
            let scaled_pos = (i * (calc_res - 1)) / (edge_resolution - 1);

            // Update height matrix for edge position
            // height_matrix[(calc_res - 1, i as usize)] = height + (terrain_position[1] / 2.0);

            vertex_info.push(VertexInfo {
                vertex: Vertex {
                    position: world_pos,
                    normal: [0.0, 1.0, 0.0],
                    tex_coords: [tex_x, tex_z],
                    color: [1.0, 0.0, 0.0],
                },
                is_edge: true,
                edge_index: Some(i as usize),
                grid_pos: (calc_res - 1, scaled_pos),
            });

            rapier_vertices.push(Point3::new(world_pos[0], world_pos[1], world_pos[2]));
        }

        // Top edge
        for i in 0..edge_resolution {
            let normalized_x = i as f32 / (edge_resolution) as f32;
            let world_pos = [
                bounds.x + (normalized_x * bounds.width),
                sample_height_world(
                    bounds.x + (normalized_x * bounds.width),
                    bounds.z,
                    height_data,
                ),
                bounds.z,
            ];
            let tex_x = (world_pos[0] + width_calc / 2.0) / width_calc;
            let tex_z = (world_pos[2] + width_calc / 2.0) / width_calc;
            let scaled_pos = (i * (calc_res - 1)) / (edge_resolution - 1);

            vertex_info.push(VertexInfo {
                vertex: Vertex {
                    position: world_pos,
                    normal: [0.0, 1.0, 0.0],
                    tex_coords: [tex_x, tex_z],
                    color: [1.0, 0.0, 0.0],
                },
                is_edge: true,
                edge_index: Some(i as usize),
                grid_pos: (scaled_pos, 0),
            });

            rapier_vertices.push(Point3::new(world_pos[0], world_pos[1], world_pos[2]));
        }

        // Bottom edge
        for i in 0..edge_resolution {
            let normalized_x = i as f32 / (edge_resolution) as f32;
            let world_pos = [
                bounds.x + (normalized_x * bounds.width),
                sample_height_world(
                    bounds.x + (normalized_x * bounds.width),
                    bounds.z + bounds.height,
                    height_data,
                ),
                bounds.z + bounds.height,
            ];
            let tex_x = (world_pos[0] + width_calc / 2.0) / width_calc;
            let tex_z = (world_pos[2] + width_calc / 2.0) / width_calc;
            let scaled_pos = (i * (calc_res - 1)) / (edge_resolution - 1);

            vertex_info.push(VertexInfo {
                vertex: Vertex {
                    position: world_pos,
                    normal: [0.0, 1.0, 0.0],
                    tex_coords: [tex_x, tex_z],
                    color: [1.0, 0.0, 0.0],
                },
                is_edge: true,
                edge_index: Some(i as usize),
                grid_pos: (scaled_pos, calc_res - 1),
            });

            rapier_vertices.push(Point3::new(world_pos[0], world_pos[1], world_pos[2]));
        }

        // Now handle interior vertices
        for z in 0..calc_res {
            for x in 0..calc_res {
                let normalized_x = x as f32 / (calc_res) as f32;
                let normalized_z = z as f32 / (calc_res) as f32;
                let world_pos = [
                    bounds.x + (normalized_x * bounds.width),
                    sample_height_world(
                        bounds.x + (normalized_x * bounds.width),
                        bounds.z + (normalized_z * bounds.height),
                        height_data,
                    ),
                    bounds.z + (normalized_z * bounds.height),
                ];
                let tex_x = (world_pos[0] + width_calc / 2.0) / width_calc;
                let tex_z = (world_pos[2] + width_calc / 2.0) / width_calc;

                vertex_info.push(VertexInfo {
                    vertex: Vertex {
                        position: world_pos,
                        normal: [0.0, 1.0, 0.0],
                        tex_coords: [tex_x, tex_z],
                        color: [1.0, 0.0, 0.0],
                    },
                    is_edge: false,
                    edge_index: None,
                    grid_pos: (x, z),
                });

                rapier_vertices.push(Point3::new(world_pos[0], world_pos[1], world_pos[2]));
            }
        }

        // Create final vertex buffer from vertex_info
        let vertices: Vec<Vertex> = vertex_info.iter().map(|info| info.vertex.clone()).collect();

        println!("height: {:?}", vertices[1].position[1]);
        println!("vertices length {:?}", vertices.len());

        // // Generate indices for triangle strips
        // for z in 0..calc_res - 1 {
        //     for x in 0..calc_res - 1 {
        //         let top_left = z * calc_res + x;
        //         let top_right = top_left + 1;
        //         let bottom_left = (z + 1) * calc_res + x;
        //         let bottom_right = bottom_left + 1;

        //         // First triangle (top-left, bottom-left, top-right)
        //         indices.push(top_left);
        //         indices.push(bottom_left);
        //         indices.push(top_right);

        //         // Second triangle (top-right, bottom-left, bottom-right)
        //         indices.push(top_right);
        //         indices.push(bottom_left);
        //         indices.push(bottom_right);

        //         // // Additional connections - are these needed??
        //         // if x < calc_res - 2 {
        //         //     // Connect to next column, but only if we're not at the quad edge
        //         //     indices.extend_from_slice(&[
        //         //         top_right as u32,
        //         //         bottom_right as u32,
        //         //         top_right + 1 as u32,
        //         //     ]);
        //         //     indices.extend_from_slice(&[
        //         //         bottom_right as u32,
        //         //         bottom_right + 1 as u32,
        //         //         top_right + 1 as u32,
        //         //     ]);
        //         // }

        //         // if z < calc_res - 2 {
        //         //     // Connect to next row, but only if we're not at the quad edge
        //         //     indices.extend_from_slice(&[
        //         //         bottom_left as u32,
        //         //         bottom_left + calc_res as u32,
        //         //         bottom_right as u32,
        //         //     ]);
        //         //     indices.extend_from_slice(&[
        //         //         bottom_right as u32,
        //         //         bottom_left + calc_res as u32,
        //         //         bottom_right + calc_res as u32,
        //         //     ]);
        //         // }
        //     }
        // }

        // Create a fast lookup map after generating vertices
        let vertex_lookup: HashMap<(u32, u32), usize> = vertex_info
            .iter()
            .enumerate()
            .map(|(index, info)| (info.grid_pos, index))
            .collect();

        // Fast vertex lookup
        let mut indices = Vec::new();

        // Single loop for all indices since grid positions are now normalized to calc_res
        for z in 0..calc_res - 1 {
            for x in 0..calc_res - 1 {
                let top_left = vertex_lookup.get(&(x, z)).expect("Vertex should exist");
                let top_right = vertex_lookup.get(&(x + 1, z)).expect("Vertex should exist");
                let bottom_left = vertex_lookup.get(&(x, z + 1)).expect("Vertex should exist");
                let bottom_right = vertex_lookup
                    .get(&(x + 1, z + 1))
                    .expect("Vertex should exist");

                indices.extend_from_slice(&[
                    *top_left as u32,
                    *bottom_left as u32,
                    *top_right as u32,
                    *top_right as u32,
                    *bottom_left as u32,
                    *bottom_right as u32,
                ]);
            }
        }

        println!("adding buffers");

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
        // let heights = height_matrix;

        // let scaling = vector![
        //     bounds.width as f32,
        //     1.0, // Height scale
        //     bounds.height as f32
        // ];

        // let translation = vector![bounds.x, 0.0, bounds.z];
        // let translation = vector![0.0, -220.0, 0.0];

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

            println!("spawing collider");

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
