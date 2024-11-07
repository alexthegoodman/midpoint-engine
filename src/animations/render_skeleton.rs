use std::{collections::HashMap, sync::Arc};

use nalgebra::{Matrix4, Point3, Rotation3, UnitVector3, Vector3};
use wgpu::util::DeviceExt;

use crate::{
    core::Transform::{matrix4_to_raw_array, Transform},
    handlers::Vertex,
};

use super::skeleton::{Joint, SkeletonPart};

// Vertices for a pyramid
const VERTICES: &[Vertex] = &[
    Vertex {
        position: [0.0, 1.0, 0.0],
        normal: [0.0, 0.0, 0.0],
        tex_coords: [0.0, 0.0],
        color: [1.0, 0.0, 0.0],
    }, // Apex
    Vertex {
        position: [-1.0, -1.0, -1.0],
        normal: [0.0, 0.0, 0.0],
        tex_coords: [0.0, 0.0],
        color: [0.0, 1.0, 0.0],
    }, // Base vertices
    Vertex {
        position: [1.0, -1.0, -1.0],
        normal: [0.0, 0.0, 0.0],
        tex_coords: [0.0, 0.0],
        color: [0.0, 0.0, 1.0],
    },
    Vertex {
        position: [1.0, -1.0, 1.0],
        normal: [0.0, 0.0, 0.0],
        tex_coords: [0.0, 0.0],
        color: [1.0, 1.0, 0.0],
    },
    Vertex {
        position: [-1.0, -1.0, 1.0],
        normal: [0.0, 0.0, 0.0],
        tex_coords: [0.0, 0.0],
        color: [0.0, 1.0, 1.0],
    },
];

// Indices for a pyramid
const INDICES: &[u16] = &[
    0, 1, 2, // Side 1
    0, 2, 3, // Side 2
    0, 3, 4, // Side 3
    0, 4, 1, // Side 4
    1, 3, 2, // Base 1
    1, 4, 3, // Base 2
];

// #[derive(Clone)]
pub struct BoneSegment {
    pub start_joint_id: String,
    pub end_joint_id: String,
    pub transform: Transform,

    // WGPU Buffers and binding information
    /// Vertex buffer containing the base pyramid shape
    pub vertex_buffer: wgpu::Buffer,
    /// Index buffer for the pyramid geometry
    pub index_buffer: wgpu::Buffer,
    /// Number of indices in the base pyramid shape
    pub num_indices: u32,
    /// Bind group for this skeleton part's resources
    pub bind_group: wgpu::BindGroup,

    pub length: f32,
    pub color: [f32; 4],
}

impl BoneSegment {
    /// Creates a new bone segment between two joint positions
    pub fn new(
        device: &wgpu::Device,
        bind_group_layout: &wgpu::BindGroupLayout,
        start_joint_id: String,
        end_joint_id: String,
        start_pos: Point3<f32>,
        end_pos: Point3<f32>,
    ) -> Self {
        // Calculate bone properties from joint positions
        let bone_vector = end_pos - start_pos;
        let length = bone_vector.magnitude();

        // Calculate rotation to align bone with direction vector
        let rotation = Self::calculate_bone_rotation(&bone_vector);

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("BoneSegment Vertex Buffer"),
            contents: bytemuck::cast_slice(VERTICES),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("BoneSegment Index Buffer"),
            contents: bytemuck::cast_slice(INDICES),
            usage: wgpu::BufferUsages::INDEX,
        });

        let empty_buffer = Matrix4::<f32>::identity();
        let raw_matrix = matrix4_to_raw_array(&empty_buffer);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pyramid Uniform Buffer"),
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

        // Create transform
        // Position is at start joint
        // Scale x and z are small to make thin bones, y is length
        // Rotation aligns bone with direction vector
        let transform = Transform::new(
            start_pos.coords,               // position at start joint
            rotation,                       // rotation to align with bone direction
            Vector3::new(0.1, length, 0.1), // scale to create bone shape
            uniform_buffer,
        );

        Self {
            start_joint_id,
            end_joint_id,
            transform,
            length,
            color: [0.8, 0.8, 0.8, 1.0],
            vertex_buffer,
            index_buffer,
            num_indices: INDICES.len() as u32,
            bind_group,
        }
    }

    /// Updates the bone transform based on new joint positions
    pub fn update_from_joint_positions(&mut self, start_pos: Point3<f32>, end_pos: Point3<f32>) {
        let bone_vector = end_pos - start_pos;
        let length = bone_vector.magnitude();

        let rotation = Self::calculate_bone_rotation(&bone_vector);

        self.transform.update_position(start_pos.coords.into());
        self.transform.update_rotation(rotation.into());
        self.transform.update_scale([0.1, length, 0.1]);
        self.length = length;
    }

    /// Calculates rotation to align bone with direction vector
    fn calculate_bone_rotation(bone_vector: &Vector3<f32>) -> Vector3<f32> {
        // Default bone direction (assuming bone model points up along Y axis)
        let default_direction = Vector3::new(0.0, 1.0, 0.0);

        // If bone vector is zero or nearly zero, return no rotation
        if bone_vector.magnitude() < 1e-6 {
            return Vector3::zeros();
        }

        // Get unit vector in bone direction
        let bone_direction = UnitVector3::new_normalize(*bone_vector);

        // Calculate rotation between default direction and bone direction
        let rotation = Rotation3::rotation_between(&default_direction, &bone_direction)
            .unwrap_or(Rotation3::identity());

        // Convert rotation matrix to euler angles
        let euler = rotation.euler_angles();
        Vector3::new(euler.0, euler.1, euler.2)
    }
}

/// Complete render information for a skeleton part
pub struct SkeletonRenderPart {
    /// Reference to the skeleton part this renders
    pub skeleton_part_id: String,
    /// All bone segments in this part
    pub bones: Vec<BoneSegment>,

    // Cache of computed data
    /// Current world space positions of joints
    pub joint_positions: HashMap<String, nalgebra::Point3<f32>>,
}

impl SkeletonRenderPart {
    pub fn new(part_id: String) -> Self {
        SkeletonRenderPart {
            skeleton_part_id: part_id,
            bones: Vec::new(),
            joint_positions: HashMap::new(),
        }
    }

    /// Updates bone transforms based on current joint positions
    pub fn update_bones(
        &mut self,
        joint_positions: &HashMap<String, Point3<f32>>,
        queue: &wgpu::Queue,
    ) {
        for bone in &mut self.bones {
            if let (Some(start_pos), Some(end_pos)) = (
                joint_positions.get(&bone.start_joint_id),
                joint_positions.get(&bone.end_joint_id),
            ) {
                // Update bone transform based on new joint positions
                bone.update_from_joint_positions(*start_pos, *end_pos);
                // Update GPU buffer
                bone.transform.update_uniform_buffer(queue);
            }
        }
    }

    /// Creates bone segments from joint hierarchy
    pub fn create_bone_segments(
        &mut self,
        device: &wgpu::Device,
        bind_group_layout: &wgpu::BindGroupLayout,
        joints: Vec<Joint>,
        joint_positions: &HashMap<String, Point3<f32>>,
    ) {
        let mut bones = Vec::new();

        for joint in joints {
            if let Some(parent_id) = &joint.parent_id {
                if let (Some(start_pos), Some(end_pos)) = (
                    joint_positions.get(parent_id),
                    joint_positions.get(&joint.id),
                ) {
                    let bone = BoneSegment::new(
                        device,
                        bind_group_layout,
                        parent_id.clone(),
                        joint.id.clone(),
                        *start_pos,
                        *end_pos,
                    );
                    bones.push(bone);
                }
            }
        }

        // bones
        self.bones = bones;
    }
}
