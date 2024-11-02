use nalgebra::{self as na, Matrix4, Vector3};
use std::{f32::consts::PI, sync::Arc};
use wgpu::util::DeviceExt;

use crate::handlers::Vertex;

use super::{
    RendererState::{MouseState, WindowSize},
    SimpleCamera::SimpleCamera,
    Transform::{matrix4_to_raw_array, Transform},
};

// #[derive(Debug)]
pub struct SimpleGizmo {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    pub texture_bind_group: wgpu::BindGroup,
    pub num_indices: u32,
    // position: na::Vector3<f32>,
    // rotation: na::UnitQuaternion<f32>,
    // scale: na::Vector3<f32>,
    pub transform: Transform,
}

impl SimpleGizmo {
    pub fn new(
        device: &wgpu::Device,
        camera: &SimpleCamera,
        window_size: WindowSize,
        bind_group_layout: Arc<wgpu::BindGroupLayout>,
        color_render_mode_buffer: Arc<wgpu::Buffer>,
        texture_bind_group_layout: Arc<wgpu::BindGroupLayout>,
    ) -> Self {
        // Generate a basic gizmo shape (like arrows for each axis)
        let (vertices, indices) = Self::create_gizmo_geometry();

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Gizmo Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Gizmo Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // set uniform buffer for transforms
        let empty_buffer = Matrix4::<f32>::identity();
        let raw_matrix = matrix4_to_raw_array(&empty_buffer);

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Gizmo Uniform Buffer"),
            contents: bytemuck::cast_slice(&raw_matrix),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let default_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Default Empty Texture"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        let default_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let default_texture_view = default_texture.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
            label: None,
        });

        // Handle the texture bind group conditionally
        let texture_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&default_texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&default_sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &color_render_mode_buffer,
                        offset: 0,
                        size: None,
                    }),
                },
            ],
            label: None,
        });

        Self {
            vertex_buffer,
            index_buffer,
            bind_group,
            texture_bind_group,
            num_indices: indices.len() as u32,
            transform: Transform::new(
                Vector3::new(2.0, 2.0, 2.0),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 1.0, 1.0),
                uniform_buffer,
            ),
        }
    }

    fn create_gizmo_geometry() -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // Create arrow shapes for each axis
        Self::add_axis_arrow(&mut vertices, &mut indices, 0); // X axis (red)
        Self::add_axis_arrow(&mut vertices, &mut indices, 1); // Y axis (green)
        Self::add_axis_arrow(&mut vertices, &mut indices, 2); // Z axis (blue)

        (vertices, indices)
    }

    fn add_axis_arrow(vertices: &mut Vec<Vertex>, indices: &mut Vec<u32>, axis: u8) {
        let base_index = vertices.len() as u32;
        let length = 1.0;
        let radius = 0.02;
        let segments = 8;

        // Arrow colors
        let color = match axis {
            0 => [1.0, 0.0, 0.0], // X axis: red
            1 => [0.0, 1.0, 0.0], // Y axis: green
            _ => [0.0, 0.0, 1.0], // Z axis: blue
        };

        // Create arrow shaft
        let mut position = na::Vector3::zeros();
        for i in 0..segments {
            let angle = (i as f32 / segments as f32) * 2.0 * PI;
            let x = radius * angle.cos();
            let y = radius * angle.sin();

            // Transform position based on axis
            match axis {
                0 => position = na::Vector3::new(0.0, x, y),
                1 => position = na::Vector3::new(x, 0.0, y),
                _ => position = na::Vector3::new(x, y, 0.0),
            }

            vertices.push(Vertex {
                position: [position.x, position.y, position.z],
                normal: [0.0, 0.0, 1.0],
                tex_coords: [0.0, 0.0],
                color,
                // Add other vertex attributes as needed
            });
        }

        // Create arrow head (cone)
        let head_radius = radius * 2.0;
        let head_length = length * 0.2;
        let head_position = match axis {
            0 => na::Vector3::new(length, 0.0, 0.0),
            1 => na::Vector3::new(0.0, length, 0.0),
            _ => na::Vector3::new(0.0, 0.0, length),
        };

        vertices.push(Vertex {
            position: [head_position.x, head_position.y, head_position.z],
            normal: [0.0, 0.0, 1.0],
            tex_coords: [0.0, 0.0],
            color,
            // Add other vertex attributes as needed
        });

        // Add indices for the arrow shaft
        for i in 0..segments {
            let next = (i + 1) % segments;
            indices.extend_from_slice(&[
                base_index + i as u32,
                base_index + next as u32,
                base_index + segments as u32,
            ]);
        }
    }

    pub fn update(&mut self, camera: &SimpleCamera, mouse_state: &MouseState) {
        // Add interaction logic here
        // For example, handle dragging, rotation, etc.
    }
}
