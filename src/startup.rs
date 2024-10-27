use nalgebra::{Matrix4, Point3, Vector3};
use serde::{Deserialize, Serialize};
use tokio::spawn;
use wgpu::util::DeviceExt;
use winit::{
    dpi::LogicalSize,
    event::*,
    event_loop::{self, ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use bytemuck::{Pod, Zeroable};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use super::shapes::Cube::Cube;
use crate::{
    core::{Grid::Grid, RendererState::RendererState},
    helpers::landscapes::read_landscape_mask,
};
use crate::{
    core::{
        RendererState::{get_renderer_state, initialize_renderer_state},
        SimpleCamera::SimpleCamera,
    },
    helpers::landscapes::read_landscape_texture,
};
use crate::{
    core::{
        RendererState::{pause_rendering, resume_rendering},
        Texture::Texture,
    },
    helpers::saved_data::LandscapeTextureKinds,
};
use crate::{helpers::landscapes::get_landscape_pixels, landscapes::Landscape::Landscape};
use crate::{
    helpers::landscapes::LandscapePixelData,
    models::Model::{Mesh, Model},
};
use crate::{models::Model::read_model, shapes::Pyramid::Pyramid};

#[derive(Serialize)]
pub struct ReadModelParams {
    pub projectId: String,
    pub modelFilename: String,
}

#[derive(Serialize)]
pub struct GetLandscapeParams {
    pub projectId: String,
    pub landscapeAssetId: String,
    pub landscapeFilename: String,
}

#[derive(Serialize)]
pub struct GetTextureParams {
    pub projectId: String,
    pub landscapeId: String,
    pub textureFilename: String,
    pub textureKind: String,
}

#[derive(Serialize)]
pub struct GetMaskParams {
    pub projectId: String,
    pub landscapeId: String,
    pub maskFilename: String,
    pub maskKind: String,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub tex_coords: [f32; 2],
    pub color: [f32; 3],
}

// Ensure Vertex is Pod and Zeroable
unsafe impl Pod for Vertex {}
unsafe impl Zeroable for Vertex {}

impl Vertex {
    const ATTRIBS: [wgpu::VertexAttribute; 4] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2, 3 => Float32x3];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

static mut CAMERA: Option<SimpleCamera> = None;

thread_local! {
    static CAMERA_INIT: std::cell::Cell<bool> = std::cell::Cell::new(false);
}

pub fn get_camera() -> &'static mut SimpleCamera {
    CAMERA_INIT.with(|init| {
        if !init.get() {
            unsafe {
                CAMERA = Some(SimpleCamera::new(
                    Point3::new(0.0, 1.0, 5.0),
                    Vector3::new(0.0, 0.0, -1.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    45.0f32.to_radians(),
                    0.1,
                    10000.0,
                ));
            }
            init.set(true);
        }
    });

    unsafe { CAMERA.as_mut().unwrap() }
}

// pub async fn start_render_loop() {
//     // Create logical components (instance, adapter, device, queue, surface, etc.)
//     let dx12_compiler = wgpu::Dx12Compiler::Dxc {
//         dxil_path: None, // Specify a path to custom location
//         dxc_path: None,  // Specify a path to custom location
//     };

//     let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
//         backends: wgpu::Backends::PRIMARY,
//         dx12_shader_compiler: dx12_compiler,
//         flags: wgpu::InstanceFlags::empty(),
//         gles_minor_version: wgpu::Gles3MinorVersion::Version2,
//     });

//     let height = canvas.height();
//     let width = canvas.width();

//     let event_loop = event_loop::EventLoop::new().unwrap();
//     let builder = WindowBuilder::new().with_inner_size(LogicalSize::new(width, height));
//     #[cfg(target_arch = "wasm32")] // necessary for web-sys
//     let builder = {
//         use winit::platform::web::WindowBuilderExtWebSys;
//         builder.with_canvas(Some(canvas))
//     };
//     let winit_window = builder.build(&event_loop).unwrap();

//     let surface = unsafe {
//         instance
//             .create_surface(winit_window)
//             .expect("Couldn't create GPU Surface")
//     };

//     let adapter = instance
//         .request_adapter(&wgpu::RequestAdapterOptions {
//             power_preference: wgpu::PowerPreference::default(),
//             compatible_surface: Some(&surface),
//             force_fallback_adapter: false,
//         })
//         .await
//         .ok_or("Failed to find an appropriate adapter")
//         .unwrap();

//     let (device, queue) = adapter
//         .request_device(
//             &wgpu::DeviceDescriptor {
//                 label: None,
//                 required_features: wgpu::Features::empty(),
//                 required_limits: wgpu::Limits::default(),
//             },
//             None,
//         )
//         .await
//         .expect("Failed to create device");

//     let device = Arc::new(device);
//     let queue = Arc::new(queue);

//     // let swap_chain_format: wgpu::TextureFormat = surface.get_preferred_format(&adapter).unwrap();
//     let swapchain_capabilities = surface.get_capabilities(&adapter);
//     let swap_chain_format = swapchain_capabilities.formats[0]; // Choosing the first available format

//     let mut config = surface.get_default_config(&adapter, width, height).unwrap();
//     surface.configure(&device, &config);

//     // Create the shader module
//     let vertex_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
//         label: Some("Vertex Shader"),
//         source: wgpu::ShaderSource::Wgsl(include_str!("./shaders/primary_vertex.wgsl").into()),
//     });

//     let fragment_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
//         label: Some("Fragment Shader"),
//         source: wgpu::ShaderSource::Wgsl(include_str!("./shaders/primary_fragment.wgsl").into()),
//     });

//     let camera = get_camera();

//     camera.update_aspect_ratio(config.width as f32 / config.height as f32);
//     camera.update_view_projection_matrix();

//     let camera_matrix = camera.view_projection_matrix;
//     let camera_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
//         label: Some("Camera Uniform Buffer"),
//         contents: bytemuck::cast_slice(camera_matrix.as_slice()),
//         usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
//     });

//     // Create the bind group for the uniform buffer
//     let camera_bind_group_layout =
//         device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
//             label: Some("Bind Group Layout"),
//             entries: &[wgpu::BindGroupLayoutEntry {
//                 binding: 0,
//                 visibility: wgpu::ShaderStages::VERTEX,
//                 ty: wgpu::BindingType::Buffer {
//                     ty: wgpu::BufferBindingType::Uniform,
//                     has_dynamic_offset: false,
//                     min_binding_size: None,
//                 },
//                 count: None,
//             }],
//         });

//     let model_bind_group_layout =
//         device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
//             entries: &[wgpu::BindGroupLayoutEntry {
//                 binding: 0,
//                 visibility: wgpu::ShaderStages::VERTEX,
//                 ty: wgpu::BindingType::Buffer {
//                     ty: wgpu::BufferBindingType::Uniform,
//                     has_dynamic_offset: false,
//                     min_binding_size: None,
//                 },
//                 count: None,
//             }],
//             label: Some("model_bind_group_layout"),
//         });

//     let model_bind_group_layout = Arc::new(model_bind_group_layout);

//     let texture_bind_group_layout =
//         device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
//             entries: &[
//                 wgpu::BindGroupLayoutEntry {
//                     binding: 0,
//                     visibility: wgpu::ShaderStages::FRAGMENT,
//                     ty: wgpu::BindingType::Texture {
//                         multisampled: false,
//                         view_dimension: wgpu::TextureViewDimension::D2Array,
//                         sample_type: wgpu::TextureSampleType::Float { filterable: true },
//                     },
//                     count: None,
//                 },
//                 wgpu::BindGroupLayoutEntry {
//                     binding: 1,
//                     visibility: wgpu::ShaderStages::FRAGMENT,
//                     ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
//                     count: None,
//                 },
//                 wgpu::BindGroupLayoutEntry {
//                     binding: 2,
//                     visibility: wgpu::ShaderStages::FRAGMENT,
//                     ty: wgpu::BindingType::Buffer {
//                         ty: wgpu::BufferBindingType::Uniform,
//                         has_dynamic_offset: false,
//                         min_binding_size: None,
//                     },
//                     count: None,
//                 },
//             ],
//             label: Some("Texture Bind Group Layout"),
//         });

//     let texture_bind_group_layout = Arc::new(texture_bind_group_layout);

//     let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
//         layout: &camera_bind_group_layout,
//         entries: &[wgpu::BindGroupEntry {
//             binding: 0,
//             resource: camera_uniform_buffer.as_entire_binding(),
//         }],
//         label: Some("Bind Group"),
//     });

//     let color_render_mode_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
//         label: Some("Color Render Mode Buffer"),
//         contents: bytemuck::cast_slice(&[0i32]), // Default to normal mode
//         usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
//     });

//     let color_render_mode_buffer = Arc::new(color_render_mode_buffer);

//     let texture_render_mode_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
//         label: Some("Texture Render Mode Buffer"),
//         contents: bytemuck::cast_slice(&[1i32]), // Default to text mode
//         usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
//     });

//     let texture_render_mode_buffer = Arc::new(texture_render_mode_buffer);

//     let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
//         label: Some("Render Pipeline Layout"),
//         bind_group_layouts: &[
//             &camera_bind_group_layout,
//             &model_bind_group_layout,
//             &texture_bind_group_layout,
//         ],
//         push_constant_ranges: &[],
//     });

//     let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
//         size: wgpu::Extent3d {
//             width: width,
//             height: height,
//             depth_or_array_layers: 1,
//         },
//         mip_level_count: 1,
//         sample_count: 1,
//         dimension: wgpu::TextureDimension::D2,
//         format: wgpu::TextureFormat::Depth24Plus,
//         usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
//         label: Some("Depth Texture"),
//         view_formats: &[],
//     });

//     let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

//     let depth_stencil_state = wgpu::DepthStencilState {
//         format: wgpu::TextureFormat::Depth24Plus,
//         depth_write_enabled: true,
//         depth_compare: wgpu::CompareFunction::Less,
//         stencil: wgpu::StencilState::default(),
//         bias: wgpu::DepthBiasState::default(),
//     };

//     // Create the render pipeline
//     let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
//         label: Some("Render Pipeline"),
//         layout: Some(&pipeline_layout),
//         vertex: wgpu::VertexState {
//             module: &vertex_shader,
//             entry_point: "main",
//             buffers: &[Vertex::desc()],
//             compilation_options: wgpu::PipelineCompilationOptions {
//                 ..Default::default()
//             },
//         },
//         fragment: Some(wgpu::FragmentState {
//             module: &fragment_shader,
//             entry_point: "main",
//             targets: &[Some(wgpu::ColorTargetState {
//                 format: swap_chain_format,
//                 blend: Some(wgpu::BlendState::REPLACE),
//                 write_mask: wgpu::ColorWrites::ALL,
//             })],
//             compilation_options: wgpu::PipelineCompilationOptions {
//                 ..Default::default()
//             },
//         }),
//         primitive: wgpu::PrimitiveState {
//             topology: wgpu::PrimitiveTopology::TriangleList,
//             // strip_index_format: Some(wgpu::IndexFormat::Uint32),
//             strip_index_format: None,
//             front_face: wgpu::FrontFace::Ccw,
//             cull_mode: Some(wgpu::Face::Back),
//             // cull_mode: None,
//             polygon_mode: wgpu::PolygonMode::Fill,
//             unclipped_depth: false,
//             conservative: false,
//         },
//         // depth_stencil: None,
//         depth_stencil: Some(depth_stencil_state),
//         multisample: wgpu::MultisampleState {
//             count: 1,
//             mask: !0,
//             alpha_to_coverage_enabled: false,
//         },
//         multiview: None,
//     });

//     let state = RendererState::new(
//         device.clone(),
//         queue.clone(),
//         model_bind_group_layout.clone(),
//         texture_bind_group_layout.clone(),
//         texture_render_mode_buffer.clone(),
//         color_render_mode_buffer.clone(),
//     )
//     .await;

//     initialize_renderer_state(state);

//     let state = get_renderer_state();
//     // let mut state_guard = get_renderer_state_read_lock();

//     // // web-based rendering loop
//     // let f = Rc::new(RefCell::new(None));
//     // let g = f.clone();

//     // let closure = Closure::wrap(Box::new(move || {
//     //     if !is_rendering_paused() {
//     //         let device = device.clone();
//     //         let state_guard = state.lock().unwrap();

//     //         render_frame(
//     //             &state_guard,
//     //             &surface,
//     //             &device,
//     //             &queue,
//     //             &render_pipeline,
//     //             &depth_view,
//     //             &camera_bind_group,
//     //             &camera_uniform_buffer,
//     //         );

//     //         drop(state_guard);
//     //     }

//     //     // Schedule the next frame
//     //     // request_animation_frame(f.borrow().as_ref().unwrap());
//     // }) as Box<dyn FnMut()>);

//     // *g.borrow_mut() = Some(closure);

//     // Start the rendering loop
//     // request_animation_frame(g.borrow().as_ref().unwrap());
// }

// fn render_frame(
//     state: &RendererState,
//     surface: &wgpu::Surface,
//     device: &wgpu::Device,
//     queue: &wgpu::Queue,
//     render_pipeline: &wgpu::RenderPipeline,
//     depth_view: &wgpu::TextureView,
//     camera_bind_group: &wgpu::BindGroup,
//     camera_uniform_buffer: &wgpu::Buffer,
// ) {
//     // draw frames...
//     let mut camera = get_camera();

//     // Render a frame
//     let frame = surface
//         .get_current_texture()
//         .expect("Failed to acquire next swap chain texture");
//     let view = frame
//         .texture
//         .create_view(&wgpu::TextureViewDescriptor::default());

//     let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
//         label: Some("Render Encoder"),
//     });

//     {
//         let color = wgpu::Color {
//             r: 0.1,
//             g: 0.2,
//             b: 0.3,
//             a: 1.0,
//         };
//         let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
//             label: Some("Render Pass"),
//             color_attachments: &[Some(wgpu::RenderPassColorAttachment {
//                 view: &view,
//                 resolve_target: None,
//                 ops: wgpu::Operations {
//                     load: wgpu::LoadOp::Clear(color),
//                     store: wgpu::StoreOp::Store,
//                 },
//             })],
//             // depth_stencil_attachment: None,
//             depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
//                 view: &depth_view, // This is the depth texture view
//                 depth_ops: Some(wgpu::Operations {
//                     load: wgpu::LoadOp::Clear(1.0), // Clear to max depth
//                     store: wgpu::StoreOp::Store,
//                 }),
//                 stencil_ops: None, // Set this if using stencil
//             }),
//             timestamp_writes: None,
//             occlusion_query_set: None,
//         });

//         // draw calls...
//         render_pass.set_pipeline(&render_pipeline);

//         camera.update();
//         let camera_matrix = camera.view_projection_matrix;
//         queue.write_buffer(
//             &camera_uniform_buffer,
//             0,
//             bytemuck::cast_slice(camera_matrix.as_slice()),
//         );

//         // draw utility grids
//         for grid in &state.grids {
//             render_pass.set_bind_group(0, &camera_bind_group, &[]);
//             render_pass.set_bind_group(1, &grid.bind_group, &[]);
//             render_pass.set_bind_group(2, &grid.texture_bind_group, &[]);

//             render_pass.set_vertex_buffer(0, grid.vertex_buffer.slice(..));
//             render_pass.set_index_buffer(grid.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

//             render_pass.draw_indexed(0..grid.index_count, 0, 0..1);
//         }

//         // // draw pyramids
//         // for pyramid in &state.pyramids {
//         //     pyramid.update_uniform_buffer(&queue);
//         //     render_pass.set_bind_group(0, &camera_bind_group, &[]);
//         //     render_pass.set_bind_group(1, &pyramid.bind_group, &[]);

//         //     render_pass.set_vertex_buffer(0, pyramid.vertex_buffer.slice(..));
//         //     render_pass.set_index_buffer(pyramid.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

//         //     render_pass.draw_indexed(0..INDICES.len() as u32, 0, 0..1);
//         // }

//         // draw cubes
//         for cube in &state.cubes {
//             cube.transform.update_uniform_buffer(&queue);
//             render_pass.set_bind_group(0, &camera_bind_group, &[]);
//             render_pass.set_bind_group(1, &cube.bind_group, &[]);

//             render_pass.set_vertex_buffer(0, cube.vertex_buffer.slice(..));
//             render_pass.set_index_buffer(cube.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

//             render_pass.draw_indexed(0..cube.index_count as u32, 0, 0..1);
//         }

//         // web_sys::console::log_1(&"Model count...".into());
//         // web_sys::console::log_1(&state.models.len().into());

//         for model in &state.models {
//             for mesh in &model.meshes {
//                 mesh.transform.update_uniform_buffer(&queue);
//                 render_pass.set_bind_group(0, &camera_bind_group, &[]);
//                 render_pass.set_bind_group(1, &mesh.bind_group, &[]);
//                 render_pass.set_bind_group(2, &mesh.texture_bind_group, &[]);

//                 render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
//                 render_pass
//                     .set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

//                 render_pass.draw_indexed(0..mesh.index_count as u32, 0, 0..1);
//             }
//         }

//         for landscape in &state.landscapes {
//             if (landscape.texture_bind_group.is_some()) {
//                 landscape.transform.update_uniform_buffer(&queue);
//                 render_pass.set_bind_group(0, &camera_bind_group, &[]);
//                 render_pass.set_bind_group(1, &landscape.bind_group, &[]);
//                 render_pass.set_bind_group(
//                     2,
//                     &landscape
//                         .texture_bind_group
//                         .as_ref()
//                         .expect("No landscape texture bind group"),
//                     &[],
//                 );

//                 render_pass.set_vertex_buffer(0, landscape.vertex_buffer.slice(..));
//                 render_pass
//                     .set_index_buffer(landscape.index_buffer.slice(..), wgpu::IndexFormat::Uint32);

//                 render_pass.draw_indexed(0..landscape.index_count as u32, 0, 0..1);
//             }
//         }
//     }

//     queue.submit(Some(encoder.finish()));
//     frame.present();
// }

pub fn handle_key_press(key_code: String, is_pressed: bool) {
    let camera = get_camera();
    let state = get_renderer_state();
    let mut state_guard = state.lock().unwrap();

    // web_sys::console::log_1(&format!("Key pressed (2): {}", key_code).into());

    match key_code.as_str() {
        "w" => {
            if is_pressed {
                // Handle the key press for W
                // web_sys::console::log_1(&"Key W pressed".into());
                camera.position += camera.direction * 0.1;
            }
        }
        "s" => {
            if is_pressed {
                // Handle the key press for S
                // web_sys::console::log_1(&"Key S pressed".into());
                camera.position -= camera.direction * 0.1;
            }
        }
        "a" => {
            if is_pressed {
                // Handle the key press for A
                // web_sys::console::log_1(&"Key A pressed".into());
                let right = camera.direction.cross(&camera.up).normalize();
                camera.position -= right * 0.1;
            }
        }
        "d" => {
            if is_pressed {
                // Handle the key press for D
                // web_sys::console::log_1(&"Key D pressed".into());
                let right = camera.direction.cross(&camera.up).normalize();
                camera.position += right * 0.1;
            }
        }
        "ArrowUp" => {
            if is_pressed {
                // Handle the key press for ArrowUp
                // web_sys::console::log_1(&"Key ArrowUp pressed".into());
                // state.pyramids[0].translate(Vector3::new(0.0, 0.1, 0.0));
                // test rotation
                // state.pyramids[0].rotate(Vector3::new(0.0, 0.1, 0.0));
                // test scale
                // state.pyramids[0].scale(Vector3::new(1.1, 1.1, 1.1));

                if state_guard.models.len() > 0 {
                    state_guard.models[0].meshes[0]
                        .transform
                        .translate(Vector3::new(0.0, 0.1, 0.0));
                }
            }
        }
        "ArrowDown" => {
            if is_pressed {
                // Handle the key press for ArrowDown
                // web_sys::console::log_1(&"Key ArrowDown pressed".into());
                // state.pyramids[0].translate(Vector3::new(0.0, -0.1, 0.0));
            }
        }
        "ArrowLeft" => {
            if is_pressed {
                // Handle the key press for ArrowLeft
                // web_sys::console::log_1(&"Key ArrowLeft pressed".into());
                // state.pyramids[0].translate(Vector3::new(-0.1, 0.0, 0.0));
            }
        }
        "ArrowRight" => {
            if is_pressed {
                // Handle the key press for ArrowRight
                // web_sys::console::log_1(&"Key ArrowRight pressed".into());
                // state.pyramids[0].translate(Vector3::new(0.1, 0.0, 0.0));
            }
        }
        _ => {
            // Handle any other keys if necessary
        }
    }

    camera.update();
}

pub fn handle_mouse_move(dx: f32, dy: f32) {
    let camera = get_camera();
    let sensitivity = 0.005;

    let dx = -dx * sensitivity;
    let dy = dy * sensitivity;

    camera.rotate(dx, dy);

    camera.update();
}

pub async fn handle_add_model(projectId: String, modelFilename: String) {
    pause_rendering();

    let state = get_renderer_state();

    // TODO: this spawn and async may be unncessary
    // spawn(async move {
    // let mut state_guard = get_renderer_state_write_lock();

    let mut state_guard = state.lock().unwrap();

    // let params = to_value(&ReadModelParams {
    //     projectId,
    //     modelFilename,
    // })
    // .unwrap();
    // // let bytes = crate::app::invoke("read_model", params).await;
    // let bytes = invoke("read_model", params).await;
    // let bytes = bytes
    //     .into_serde()
    //     .expect("Failed to transform byte string to value");

    let bytes = read_model(projectId, modelFilename)
        .await
        .expect("Couldn't get model bytes");

    state_guard.add_model(&bytes).await;

    drop(state_guard);

    resume_rendering();
    // });
}

#[derive(Serialize, Deserialize)]
pub struct LandscapeData {
    // pub width: usize,
    // pub height: usize,
    pub width: usize,
    pub height: usize,
    // pub data: Vec<u8>,
    pub pixel_data: Vec<Vec<PixelData>>,
}

#[derive(Serialize, Deserialize)]
pub struct PixelData {
    pub height_value: f32,
    pub position: [f32; 3],
    pub tex_coords: [f32; 2],
}

pub async fn handle_add_landscape(
    projectId: String,
    landscapeAssetId: String,
    landscapeComponentId: String,
    landscapeFilename: String,
    // callback: js_sys::Function,
) {
    pause_rendering();

    let state = get_renderer_state();

    // spawn(async move {
    // let params = to_value(&GetLandscapeParams {
    //     projectId,
    //     landscapeAssetId,
    //     landscapeFilename,
    // })
    // .unwrap();

    // let js_data = invoke("get_landscape_pixels", params).await;
    // let data: LandscapeData = js_data
    //     .into_serde()
    //     .expect("Failed to transform byte string to value");

    let mut state_guard = state.lock().unwrap();

    let data = get_landscape_pixels(projectId, landscapeAssetId, landscapeFilename);

    state_guard.add_landscape(&landscapeComponentId, &data);

    drop(state_guard);

    resume_rendering();

    // let this = JsValue::null();
    // let _ = callback.call0(&this);
    // });
}

pub async fn handle_add_landscape_texture(
    project_id: String,
    landscape_component_id: String,
    landscape_asset_id: String,
    texture_filename: String,
    texture_kind: String,
    mask_filename: String,
) {
    pause_rendering();

    let state = get_renderer_state();

    // Clone the values that need to be moved into the closure
    let landscape_component_id_clone = landscape_component_id.clone();
    let texture_kind_clone = texture_kind.clone();

    // spawn(async move {
    let mut state_guard = state.lock().unwrap();

    let texture = fetch_texture_data(
        project_id.clone(),
        landscape_asset_id.clone(),
        texture_filename,
        texture_kind.clone(),
    )
    .await;
    let mask = fetch_mask_data(
        project_id.clone(),
        landscape_asset_id.clone(),
        mask_filename,
        texture_kind.clone(),
    )
    .await;

    // if let Some(texture) = texture {
    let kind = match texture_kind_clone.as_str() {
        "Primary" => LandscapeTextureKinds::Primary,
        "Rockmap" => LandscapeTextureKinds::Rockmap,
        "Soil" => LandscapeTextureKinds::Soil,
        _ => {
            // web_sys::console::error_1(
            //     &format!("Invalid texture kind: {}", texture_kind_clone).into(),
            // );
            return;
        }
    };

    let maskKind = match texture_kind_clone.as_str() {
        "Primary" => LandscapeTextureKinds::PrimaryMask,
        "Rockmap" => LandscapeTextureKinds::RockmapMask,
        "Soil" => LandscapeTextureKinds::SoilMask,
        _ => {
            // web_sys::console::error_1(
            //     &format!("Invalid texture kind: {}", texture_kind_clone).into(),
            // );
            return;
        }
    };

    // state_guard.update_landscape_texture(
    //     landscape_component_id_clone,
    //     kind,
    //     texture,
    //     maskKind,
    //     mask,
    // );

    // drop(state_guard);

    // resume_rendering();
    // });
}

#[derive(Deserialize)]
struct TextureData {
    bytes: Vec<u8>,
    width: u32,
    height: u32,
}

async fn fetch_texture_data(
    project_id: String,
    landscape_id: String,
    texture_filename: String,
    texture_kind: String,
) -> Texture {
    // let params = to_value(&GetTextureParams {
    //     projectId: project_id,
    //     landscapeId: landscape_id,
    //     textureFilename: texture_filename,
    //     textureKind: texture_kind,
    // })
    // .unwrap();
    // let js_data = invoke("read_landscape_texture", params).await;
    // let texture_data: TextureData = js_data
    //     .into_serde()
    //     .ok()
    //     .expect("Couldn't transform texture data serde");

    let texture_data =
        read_landscape_texture(project_id, landscape_id, texture_filename, texture_kind)
            .await
            .expect("Couldn't get texture data");

    // Some((texture_data.data, texture_data.width, texture_data.height))
    Texture::new(texture_data.bytes, texture_data.width, texture_data.height)
}

async fn fetch_mask_data(
    project_id: String,
    landscape_id: String,
    mask_filename: String,
    mask_kind: String,
) -> Texture {
    // let params = to_value(&GetMaskParams {
    //     projectId: project_id,
    //     landscapeId: landscape_id,
    //     maskFilename: mask_filename,
    //     maskKind: mask_kind,
    // })
    // .unwrap();
    // let js_data = invoke("read_landscape_mask", params).await;
    let mask_data = read_landscape_mask(project_id, landscape_id, mask_filename, mask_kind)
        .await
        .expect("Couldn't get mask data");
    // let mask_data: TextureData = js_data
    //     .into_serde()
    //     .ok()
    //     .expect("Couldn't transform texture data serde");

    // Some((texture_data.data, texture_data.width, texture_data.height))
    Texture::new(mask_data.bytes, mask_data.width, mask_data.height)
}
