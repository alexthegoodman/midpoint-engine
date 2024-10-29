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
use crate::{core::SimpleCamera::SimpleCamera, helpers::landscapes::read_landscape_texture};
use crate::{
    core::{Grid::Grid, RendererState::RendererState},
    helpers::landscapes::read_landscape_mask,
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

pub fn handle_key_press(state: Arc<Mutex<RendererState>>, key_code: &str, is_pressed: bool) {
    let camera = get_camera();
    // let state = get_renderer_state();
    let mut state_guard = state.lock().unwrap();

    // web_sys::console::log_1(&format!("Key pressed (2): {}", key_code).into());

    match key_code {
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
    let sensitivity = 0.0000005;

    let dx = -dx * sensitivity;
    let dy = dy * sensitivity;

    camera.rotate(dx, dy);

    camera.update();
}

pub fn handle_add_model(
    state: Arc<Mutex<RendererState>>,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    // projectId: String,
    modelFilename: String,
) {
    pause_rendering();

    // let state = get_renderer_state();

    // ideally would spawn because adding model could be expensive
    // not sure how to pass wgpu items across threads

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

    let bytes = read_model(
        state_guard
            .project_selected
            .expect("Couldn't get selected project")
            .to_string(),
        modelFilename,
    )
    .expect("Couldn't get model bytes");

    state_guard.add_model(device, queue, &bytes);

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
    state: Arc<Mutex<RendererState>>,
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    projectId: String,
    landscapeAssetId: String,
    landscapeComponentId: String,
    landscapeFilename: String,
    // callback: js_sys::Function,
) {
    pause_rendering();

    // let state = get_renderer_state();

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

    state_guard.add_landscape(device, queue, &landscapeComponentId, &data);

    drop(state_guard);

    resume_rendering();

    // let this = JsValue::null();
    // let _ = callback.call0(&this);
    // });
}

pub async fn handle_add_landscape_texture(
    state: Arc<Mutex<RendererState>>,
    project_id: String,
    landscape_component_id: String,
    landscape_asset_id: String,
    texture_filename: String,
    texture_kind: String,
    mask_filename: String,
) {
    pause_rendering();

    // let state = get_renderer_state();

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
