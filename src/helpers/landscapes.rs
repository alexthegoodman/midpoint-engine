use std::{fs::File, path::PathBuf};

use image::GenericImageView;
use serde::Serialize;
use tiff::decoder::{Decoder, DecodingResult};

use super::utilities::get_common_os_dir;

pub struct LandscapePixelData {
    pub width: usize,
    pub height: usize,
    // data: Vec<u8>,
    pub pixel_data: Vec<Vec<PixelData>>,
    pub rapier_heights: na::DMatrix<f32>,
}

#[derive(Serialize)]
pub struct PixelData {
    pub height_value: f32,
    pub position: [f32; 3],
    pub tex_coords: [f32; 2],
}

#[derive(Serialize)]
pub struct TextureData {
    pub bytes: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

use nalgebra as na;

pub fn read_tiff_heightmap(
    landscape_path: &str,
    target_width: f32,
    target_length: f32,
    target_height: f32,
) -> (usize, usize, Vec<Vec<PixelData>>, na::DMatrix<f32>) {
    // Added DMatrix return
    let file = File::open(landscape_path).expect("Couldn't open tif file");
    let mut decoder = Decoder::new(file).expect("Couldn't decode tif file");

    let (width, height) = decoder.dimensions().expect("Couldn't get tif dimensions");

    let width = usize::try_from(width).unwrap();
    let height = usize::try_from(height).unwrap();

    let image = match decoder
        .read_image()
        .expect("Couldn't read image data from tif")
    {
        DecodingResult::F32(vec) => vec,
        _ => return (0, 0, Vec::new(), na::DMatrix::zeros(0, 0)),
    };

    println!("Continuing!");

    let mut pixel_data = Vec::new();
    // Create the heights matrix for the collider
    let mut heights = na::DMatrix::zeros(height, width);

    // Calculate scaling factors
    let x_scale = target_width / width as f32;
    let y_scale = target_length / height as f32;
    let z_scale = target_height;

    let min_height = *image
        .iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let max_height = *image
        .iter()
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let height_range = max_height - min_height;

    for y in 0..height {
        let mut row = Vec::new();
        for x in 0..width {
            let idx = (y * width + x) as usize;
            let normalized_height = (image[idx] - min_height) / height_range;
            let height_value = normalized_height * z_scale;

            // Set the height in the DMatrix
            heights[(y, x)] = height_value;

            let position = [
                x as f32 * x_scale - target_width / 2.0,
                height_value,
                y as f32 * y_scale - target_length / 2.0,
            ];
            let tex_coords = [x as f32 / width as f32, y as f32 / height as f32];

            row.push(PixelData {
                height_value,
                position,
                tex_coords,
            });
        }
        pixel_data.push(row);
    }

    println!("Tiff heightmap finished!");

    (width, height, pixel_data, heights)
}

pub fn get_landscape_pixels(
    // state: tauri::State<'_, AppState>,
    projectId: String,
    landscapeAssetId: String,
    landscapeFilename: String,
) -> LandscapePixelData {
    // let handle = &state.handle;
    // let config = handle.config();
    // let package_info = handle.package_info();
    // let env = handle.env();

    let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
    let landscapes_dir = sync_dir.join(format!(
        "midpoint/projects/{}/landscapes/{}/heightmaps",
        projectId, landscapeAssetId
    ));
    let landscape_path = landscapes_dir.join(landscapeFilename);

    println!("landscape_path {:?}", landscape_path);

    let (width, height, pixel_data, rapier_heights) = read_tiff_heightmap(
        landscape_path
            .to_str()
            .expect("Couldn't form landscape string"),
        2048.0,
        2048.0,
        250.0,
    );

    LandscapePixelData {
        width,
        height,
        // data: heightmap.to_vec(),
        pixel_data,
        rapier_heights,
    }
}

pub fn read_landscape_texture(
    // state: tauri::State<'_, AppState>,
    projectId: String,
    landscapeId: String,
    textureFilename: String,
    textureKind: String,
) -> Result<TextureData, String> {
    // let handle = &state.handle;
    // let config = handle.config();
    // let package_info = handle.package_info();
    // let env = handle.env();

    let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
    let texture_path = sync_dir.join(format!(
        "midpoint/projects/{}/textures/{}",
        projectId, textureFilename
    ));

    // Read the image file
    let img = image::open(&texture_path)
        .map_err(|e| format!("Failed to open landscape texture: {}", e))?;

    // Get dimensions
    let (width, height) = img.dimensions();

    // Convert to RGBA
    let rgba_img = img.to_rgba8();
    let bytes = rgba_img.into_raw();

    Ok(TextureData {
        bytes,
        width,
        height,
    })
}

pub fn read_landscape_mask(
    // state: tauri::State<'_, AppState>,
    projectId: String,
    landscapeId: String,
    maskFilename: String,
    maskKind: String,
) -> Result<TextureData, String> {
    // let handle = &state.handle;
    // let config = handle.config();
    // let package_info = handle.package_info();
    // let env = handle.env();

    let kind_slug = match maskKind.as_str() {
        "Primary" => "heightmaps",
        "Rockmap" => "rockmaps",
        "Soil" => "soils",
        _ => "",
    };

    let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
    let mask_path = sync_dir.join(format!(
        "midpoint/projects/{}/landscapes/{}/{}/{}",
        projectId, landscapeId, kind_slug, maskFilename
    ));

    println!("mask_path {:?}", mask_path);

    // Read the image file
    let img =
        image::open(&mask_path).map_err(|e| format!("Failed to open landscape mask: {}", e))?;

    // Get dimensions
    let (width, height) = img.dimensions();

    // Convert to RGBA
    let rgba_img = img.to_rgba8();
    let bytes = rgba_img.into_raw();

    Ok(TextureData {
        bytes,
        width,
        height,
    })
}
