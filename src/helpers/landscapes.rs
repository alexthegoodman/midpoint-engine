use std::{fs::File, path::PathBuf};

use image::GenericImageView;
use serde::Serialize;
use tiff::decoder::{Decoder, DecodingResult};

use crate::landscapes::ChunkedTerrainGenerator::ChunkedTerrainGenerator;

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
            // heights[(x, y)] = height_value;

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
    upscaled: bool,
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
    // let landscape_path = landscapes_dir.join(landscapeFilename);
    let landscape_path = if upscaled {
        landscapes_dir
            .join("upscaled")
            .join("upscaled_heightmap.tiff")
    } else {
        landscapes_dir.join(landscapeFilename)
    };

    println!("landscape_path {:?}", landscape_path);

    let square_size = 1024.0 * 100.0;
    let square_height = 1858.0 * 10.0;
    let (width, height, pixel_data, rapier_heights) = read_tiff_heightmap(
        landscape_path
            .to_str()
            .expect("Couldn't form landscape string"),
        // battlefield size
        // 2048.0,
        // 2048.0,
        // 250.0,
        // literal grand canyon in meters
        square_size,
        square_size,
        square_height,
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

use image::{open, ImageBuffer, Luma};
use rayon::prelude::*;
use std::path::Path;

use rayon::prelude::*;
/// Upscales a TIFF heightmap by a given multiplier and subdivides it into tiles
///
/// # Arguments
/// * `input_path` - Path to the input TIFF file
/// * `output_dir` - Directory where the subdivided tiles will be saved
/// * `multiplier` - Factor by which to upscale (e.g., 100 will upscale by 100x then also create 100 images, each equal in size to the original image)
/// * `smoothing_factor` - 0.0 to 1.0, where 0.0 is no smoothing and 1.0 is maximum smoothing
///
/// # Returns
/// Result with the number of tiles created
use tiff::encoder::*;

pub fn upscale_tiff_heightmap(
    project_id: String,
    asset_id: String,
    input_path: &Path,
    output_dir: &Path,
    multiplier: u32,
    tiles_per_side: u32,
    smoothing_factor: f32,
) -> Result<usize, Box<dyn std::error::Error>> {
    println!(
        "Upscaling {:?} {:?} {:?} {:?}",
        input_path, output_dir, multiplier, smoothing_factor
    );

    // Open and decode input TIFF
    let mut decoder = Decoder::new(File::open(input_path)?)?;

    // Get image dimensions
    let width = decoder.dimensions()?.0;
    let height = decoder.dimensions()?.1;

    // Read the image data
    let image_data = match decoder.read_image()? {
        DecodingResult::F32(data) => data,
        _ => return Err("Unsupported TIFF format - expected F32".into()),
    };

    // Calculate dimensions
    let upscaled_width = width * multiplier;
    let upscaled_height = height * multiplier;
    let tile_width = upscaled_width / tiles_per_side; // still divide by multipler, not the square root
    let tile_height = upscaled_height / tiles_per_side;

    println!(
        "read data... {:?}x{:?} and {:?}x{:?}",
        upscaled_width, upscaled_height, tile_width, tile_height
    );

    // Create output directory if it doesn't exist
    std::fs::create_dir_all(output_dir)?;

    // Let's try both methods and compare
    let mut upscaled_nearest = vec![0.0f32; (upscaled_width * upscaled_height) as usize];
    let mut upscaled_bilinear = vec![0.0f32; (upscaled_width * upscaled_height) as usize];

    // let mut last_value = image_data[0]; // Initialize with first value
    // let mut value_changes = 0;

    // Upscale using both methods
    for y in 0..upscaled_height {
        for x in 0..upscaled_width {
            // Calculate source coordinates
            let src_x = (x as f32 / multiplier as f32).floor();
            let src_y = (y as f32 / multiplier as f32).floor();

            // Nearest Neighbor
            let nearest_x = src_x.clamp(0.0, (width - 1) as f32) as u32;
            let nearest_y = src_y.clamp(0.0, (height - 1) as f32) as u32;
            let nearest_value = image_data[(nearest_y * width + nearest_x) as usize];
            upscaled_nearest[(y * upscaled_width + x) as usize] = nearest_value;

            // Bilinear (modified to print debug info for first few pixels)
            let x_fract = (x as f32 / multiplier as f32) - src_x;
            let y_fract = (y as f32 / multiplier as f32) - src_y;

            let x1 = src_x.clamp(0.0, (width - 1) as f32) as u32;
            let y1 = src_y.clamp(0.0, (height - 1) as f32) as u32;
            let x2 = (x1 + 1).min(width - 1);
            let y2 = (y1 + 1).min(height - 1);

            let p11 = image_data[(y1 * width + x1) as usize];
            let p12 = image_data[(y2 * width + x1) as usize];
            let p21 = image_data[(y1 * width + x2) as usize];
            let p22 = image_data[(y2 * width + x2) as usize];

            let bilinear_value = bilinear_interpolate(p11, p12, p21, p22, x_fract, y_fract);
            upscaled_bilinear[(y * upscaled_width + x) as usize] = bilinear_value;
        }
    }

    // Print ranges for both methods
    let (nearest_min, nearest_max) = upscaled_nearest
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &value| {
            (min.min(value), max.max(value))
        });
    println!("Nearest neighbor range: {} to {}", nearest_min, nearest_max);

    let (bilinear_min, bilinear_max) = upscaled_bilinear
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), &value| {
            (min.min(value), max.max(value))
        });
    println!("Bilinear range: {} to {}", bilinear_min, bilinear_max);

    // After upscaling, verify the data size
    assert_eq!(
        upscaled_nearest.len(),
        (upscaled_width * upscaled_height) as usize,
        "Upscaled data size mismatch"
    );

    println!("upscaled... {:?}", upscaled_nearest.len());

    // Apply smoothing if requested
    if smoothing_factor > 0.0 {
        apply_smoothing(
            &mut upscaled_nearest,
            upscaled_width,
            upscaled_height,
            smoothing_factor,
        );
    }

    println!("smoothed...");

    // DEBUG verify upscale
    let tile_path = output_dir.join("upscaled_heightmap.tiff");
    let mut file = File::create(&tile_path).expect("Failed to create test tile file");

    let mut tiff = TiffEncoder::new(&mut file).unwrap();
    tiff.write_image::<colortype::Gray32Float>(upscaled_width, upscaled_height, &upscaled_nearest)
        .unwrap_or_else(|e| {
            panic!(
                "Failed to write test tile {}_{}: {}",
                upscaled_width, upscaled_height, e
            )
        });

    // // Subdivide and save tiles
    // let tiles: Vec<(usize, usize)> = (0..tiles_per_side as usize)
    //     .flat_map(|y| (0..tiles_per_side as usize).map(move |x| (x, y)))
    //     .collect();

    // tiles.par_iter().for_each(|&(tile_x, tile_y)| {
    //     // Create buffer of correct size: 4096x4096
    //     let mut tile_data = vec![0.0f32; (tile_width * tile_height) as usize];

    //     let start_x = tile_x as u32 * tile_width;
    //     let start_y = tile_y as u32 * tile_height;

    //     println!(
    //         "Processing tile {}_{}: starts at ({}, {}), dimensions {}x{}",
    //         tile_x, tile_y, start_x, start_y, tile_width, tile_height
    //     );

    //     // Now we'll sample the full 4096x4096 region
    //     for y in 0..tile_height {
    //         for x in 0..tile_width {
    //             let src_x = start_x + x;
    //             let src_y = start_y + y;
    //             let src_idx = src_y * upscaled_width + src_x;
    //             let dst_idx = y * tile_width + x;
    //             tile_data[dst_idx as usize] = upscaled_nearest[src_idx as usize];
    //         }
    //     }

    //     // Save the full-sized tile...
    //     let tile_path = output_dir.join(format!("tile_{}_{}.tiff", tile_x, tile_y));
    //     let mut file = File::create(&tile_path).expect("Failed to create tile file");

    //     let mut tiff = TiffEncoder::new(&mut file).unwrap();
    //     tiff.write_image::<colortype::Gray32Float>(tile_width, tile_height, &tile_data)
    //         .unwrap_or_else(|e| panic!("Failed to write tile {}_{}: {}", tile_x, tile_y, e));
    // });

    println!("upscale finsihed!");

    let landscape_data = get_landscape_pixels(
        project_id,
        asset_id,
        input_path
            .to_str()
            .expect("Couldn't make string")
            .to_string(), // doesnt matter on upscaled
        true,
    );

    let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
        output_dir
            .join("lod_1.mdbf")
            .to_str()
            .expect("Couldn't make string")
            .to_string(),
        1,
    );
    chunked_terrain_generator.generate_terrain(&landscape_data, 1.0, |_| {
        // println!("Generating Vertices Progress...");
    });

    println!("Generated LOD 1 Vertices!");

    let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
        output_dir
            .join("lod_4.mdbf")
            .to_str()
            .expect("Couldn't make string")
            .to_string(),
        4,
    );
    chunked_terrain_generator.generate_terrain(&landscape_data, 1.0, |_| {
        // println!("Generating Vertices Progress...");
    });

    println!("Generated LOD 4 Vertices!");

    let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
        output_dir
            .join("lod_8.mdbf")
            .to_str()
            .expect("Couldn't make string")
            .to_string(),
        8,
    );
    chunked_terrain_generator.generate_terrain(&landscape_data, 1.0, |_| {
        // println!("Generating Vertices Progress...");
    });

    println!("Generated LOD 8 Vertices!");

    let mut chunked_terrain_generator = ChunkedTerrainGenerator::new(
        output_dir
            .join("lod_16.mdbf")
            .to_str()
            .expect("Couldn't make string")
            .to_string(),
        16,
    );
    chunked_terrain_generator.generate_terrain(&landscape_data, 1.0, |_| {
        // println!("Generating Vertices Progress...");
    });

    println!("Generated LOD 16 Vertices!");

    Ok(0)
}

fn bilinear_interpolate(p11: f32, p12: f32, p21: f32, p22: f32, x: f32, y: f32) -> f32 {
    let p1 = p11 * (1.0 - x) + p21 * x;
    let p2 = p12 * (1.0 - x) + p22 * x;
    p1 * (1.0 - y) + p2 * y
}

fn apply_smoothing(image: &mut Vec<f32>, width: u32, height: u32, factor: f32) {
    let mut smoothed = image.clone();
    let kernel_size = 3;
    let half_kernel = kernel_size / 2;

    for y in half_kernel..height - half_kernel {
        for x in half_kernel..width - half_kernel {
            let mut sum = 0.0;
            let mut count = 0.0;

            for ky in 0..kernel_size {
                for kx in 0..kernel_size {
                    let px = x + kx - half_kernel;
                    let py = y + ky - half_kernel;

                    let weight = 1.0
                        / (1.0
                            + ((kx as f32 - half_kernel as f32).powi(2)
                                + (ky as f32 - half_kernel as f32).powi(2)));

                    let idx = (py * width + px) as usize;
                    sum += image[idx] * weight;
                    count += weight;
                }
            }

            let idx = (y * width + x) as usize;
            let original = image[idx];
            let smoothed_value = sum / count;
            smoothed[idx] = original * (1.0 - factor) + smoothed_value * factor;
        }
    }

    *image = smoothed;
}
