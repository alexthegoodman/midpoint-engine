use gltf::Glb;
use gltf::Gltf;
use meshopt::{self, SimplifyOptions, VertexDataAdapter};
use std::fs::File;
use std::io::Seek;
use std::io::{BufWriter, Write};
use std::mem::size_of;
use std::path::Path;

use crate::handlers::Vertex;

// Define file header and chunk identifiers
const FILE_MAGIC: u32 = 0x444F4C4D; // "MLOD" in ASCII, reversed for endianness
const HEADER_VERSION: u32 = 1;
const MESH_CHUNK_ID: u32 = 0x4853454D; // "MESH" in ASCII, reversed

#[repr(C, packed)]
struct FileHeader {
    magic: u32,      // Magic number for file identification
    version: u32,    // Version of the file format
    mesh_count: u32, // Total number of meshes
    lod_levels: u32, // Number of LOD levels per mesh
}

#[repr(C, packed)]
struct MeshHeader {
    chunk_id: u32,     // Chunk identifier
    chunk_size: u64,   // Size of the chunk data in bytes
    vertex_count: u32, // Number of vertices
    lod_count: u32,    // Number of LOD levels
}

#[repr(C, packed)]
struct LODHeader {
    lod_level: u32,   // LOD level (0 = highest detail)
    index_count: u32, // Number of indices
    offset: u64,      // Offset into the index buffer
}

struct SavedMesh {
    lod_level: u32,
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    index_count: u32,
}

struct GLBImporter {}

impl GLBImporter {
    pub fn import(bytes: &Vec<u8>, output_path: &str) {
        let glb = Glb::from_slice(&bytes).expect("Couldn't create glb from slice");

        let mut meshes = Vec::new();

        let gltf = Gltf::from_slice(&glb.json).expect("Failed to parse GLTF JSON");

        let buffer_data = match glb.bin {
            Some(bin) => bin,
            None => panic!("No binary data found in GLB file"),
        };

        let uses_textures = gltf.textures().len().gt(&0);

        println!("Import textures count: {:?}", gltf.textures().len());

        if gltf.textures().len() > 0 {
            for texture in gltf.textures() {
                match texture.source().source() {
                    gltf::image::Source::View { view, mime_type: _ } => {
                        let img_data = &buffer_data[view.offset()..view.offset() + view.length()];
                        let img = image::load_from_memory(img_data).unwrap().to_rgba8();
                        let (width, height) = img.dimensions();

                        // TODO: save texture to png file
                    }
                    gltf::image::Source::Uri { uri, mime_type: _ } => {
                        panic!(
                            "External URI image sources are not yet supported in glb files: {}",
                            uri
                        );
                    }
                }
            }
        }

        for mesh in gltf.meshes() {
            for primitive in mesh.primitives() {
                let reader = primitive.reader(|buffer| Some(&buffer_data));

                let positions = reader
                    .read_positions()
                    .expect("Positions not existing in glb");
                let colors = reader
                    .read_colors(0)
                    .map(|v| v.into_rgb_f32().collect())
                    .unwrap_or_else(|| vec![[1.0, 1.0, 1.0]; positions.len()]);
                let normals: Vec<[f32; 3]> = reader
                    .read_normals()
                    .map(|iter| iter.collect())
                    .unwrap_or_else(|| vec![[0.0, 0.0, 1.0]; positions.len()]);
                let tex_coords: Vec<[f32; 2]> = reader
                    .read_tex_coords(0)
                    .map(|v| v.into_f32().collect())
                    .unwrap_or_else(|| vec![[0.0, 0.0]; positions.len()]);

                let vertices: Vec<Vertex> = positions
                    .zip(normals.iter())
                    .zip(tex_coords.iter())
                    .zip(colors.iter())
                    .map(|(((p, n), t), c)| Vertex {
                        position: p,
                        normal: *n,
                        tex_coords: *t,
                        color: *c,
                    })
                    .collect();

                let indices: Vec<u32> = reader
                    .read_indices()
                    .map(|iter| iter.into_u32().collect())
                    .unwrap_or_default();

                println!("Import Model vertices: {:?}", vertices.len());
                println!("Import Model indices: {:?}", indices.len());

                let index_count = indices.len() as u32;

                // Add the original mesh as LOD level 0
                meshes.push(SavedMesh {
                    lod_level: 0,
                    vertices: vertices.clone(),
                    indices: indices.clone(),
                    index_count,
                });

                // Generate additional LOD levels
                Self::generate_lods(&vertices, &indices, &mut meshes);
            }
        }

        // Save meshes to .bin file
        Self::save_to_binary(&meshes, output_path).expect("Couldn't save meshes to binary file");
    }

    fn generate_lods(vertices: &Vec<Vertex>, indices: &Vec<u32>, meshes: &mut Vec<SavedMesh>) {
        // Define LOD reduction factors - each level reduces triangle count by this percentage
        let lod_ratios = [0.5, 0.25, 0.125, 0.0625, 0.03125]; // 50%, 25%, 12.5%, 6.25%, 3.125%

        // Skip if we don't have a valid mesh
        if indices.len() < 3 || vertices.is_empty() {
            return;
        }

        // Prepare vertex data for meshopt
        // We need to get the vertex positions as a byte slice
        let vertex_positions_raw: Vec<u8> = vertices
            .iter()
            .flat_map(|v| {
                let pos = v.position;
                // Convert each position to raw bytes
                let x_bytes = pos[0].to_ne_bytes();
                let y_bytes = pos[1].to_ne_bytes();
                let z_bytes = pos[2].to_ne_bytes();
                // Concatenate all bytes
                [&x_bytes[..], &y_bytes[..], &z_bytes[..]].concat()
            })
            .collect();

        // Create vertex data adapter for meshopt
        let vertex_stride = size_of::<[f32; 3]>();
        let vertex_data_adapter = VertexDataAdapter::new(
            &vertex_positions_raw,
            vertex_stride,
            0, // Offset is 0 since we're passing just the positions
        )
        .unwrap();

        // Create each LOD level
        for (i, ratio) in lod_ratios.iter().enumerate() {
            let lod_level = i as u32 + 1; // LOD levels start from 1 (0 is original)

            // Calculate target triangle count
            let original_triangle_count = indices.len() / 3;
            let target_triangle_count = (original_triangle_count as f32 * ratio) as usize;

            // Skip if target count is too small
            if target_triangle_count < 1 {
                continue;
            }

            // Configure simplification options
            // let options = SimplifyOptions {
            //     lock_border: false, // Don't lock border vertices
            //     aggressive: false,  // Use less aggressive simplification
            // };
            let options = SimplifyOptions::empty();

            // Keep track of the simplification error
            let mut result_error = 0.0;

            // Simplify the mesh
            let simplified_indices = meshopt::simplify(
                &indices,
                &vertex_data_adapter,
                target_triangle_count * 3, // Target index count
                0.01,                      // Target error threshold (adjust as needed)
                options,
                Some(&mut result_error),
            );

            println!(
                "LOD {} - Original indices: {}, Simplified indices: {}, Error: {}",
                lod_level,
                indices.len(),
                simplified_indices.len(),
                result_error
            );

            let index_count = simplified_indices.len() as u32;

            // Store the simplified mesh
            meshes.push(SavedMesh {
                lod_level,
                vertices: vertices.clone(), // Keep the original vertices
                indices: simplified_indices,
                index_count,
            });
        }
    }

    fn save_to_binary(meshes: &Vec<SavedMesh>, output_path: &str) -> Result<(), std::io::Error> {
        // Open file for writing
        let file = File::create(Path::new(output_path))?;
        let mut writer = BufWriter::new(file);

        // If we have no meshes, create an empty file with just a header
        if meshes.is_empty() {
            let header = FileHeader {
                magic: FILE_MAGIC,
                version: HEADER_VERSION,
                mesh_count: 0,
                lod_levels: 0,
            };

            // Write file header
            writer.write_all(unsafe {
                std::slice::from_raw_parts(
                    &header as *const FileHeader as *const u8,
                    std::mem::size_of::<FileHeader>(),
                )
            })?;

            return Ok(());
        }

        // Group meshes by their original mesh (grouping LODs together)
        let mut mesh_groups: Vec<Vec<&SavedMesh>> = Vec::new();

        // Find all unique mesh groups by looking at LOD level 0
        let base_meshes: Vec<&SavedMesh> = meshes.iter().filter(|m| m.lod_level == 0).collect();

        // For each base mesh, find all its LOD levels
        for base_mesh in base_meshes {
            let vertex_count = base_mesh.vertices.len();
            let mut group = Vec::new();

            // Find all LOD levels for this mesh
            for mesh in meshes.iter() {
                if mesh.vertices.len() == vertex_count {
                    group.push(mesh);
                }
            }

            // Sort by LOD level (highest detail first)
            group.sort_by_key(|m| m.lod_level);
            mesh_groups.push(group);
        }

        // Create and write file header
        let header = FileHeader {
            magic: FILE_MAGIC,
            version: HEADER_VERSION,
            mesh_count: mesh_groups.len() as u32,
            lod_levels: if !mesh_groups.is_empty() {
                mesh_groups[0].len() as u32
            } else {
                0
            },
        };

        // Write file header
        writer.write_all(unsafe {
            std::slice::from_raw_parts(
                &header as *const FileHeader as *const u8,
                std::mem::size_of::<FileHeader>(),
            )
        })?;

        // Write each mesh group
        for mesh_group in &mesh_groups {
            if mesh_group.is_empty() {
                continue;
            }

            let base_mesh = &mesh_group[0]; // LOD level 0

            // Create and write mesh header
            let mesh_header = MeshHeader {
                chunk_id: MESH_CHUNK_ID,
                chunk_size: 0, // We'll update this later
                vertex_count: base_mesh.vertices.len() as u32,
                lod_count: mesh_group.len() as u32,
            };

            // Remember position for updating chunk size later
            let mesh_header_pos = writer.stream_position()?;

            // Write mesh header
            writer.write_all(unsafe {
                std::slice::from_raw_parts(
                    &mesh_header as *const MeshHeader as *const u8,
                    std::mem::size_of::<MeshHeader>(),
                )
            })?;

            // Write LOD headers - we'll need to update offsets later
            let lod_headers_start_pos = writer.stream_position()?;
            let mut lod_headers = Vec::new();

            for mesh in mesh_group {
                lod_headers.push(LODHeader {
                    lod_level: mesh.lod_level,
                    index_count: mesh.index_count,
                    offset: 0, // We'll update this later
                });
            }

            // Write placeholder LOD headers
            for lod_header in &lod_headers {
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        lod_header as *const LODHeader as *const u8,
                        std::mem::size_of::<LODHeader>(),
                    )
                })?;
            }

            // Write vertices once (shared across all LODs)
            let vertices_start_pos = writer.stream_position()?;
            for vertex in &base_mesh.vertices {
                // Write position
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        &vertex.position as *const [f32; 3] as *const u8,
                        std::mem::size_of::<[f32; 3]>(),
                    )
                })?;

                // Write normal
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        &vertex.normal as *const [f32; 3] as *const u8,
                        std::mem::size_of::<[f32; 3]>(),
                    )
                })?;

                // Write texture coordinates
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        &vertex.tex_coords as *const [f32; 2] as *const u8,
                        std::mem::size_of::<[f32; 2]>(),
                    )
                })?;

                // Write color
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        &vertex.color as *const [f32; 3] as *const u8,
                        std::mem::size_of::<[f32; 3]>(),
                    )
                })?;
            }

            // Remember current position for indices start
            let indices_start_pos = writer.stream_position()?;

            // Write indices for each LOD level and update offsets
            let mut current_offset = 0;
            for (i, mesh) in mesh_group.iter().enumerate() {
                // Update LOD header with correct offset
                lod_headers[i].offset = current_offset;

                // Write indices
                for index in &mesh.indices {
                    writer.write_all(&index.to_ne_bytes())?;
                }

                current_offset += mesh.indices.len() as u64 * std::mem::size_of::<u32>() as u64;
            }

            // Calculate chunk size
            let chunk_end_pos = writer.stream_position()?;
            let chunk_size =
                chunk_end_pos - mesh_header_pos - std::mem::size_of::<MeshHeader>() as u64;

            // Go back and update mesh header with correct chunk size
            writer.seek(std::io::SeekFrom::Start(mesh_header_pos))?;
            let mut updated_mesh_header = mesh_header;
            updated_mesh_header.chunk_size = chunk_size;

            writer.write_all(unsafe {
                std::slice::from_raw_parts(
                    &updated_mesh_header as *const MeshHeader as *const u8,
                    std::mem::size_of::<MeshHeader>(),
                )
            })?;

            // Go back and update LOD headers with correct offsets
            writer.seek(std::io::SeekFrom::Start(lod_headers_start_pos))?;
            for lod_header in &lod_headers {
                writer.write_all(unsafe {
                    std::slice::from_raw_parts(
                        lod_header as *const LODHeader as *const u8,
                        std::mem::size_of::<LODHeader>(),
                    )
                })?;
            }

            // Return to end of file for next mesh
            writer.seek(std::io::SeekFrom::Start(chunk_end_pos))?;
        }

        writer.flush()?;
        println!("Successfully wrote mesh data to: {}", output_path);

        Ok(())
    }
}
