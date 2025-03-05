use gltf::Glb;
use gltf::Gltf;

use crate::handlers::Vertex;

struct SavedMesh {
    lod_level: u32,
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    index_count: u32,
}

struct GLBImporter {}

impl GLBImporter {
    pub fn import(bytes: &Vec<u8>) {
        let glb = Glb::from_slice(&bytes).expect("Couldn't create glb from slice");

        let mut meshes = Vec::new();

        let gltf = Gltf::from_slice(&glb.json).expect("Failed to parse GLTF JSON");

        let buffer_data = match glb.bin {
            Some(bin) => bin,
            None => panic!("No binary data found in GLB file"),
        };

        let uses_textures = gltf.textures().len().gt(&0);

        println!("Import textures count: {:?}", gltf.textures().len());

        if (gltf.textures().len() > 0) {
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

                meshes.push(SavedMesh {
                    lod_level: 0,
                    vertices,
                    indices,
                    index_count,
                });
            }
        }

        // TODO: create 5 additional LOD levels with meshopt::simplify

        // TODO: save meshes to .bin file
    }
}
