// use std::fs::File;
// use std::io::{BufReader, BufWriter, Read, Write};
// use std::path::Path;
// use std::thread;
// use std::time::{Duration, Instant};

// use crate::handlers::Vertex;
// use crate::helpers::landscapes::LandscapePixelData;

// const CHUNK_SIZE: usize = 64; // Process 64x64 vertex chunks at a time
// const SAVE_INTERVAL: Duration = Duration::from_secs(5); // Save progress every 5 seconds
// const CHUNK_SLEEP: Duration = Duration::from_millis(100); // Sleep 1ms between chunks

// #[derive(Debug)]
// pub struct TerrainChunk {
//     pub vertices: Vec<Vertex>,
//     pub indices: Vec<u32>,
//     pub chunk_x: usize,
//     pub chunk_y: usize,
// }

// pub struct ChunkedTerrainGenerator {
//     last_save: Instant,
//     processed_chunks: Vec<TerrainChunk>,
//     save_path: String,
// }

// impl ChunkedTerrainGenerator {
//     pub fn new(save_path: String) -> Self {
//         Self {
//             last_save: Instant::now(),
//             processed_chunks: Vec::new(),
//             save_path,
//         }
//     }

//     pub fn generate_terrain(
//         &mut self,
//         data: &LandscapePixelData,
//         scale: f32,
//         progress_callback: impl Fn(f32),
//     ) -> (Vec<Vertex>, Vec<u32>) {
//         // Try to load existing progress
//         if let Some((vertices, indices)) = self.load_progress() {
//             return (vertices, indices);
//         }

//         let total_chunks = ((data.height + CHUNK_SIZE - 1) / CHUNK_SIZE)
//             * ((data.width + CHUNK_SIZE - 1) / CHUNK_SIZE);
//         let mut chunks_processed = 0;

//         let mut all_vertices = Vec::with_capacity(data.width * data.height);
//         let mut all_indices = Vec::new();
//         let mut vertex_offset = 0;

//         // Process terrain in chunks
//         for chunk_y in (0..data.height).step_by(CHUNK_SIZE) {
//             for chunk_x in (0..data.width).step_by(CHUNK_SIZE) {
//                 let chunk = self.process_chunk(data, chunk_x, chunk_y, vertex_offset);

//                 // Add vertices and adjusted indices to main collections
//                 all_vertices.extend(chunk.vertices.clone());
//                 all_indices.extend(chunk.indices.clone());

//                 vertex_offset = all_vertices.len() as u32;
//                 self.processed_chunks.push(chunk);

//                 chunks_processed += 1;
//                 progress_callback(chunks_processed as f32 / total_chunks as f32);

//                 // Periodically save progress
//                 if self.last_save.elapsed() >= SAVE_INTERVAL {
//                     println!(
//                         "Save generation progress {:?} {:?}",
//                         all_vertices.len(),
//                         data.height * data.width
//                     );
//                     self.save_progress(&all_vertices, &all_indices);
//                     self.last_save = Instant::now();
//                     thread::sleep(Duration::from_millis(500));
//                 }

//                 // Small sleep between chunks to prevent freezing
//                 thread::sleep(CHUNK_SLEEP);
//             }
//         }

//         // Final save
//         self.save_progress(&all_vertices, &all_indices);

//         (all_vertices, all_indices)
//     }

//     fn process_chunk(
//         &self,
//         data: &LandscapePixelData,
//         chunk_x: usize,
//         chunk_y: usize,
//         vertex_offset: u32,
//     ) -> TerrainChunk {
//         let mut vertices = Vec::new();
//         let mut indices = Vec::new();

//         let chunk_width = CHUNK_SIZE.min(data.width - chunk_x);
//         let chunk_height = CHUNK_SIZE.min(data.height - chunk_y);

//         // Generate vertices for this chunk
//         for y in 0..chunk_height {
//             for x in 0..chunk_width {
//                 let global_x = chunk_x + x;
//                 let global_y = chunk_y + y;

//                 vertices.push(Vertex {
//                     position: data.pixel_data[global_y][global_x].position,
//                     normal: [0.0, 0.0, 0.0],
//                     tex_coords: data.pixel_data[global_y][global_x].tex_coords,
//                     color: [1.0, 1.0, 1.0],
//                 });
//             }
//         }

//         // Generate indices for this chunk
//         for y in 0..(chunk_height - 1) {
//             for x in 0..(chunk_width - 1) {
//                 let top_left = (y * chunk_width + x) as u32 + vertex_offset;
//                 let top_right = top_left + 1;
//                 let bottom_left = ((y + 1) * chunk_width + x) as u32 + vertex_offset;
//                 let bottom_right = bottom_left + 1;

//                 // Main triangles
//                 indices.extend_from_slice(&[top_left, bottom_left, top_right]);
//                 indices.extend_from_slice(&[top_right, bottom_left, bottom_right]);

//                 // Additional connections within chunk
//                 if x < chunk_width - 2 {
//                     indices.extend_from_slice(&[top_right, bottom_right, top_right + 1]);
//                     indices.extend_from_slice(&[bottom_right, bottom_right + 1, top_right + 1]);
//                 }

//                 if y < chunk_height - 2 {
//                     indices.extend_from_slice(&[
//                         bottom_left,
//                         bottom_left + chunk_width as u32,
//                         bottom_right,
//                     ]);
//                     indices.extend_from_slice(&[
//                         bottom_right,
//                         bottom_left + chunk_width as u32,
//                         bottom_right + chunk_width as u32,
//                     ]);
//                 }
//             }
//         }

//         TerrainChunk {
//             vertices,
//             indices,
//             chunk_x: chunk_x / CHUNK_SIZE,
//             chunk_y: chunk_y / CHUNK_SIZE,
//         }
//     }

//     fn save_progress(&self, vertices: &[Vertex], indices: &[u32]) {
//         if let Ok(file) = File::create(&self.save_path) {
//             let mut writer = BufWriter::new(file);

//             // Write vertices
//             let vertex_bytes = unsafe {
//                 std::slice::from_raw_parts(
//                     vertices.as_ptr() as *const u8,
//                     vertices.len() * std::mem::size_of::<Vertex>(),
//                 )
//             };
//             writer
//                 .write_all(&(vertices.len() as u64).to_le_bytes())
//                 .unwrap();
//             writer.write_all(vertex_bytes).unwrap();

//             // Write indices
//             let index_bytes = unsafe {
//                 std::slice::from_raw_parts(
//                     indices.as_ptr() as *const u8,
//                     indices.len() * std::mem::size_of::<u32>(),
//                 )
//             };
//             writer
//                 .write_all(&(indices.len() as u64).to_le_bytes())
//                 .unwrap();
//             writer.write_all(index_bytes).unwrap();
//         }
//     }

//     fn load_progress(&self) -> Option<(Vec<Vertex>, Vec<u32>)> {
//         if let Ok(file) = File::open(&self.save_path) {
//             let mut reader = BufReader::new(file);

//             // Read vertex count
//             let mut count_bytes = [0u8; 8];
//             reader.read_exact(&mut count_bytes).ok()?;
//             let vertex_count = u64::from_le_bytes(count_bytes) as usize;

//             // Read vertices
//             let mut vertices = Vec::with_capacity(vertex_count);
//             unsafe {
//                 vertices.set_len(vertex_count);
//                 let vertex_slice = std::slice::from_raw_parts_mut(
//                     vertices.as_mut_ptr() as *mut u8,
//                     vertex_count * std::mem::size_of::<Vertex>(),
//                 );
//                 reader.read_exact(vertex_slice).ok()?;
//             }

//             // Read index count
//             reader.read_exact(&mut count_bytes).ok()?;
//             let index_count = u64::from_le_bytes(count_bytes) as usize;

//             // Read indices
//             let mut indices = Vec::with_capacity(index_count);
//             unsafe {
//                 indices.set_len(index_count);
//                 let index_slice = std::slice::from_raw_parts_mut(
//                     indices.as_mut_ptr() as *mut u8,
//                     index_count * std::mem::size_of::<u32>(),
//                 );
//                 reader.read_exact(index_slice).ok()?;
//             }

//             Some((vertices, indices))
//         } else {
//             None
//         }
//     }
// }

use std::fs::{File, OpenOptions};
use std::io::{BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use crate::handlers::Vertex;
use crate::helpers::landscapes::LandscapePixelData;

const CHUNK_SIZE: usize = 512; // Process 512x512 vertex chunks at a time
const SAVE_INTERVAL: Duration = Duration::from_secs(5);
const CHUNK_SLEEP: Duration = Duration::from_millis(200);

#[derive(Debug)]
pub struct TerrainChunk {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub chunk_x: usize,
    pub chunk_y: usize,
}

pub struct ChunkedTerrainGenerator {
    last_save: Instant,
    save_path: String,
    vertex_count: u64,
    index_count: u64,
    stride: usize,
}

impl ChunkedTerrainGenerator {
    pub fn new(save_path: String, stride: usize) -> Self {
        // Initialize or clear the file
        // if let Ok(file) = OpenOptions::new()
        //     .write(true)
        //     .create(true)
        //     .truncate(true)
        //     .open(&save_path)
        // {
        //     let mut writer = BufWriter::new(file);
        //     writer.write_all(&0u64.to_le_bytes()).unwrap(); // vertex count
        //     writer.write_all(&0u64.to_le_bytes()).unwrap(); // index count
        //     writer.write_all(&(stride as u64).to_le_bytes()).unwrap(); // stride value
        // }

        Self {
            last_save: Instant::now(),
            save_path,
            vertex_count: 0,
            index_count: 0,
            stride,
        }
    }

    pub fn generate_terrain(
        &mut self,
        data: &LandscapePixelData,
        scale: f32,
        progress_callback: impl Fn(f32),
    ) -> (usize, usize) {
        let strided_width = (data.width + self.stride - 1) / self.stride;
        let strided_height = (data.height + self.stride - 1) / self.stride;

        let total_chunks = ((strided_height + CHUNK_SIZE - 1) / CHUNK_SIZE)
            * ((strided_width + CHUNK_SIZE - 1) / CHUNK_SIZE);
        let mut chunks_processed = 0;
        let mut vertex_offset = 0;

        // Process terrain in chunks
        for chunk_y in (0..strided_height).step_by(CHUNK_SIZE) {
            for chunk_x in (0..strided_width).step_by(CHUNK_SIZE) {
                let chunk = self.process_chunk(data, chunk_x, chunk_y, vertex_offset);
                self.append_chunk(&chunk);

                vertex_offset = self.vertex_count as u32;
                chunks_processed += 1;
                progress_callback(chunks_processed as f32 / total_chunks as f32);

                thread::sleep(CHUNK_SLEEP);
            }
        }

        self.add_file_footer();
        (self.vertex_count as usize, self.index_count as usize)
    }

    fn process_chunk(
        &self,
        data: &LandscapePixelData,
        chunk_x: usize,
        chunk_y: usize,
        vertex_offset: u32,
    ) -> TerrainChunk {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let strided_width = (data.width + self.stride - 1) / self.stride;
        let strided_height = (data.height + self.stride - 1) / self.stride;

        let chunk_width = CHUNK_SIZE.min(strided_width - chunk_x);
        let chunk_height = CHUNK_SIZE.min(strided_height - chunk_y);

        // Generate vertices for this chunk with stride
        for y in 0..chunk_height {
            for x in 0..chunk_width {
                let global_x = (chunk_x + x) * self.stride;
                let global_y = (chunk_y + y) * self.stride;

                // Skip if we're out of bounds due to stride
                if global_x >= data.width || global_y >= data.height {
                    continue;
                }

                vertices.push(Vertex {
                    position: data.pixel_data[global_y][global_x].position,
                    normal: [0.0, 0.0, 0.0],
                    tex_coords: [
                        (global_x as f32) / (data.width as f32),
                        (global_y as f32) / (data.height as f32),
                    ],
                    color: [1.0, 1.0, 1.0],
                });
            }
        }

        // Generate indices for this chunk
        for y in 0..(chunk_height - 1) {
            for x in 0..(chunk_width - 1) {
                let top_left = (y * chunk_width + x) as u32 + vertex_offset;
                let top_right = top_left + 1;
                let bottom_left = ((y + 1) * chunk_width + x) as u32 + vertex_offset;
                let bottom_right = bottom_left + 1;

                // Main triangles
                indices.extend_from_slice(&[top_left, bottom_left, top_right]);
                indices.extend_from_slice(&[top_right, bottom_left, bottom_right]);
            }
        }

        TerrainChunk {
            vertices,
            indices,
            chunk_x: chunk_x / CHUNK_SIZE,
            chunk_y: chunk_y / CHUNK_SIZE,
        }
    }

    fn append_chunk(&mut self, chunk: &TerrainChunk) {
        if let Ok(file) = OpenOptions::new()
            .write(true)
            .append(true)
            .create(true)
            .open(&self.save_path)
        {
            println!("append_chunk");
            let mut writer = BufWriter::new(file);

            let vertex_bytes = unsafe {
                std::slice::from_raw_parts(
                    chunk.vertices.as_ptr() as *const u8,
                    chunk.vertices.len() * std::mem::size_of::<Vertex>(),
                )
            };
            writer.write_all(vertex_bytes).unwrap();

            let index_bytes = unsafe {
                std::slice::from_raw_parts(
                    chunk.indices.as_ptr() as *const u8,
                    chunk.indices.len() * std::mem::size_of::<u32>(),
                )
            };
            writer.write_all(index_bytes).unwrap();

            self.vertex_count += chunk.vertices.len() as u64;
            self.index_count += chunk.indices.len() as u64;

            thread::sleep(Duration::from_millis(100));
        }
    }

    // fn update_file_header(&self) {
    //     if let Ok(mut file) = OpenOptions::new().write(true).open(&self.save_path) {
    //         file.seek(SeekFrom::Start(0)).unwrap();
    //         file.write_all(&self.vertex_count.to_le_bytes()).unwrap();
    //         file.write_all(&self.index_count.to_le_bytes()).unwrap();
    //         file.write_all(&(self.stride as u64).to_le_bytes()).unwrap();
    //     }
    // }

    fn add_file_footer(&self) {
        if let Ok(mut file) = OpenOptions::new()
            .append(true)
            .create(true)
            .truncate(false) // Important! Don't truncate the file
            .open(&self.save_path)
        {
            // file.seek(SeekFrom::Start(0)).unwrap();
            file.write_all(&self.vertex_count.to_le_bytes()).unwrap();
            file.write_all(&self.index_count.to_le_bytes()).unwrap();
            file.write_all(&(self.stride as u64).to_le_bytes()).unwrap();
        }
    }

    pub fn load_terrain(&self) -> Option<(Vec<Vertex>, Vec<u32>, usize)> {
        // Normalize the path to use platform-specific separators
        let normalized_path = std::path::Path::new(&self.save_path).canonicalize().ok()?;

        println!("Attempting to load terrain from: {:?}", normalized_path);

        if let Ok(mut file) = File::open(&normalized_path) {
            // Get file size
            let metadata = file.metadata().ok()?;
            println!("File size: {} bytes", metadata.len());

            let mut reader = BufReader::new(file);

            // // Try reading just the first count to see if we're reading correctly
            // let mut count_bytes = [0u8; 8];
            // match reader.read_exact(&mut count_bytes) {
            //     Ok(_) => println!("First 8 bytes: {:?}", count_bytes),
            //     Err(e) => println!("Error reading first 8 bytes: {}", e),
            // }

            // // Reset reader position
            // reader.seek(std::io::SeekFrom::Start(0)).ok()?;

            // Seek to 24 bytes from end (3 u64s)
            reader.seek(SeekFrom::End(-24)).ok()?;

            // Read counts from footer
            let mut count_bytes = [0u8; 8];
            reader.read_exact(&mut count_bytes).ok()?;
            let vertex_count = u64::from_le_bytes(count_bytes) as usize;

            reader.read_exact(&mut count_bytes).ok()?;
            let index_count = u64::from_le_bytes(count_bytes) as usize;

            reader.read_exact(&mut count_bytes).ok()?;
            let stride = u64::from_le_bytes(count_bytes) as usize;

            // Seek back to start for data
            reader.seek(SeekFrom::Start(0)).ok()?;

            let mut vertices = Vec::with_capacity(vertex_count);
            unsafe {
                vertices.set_len(vertex_count);
                let vertex_slice = std::slice::from_raw_parts_mut(
                    vertices.as_mut_ptr() as *mut u8,
                    vertex_count * std::mem::size_of::<Vertex>(),
                );
                if let Err(e) = reader.read_exact(vertex_slice) {
                    println!("Error reading vertices: {}", e);
                    return None;
                }
            }

            let mut indices = Vec::with_capacity(index_count);
            unsafe {
                indices.set_len(index_count);
                let index_slice = std::slice::from_raw_parts_mut(
                    indices.as_mut_ptr() as *mut u8,
                    index_count * std::mem::size_of::<u32>(),
                );
                if let Err(e) = reader.read_exact(index_slice) {
                    println!("Error reading indices: {}", e);
                    return None;
                }
            }

            println!(
                "Successfully loaded {} vertices and {} indices",
                vertices.len(),
                indices.len()
            );
            Some((vertices, indices, stride))
        } else {
            println!("Could not open file: {}", &self.save_path);
            None
        }
    }
}
