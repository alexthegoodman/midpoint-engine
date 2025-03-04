use std::{fs, path::PathBuf};

use directories::{BaseDirs, UserDirs};
use nalgebra::Matrix4;
use uuid::Uuid;

use super::saved_data::{LevelData, SavedState};

pub fn get_common_os_dir() -> Option<PathBuf> {
    UserDirs::new().map(|user_dirs| {
        let common_os = user_dirs
            .document_dir()
            .expect("Couldn't find Documents directory")
            .join("CommonOS");
        fs::create_dir_all(&common_os)
            .ok()
            .expect("Couldn't check or create CommonOS directory");
        common_os
    })
}

pub fn get_project_dir(project_id: &str) -> Option<PathBuf> {
    let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
    let project_dir = sync_dir.join("midpoint/projects").join(project_id);

    fs::create_dir_all(&project_dir)
        .ok()
        .expect("Couldn't check or create Projects directory");

    Some(project_dir)
}

pub fn load_project_state(project_id: &str) -> Result<SavedState, Box<dyn std::error::Error>> {
    let sync_dir = get_common_os_dir().expect("Couldn't get CommonOS directory");
    let project_dir = sync_dir.join("midpoint/projects").join(project_id);
    let json_path = project_dir.join("midpoint.json");

    // Check if the project directory and json file exist
    if !project_dir.exists() {
        return Err(format!("Project directory '{}' not found", project_id).into());
    }
    if !json_path.exists() {
        return Err(format!("midpoint.json not found in project '{}'", project_id).into());
    }

    // Read and parse the JSON file
    let json_content = fs::read_to_string(json_path)?;
    let state: SavedState = serde_json::from_str(&json_content)?;

    Ok(state)
}

pub fn create_project_state(project_id: &str) -> Result<SavedState, Box<dyn std::error::Error>> {
    let project_dir = get_project_dir(project_id).expect("Couldn't get project directory");

    let empty_level = LevelData {
        id: Uuid::new_v4().to_string(),
        components: Some(Vec::new()),
    };

    let mut levels = Vec::new();
    levels.push(empty_level);

    let empty_saved_state = SavedState {
        concepts: Vec::new(),
        models: Vec::new(),
        landscapes: Some(Vec::new()),
        textures: Some(Vec::new()),
        levels: Some(levels),
        skeleton_parts: Vec::new(),
        skeletons: Vec::new(),
        motion_paths: Vec::new(),
    };

    let json = serde_json::to_string_pretty(&empty_saved_state)?;
    fs::write(project_dir.join("midpoint.json"), json)?;

    Ok(empty_saved_state)
}
