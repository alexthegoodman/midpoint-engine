use std::{fs, path::PathBuf};

use directories::{BaseDirs, UserDirs};

use super::saved_data::SavedState;

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
