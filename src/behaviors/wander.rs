use std::{
    sync::{Arc, Mutex, MutexGuard},
    time::Instant,
};

use nalgebra::{vector, ComplexField, Vector3};
use nalgebra_glm::{Quat, Vec3};
use rand::Rng;
use rapier3d::{parry::query::ShapeCastOptions, prelude::*};
use uuid::Uuid;

use crate::core::{RendererState::RendererState, Transform::Transform};

pub struct WanderBehavior {
    // Configuration
    pub radius: f32,       // How far to wander from current position
    pub jitter: f32,       // Random variation in movement
    pub speed: f32,        // Movement speed
    pub min_distance: f32, // Minimum distance from obstacles
    pub max_slope: f32,    // Maximum traversable slope angle

    // Internal state
    target_position: Vector3<f32>,
    last_update: Instant,
}

impl WanderBehavior {
    pub fn new(radius: f32, speed: f32) -> Self {
        WanderBehavior {
            radius,
            speed,
            jitter: 0.5,
            min_distance: 1.0,
            max_slope: 45.0,
            target_position: Vec3::identity(),
            last_update: Instant::now(),
        }
    }

    pub fn update(
        &mut self,
        renderer_state: Arc<Mutex<RendererState>>,
        rigid_body: &mut RigidBody,
        rigid_body_handle: RigidBodyHandle,
        collider: &Collider,
        transform: &mut Transform,
        dt: f32,
    ) {
        let mut renderer_state = renderer_state.lock().unwrap();
        let current_pos = transform.position;

        // Update target periodically or when reached
        if self.last_update.elapsed().as_secs_f32() > 2.0
            || current_pos.metric_distance(&self.target_position) < 0.5
        {
            self.choose_new_target(
                renderer_state.rigid_body_set.clone(),
                renderer_state.collider_set.clone(),
                renderer_state.query_pipeline.clone(),
                current_pos,
            );
            self.last_update = Instant::now();
        }

        // Calculate movement direction
        let direction = (self.target_position - current_pos).normalize();

        // Forward obstacle detection using shape cast
        let shape = collider.shape().clone();
        let shape_pos = Isometry::new(
            vector![current_pos.x, current_pos.y, current_pos.z],
            vector![0.0, 0.0, 0.0],
        );
        let shape_vel = vector![
            direction.x * self.min_distance,
            direction.y * self.min_distance,
            direction.z * self.min_distance
        ];

        let options = ShapeCastOptions {
            max_time_of_impact: 1.0,
            target_distance: 0.0,
            stop_at_penetration: false,
            compute_impact_geometry_on_penetration: true,
        };

        if renderer_state
            .query_pipeline
            .cast_shape(
                &renderer_state.rigid_body_set,
                &renderer_state.collider_set,
                &shape_pos,
                &shape_vel,
                shape,
                options,
                QueryFilter::default().exclude_rigid_body(rigid_body_handle),
            )
            .is_some()
        {
            self.choose_new_target(
                renderer_state.rigid_body_set.clone(),
                renderer_state.collider_set.clone(),
                renderer_state.query_pipeline.clone(),
                current_pos,
            );
            return;
        }

        // Ground detection using shape cast
        let ground_shape = Cuboid::new(vector![0.1, 0.1, 0.1]);
        let ground_pos = Isometry::new(
            vector![current_pos.x, current_pos.y + 1.0, current_pos.z],
            vector![0.0, 0.0, 0.0],
        );
        let ground_vel = vector![0.0, -2.0, 0.0];

        let ground_options = ShapeCastOptions {
            max_time_of_impact: 2.0,
            target_distance: 0.0,
            stop_at_penetration: false,
            compute_impact_geometry_on_penetration: true,
        };

        if let Some((_, hit)) = renderer_state.query_pipeline.cast_shape(
            &renderer_state.rigid_body_set,
            &renderer_state.collider_set,
            &ground_pos,
            &ground_vel,
            &ground_shape,
            ground_options,
            QueryFilter::default().exclude_rigid_body(rigid_body_handle),
        ) {
            // if let Some(normal) = hit.normal1 {
            let normal = hit.normal1;
            let slope_angle = normal.y.acos().to_degrees();

            if slope_angle > self.max_slope {
                self.choose_new_target(
                    renderer_state.rigid_body_set.clone(),
                    renderer_state.collider_set.clone(),
                    renderer_state.query_pipeline.clone(),
                    current_pos,
                );
                return;
            }
            // }
        }

        // Apply movement
        let movement = direction * self.speed * dt;
        rigid_body.set_linvel(vector![movement.x, movement.y, movement.z], true);
    }

    fn choose_new_target(
        &mut self,
        rigid_body_set: RigidBodySet,
        collider_set: ColliderSet,
        physics: QueryPipeline,
        current_pos: Vec3,
    ) {
        let mut rng = rand::thread_rng();

        for _ in 0..10 {
            let angle = rng.gen_range(0.0..std::f32::consts::TAU);
            let distance = rng.gen_range(0.0..self.radius);

            let offset = Vec3::new(angle.cos() * distance, 0.0, angle.sin() * distance);

            let potential_target = current_pos + offset;

            // Ground detection
            let ground_shape = Cuboid::new(vector![0.1, 0.1, 0.1]);
            let ground_pos = Isometry::new(
                vector![
                    potential_target.x,
                    potential_target.y + 5.0,
                    potential_target.z
                ],
                vector![0.0, 0.0, 0.0],
            );
            let ground_vel = vector![0.0, -10.0, 0.0];

            let ground_options = ShapeCastOptions {
                max_time_of_impact: 10.0,
                target_distance: 0.0,
                stop_at_penetration: false,
                compute_impact_geometry_on_penetration: true,
            };

            if let Some((_, hit)) = physics.cast_shape(
                &rigid_body_set,
                &collider_set,
                &ground_pos,
                &ground_vel,
                &ground_shape,
                ground_options,
                QueryFilter::default(),
            ) {
                let ground_height = ground_pos.translation.y - (hit.time_of_impact * ground_vel.y);
                let ground_pos = Vec3::new(potential_target.x, ground_height, potential_target.z);

                // Validate position with a shape cast
                let check_shape = Ball::new(0.5);
                let check_pos = Isometry::new(
                    vector![ground_pos.x, ground_pos.y + 1.0, ground_pos.z],
                    vector![0.0, 0.0, 0.0],
                );

                // Cast slightly downward to ensure we're grounded
                let check_vel = vector![0.0, -0.1, 0.0];

                let check_options = ShapeCastOptions {
                    max_time_of_impact: 1.0,
                    target_distance: 0.0,
                    stop_at_penetration: true,
                    compute_impact_geometry_on_penetration: false,
                };

                if physics
                    .cast_shape(
                        &rigid_body_set,
                        &collider_set,
                        &check_pos,
                        &check_vel,
                        &check_shape,
                        check_options,
                        QueryFilter::default(),
                    )
                    .is_none()
                {
                    self.target_position = ground_pos;
                    return;
                }
            }
        }

        self.target_position = current_pos;
    }
}
