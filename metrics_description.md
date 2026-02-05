# Navigation Metrics Description

## Aggregated Metrics (Summary)

| Metric | Description | Good Values |
|--------|-------------|-------------|
| **Success Rate** | Fraction of runs reaching the goal | Close to 1.0 |
| **Median Path Length Ratio** | Actual path / straight-line distance (successful runs only) | 1.0–2.0 |
| **Median Time to Goal** | Time to reach goal in seconds (successful runs only) | Lower is better |
| **Avg Stuck Events** | Times robot got stuck (stayed in 0.5m radius for >8s) | 0 is ideal |
| **Avg Collision Events** | Number of collisions with obstacles | Must be 0 |
| **Avg Min Obstacle Distance** | Closest approach to any obstacle (meters) | > 0.3m |
| **Avg Mean Obstacle Distance** | Average clearance from obstacles (meters) | > 1.0m |
| **Avg Warning Zone Steps** | Steps spent within 0.3–0.6m of obstacles | Lower is better |
| **Avg Danger Zone Steps** | Steps spent within 0.3m of obstacles | Should be 0 |
| **Avg Warning/Danger Zone Ratio** | Percentage of time in each zone | < 10% / ≈ 0% |
| **Avg Smoothness** | Mean trajectory curvature (lower = smoother) | < 1.0 |

---

## Per-Run Metrics

Each simulation run records:

- **success**: Whether the robot reached the goal
- **path_length**: Total distance traveled (meters)
- **path_length_ratio**: `path_length / euclidean_distance(start, goal)`
- **time_taken**: Simulation time to reach goal (seconds)
- **stuck_events**: Number of times stuck (0.5m radius, >8s duration)
- **collision_events**: Number of distinct collision intervals
- **min_obstacle_distance**: Closest distance to any obstacle during the run
- **mean_obstacle_distance**: Average of minimum distances at each timestep
- **warning_zone_steps**: Steps with obstacle distance in [0.3m, 0.6m)
- **danger_zone_steps**: Steps with obstacle distance < 0.3m
- **warning_zone_ratio / danger_zone_ratio**: Above as percentage of total steps
- **smoothness**: Mean curvature of trajectory

---

## Zone Definitions

| Zone | Distance | Meaning |
|------|----------|---------|
| Safe | ≥ 0.6m | Normal operation |
| Warning | 0.3–0.6m | Close proximity |
| Danger | < 0.3m | Critical, near-collision |

---

## Notes

- **Median** is used for Path Length Ratio and Time to Goal to reduce outlier sensitivity
- **Collision events** count distinct intervals where robot intersects an obstacle
  - Note: Some collisions may be caused by high-velocity dynamic obstacles hitting the AGV, which is not the AGV's fault
- **Smoothness** = mean curvature; lower values indicate gentler, more efficient paths
