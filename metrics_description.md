# Metrics and Algorithm Description

## Navigation Metrics

The following metrics are computed over N simulations (e.g., N=100 or N=1000) to evaluate the performance of the autonomous vehicle.

### 1. Success Rate
The ratio of successful runs to the total number of simulations.
- **Formula:** `Success Rate = (Number of Successful Runs) / N`

### 2. Average Path Length Ratio
The ratio of the actual path length traveled by the robot to the straight-line distance from start to goal. This metric indicates the efficiency of the path.
- **Formula:** `Ratio = (Path Length) / (Euclidean Distance(Start, Goal))`
- **Note:** This is calculated **only for successful runs**.

### 3. Average Time Taken
The average time required for the robot to reach the goal.
- **Note:** Calculated **only for successful runs**.

### 4. Average Number of Stuck Events
Measures how often the robot effectively stops or enters a deadlock situation.
- **Definition:** A "stuck event" occurs if the robot stays within a radius of **0.5m** for more than **8.0 seconds**.
- **Note:** The threshold of 8 seconds was chosen as a robust indicator of stagnation (within the suggested 5-10s range).

### 5. Obstacle Clearance Metrics
Aggregated statistics on the distance between the robot and obstacles throughout the simulations.
- **Minimum Distance to Obstacles:** The absolute minimum distance recorded between the robot and any obstacle across all time steps.
- **Average Distance to Obstacles:** The average of the minimum distances recorded at each time step.

### 6. Safety Margin Violations
Quantifies how often the robot enters defined proximity zones around obstacles.
- **Warning Zone:** Distance < **0.6m**
- **Danger Zone:** Distance < **0.3m**
- **Metrics:**
  - **Average Steps:** The average number of time steps spent in these zones per simulation.
### 6. Safety Margin Violations
Quantifies how often the robot enters defined proximity zones around obstacles.
- **Warning Zone:** Distance < **0.6m**
- **Danger Zone:** Distance < **0.3m**
- **Metrics:**
  - **Average Steps:** The average number of time steps spent in these zones per simulation.
  - **Average Ratio:** The average percentage of time (steps/total_steps) spent in these zones per simulation.

### 7. Average Collision Events
Measures the number of distinct collision occurrences per simulation.
- **Definition:** A collision event is defined as a continuous time interval where the distance to an obstacle is less than **0.0m**.
- **Metric:** The average number of such distinct events across all simulations.

### 8. Average Smoothness
Measures the smoothness of the trajectory based on the curvature of the path. Lower curvature indicates a smoother path.
- **Formula:** For a parametric curve $(x(t), y(t))$, the curvature $\kappa$ is given by:
  $$ \kappa(t) = \frac{|\dot{x}\ddot{y} - \dot{y}\ddot{x}|}{(\dot{x}^2 + \dot{y}^2)^{3/2}} $$
- The metric reports the average curvature over the trajectory.

---

## Velocity Obstacles (VO) Algorithm

The Velocity Obstacles (VO) algorithm enables dynamic obstacle avoidance for autonomous vehicles by computing forbidden velocity regions that would result in collisions within a time horizon. Candidate velocities are evaluated according to a prioritized strategy: the agent first attempts to pass behind moving obstacles, then slows down or stops if necessary, and finally considers passing in front only if safe.

- **Time Horizon:** A time horizon of **6 seconds** is used for trajectory prediction. This value was empirically determined to offer an optimal tradeoff between safety (sufficient reaction time) and reactivity (avoiding overly conservative behavior).
