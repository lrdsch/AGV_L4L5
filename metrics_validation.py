"""
metrics_validation.py

Module for validating navigation algorithm performance over multiple simulation runs.
Computes per-run and aggregated metrics for robot navigation tasks.

Author: HySDG-ESD Project Team
"""

from typing import List, Dict, Tuple, Optional
import numpy as np
import os
import multiprocessing

# Global configuration to suppress warnings in all processes
# Must be set before importing libraries that use joblib
os.environ["LOKY_MAX_CPU_COUNT"] = str(multiprocessing.cpu_count())

def compute_path_length(trajectory: List[Dict[str, float]]) -> float:
    """
    Compute total path length of the trajectory.
    """
    if len(trajectory) < 2:
        return 0.0
    coords = np.array([[s['x'], s['y']] for s in trajectory])
    diffs = np.diff(coords, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    return float(np.sum(dists))

def compute_curvature_smoothness(trajectory: List[Dict[str, float]], dt: float, epsilon: float = 1e-8) -> Optional[float]:
    """
    Compute mean curvature (smoothness) of the trajectory using finite differences.
    Returns None if trajectory is too short.
    """
    if len(trajectory) < 3:
        return None
    x = np.array([s['x'] for s in trajectory])
    y = np.array([s['y'] for s in trajectory])
    # First derivatives
    dx = np.gradient(x, dt)
    dy = np.gradient(y, dt)
    # Second derivatives
    ddx = np.gradient(dx, dt)
    ddy = np.gradient(dy, dt)
    # Curvature formula
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = np.power(dx**2 + dy**2, 1.5) + epsilon
    kappa = numerator / denominator
    # Remove NaN/inf
    kappa = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)
    return float(np.mean(kappa))

def compute_obstacle_metrics(
    trajectory: List[Dict[str, float]],
    obstacles: List[Dict[str, float]],
    robot_radius: float
) -> Tuple[Optional[float], Optional[float], Optional[List[float]]]:
    """
    Compute minimum and mean distance to obstacles at each timestep.
    Returns (min_distance, mean_distance, list of min_distances per timestep)
    Returns (None, None, None) if obstacles is empty.
    """
    if not obstacles:
        return None, None, None
    obs_xy = np.array([[o['x'], o['y']] for o in obstacles])
    obs_r = np.array([o['radius'] for o in obstacles])
    min_distances = []
    for state in trajectory:
        p = np.array([state['x'], state['y']])
        dists = np.linalg.norm(obs_xy - p, axis=1) - (robot_radius + obs_r)
        min_dist = np.min(dists)
        min_distances.append(float(min_dist))
    min_distance = float(np.min(min_distances))
    mean_distance = float(np.mean(min_distances))
    return min_distance, mean_distance, min_distances

def compute_stuck_events(
    trajectory: List[Dict[str, float]],
    stuck_radius: float,
    stuck_time: float,
    dt: float
) -> int:
    """
    Count number of stuck events: robot stays within stuck_radius for at least stuck_time seconds.
    """
    if len(trajectory) < 2:
        return 0
    coords = np.array([[s['x'], s['y']] for s in trajectory])
    stuck_steps = int(np.ceil(stuck_time / dt))
    stuck_count = 0
    i = 0
    N = len(coords)
    while i < N - stuck_steps:
        window = coords[i:i+stuck_steps]
        center = window[0]
        dists = np.linalg.norm(window - center, axis=1)
        if np.all(dists <= stuck_radius):
            stuck_count += 1
            # Skip ahead until robot leaves stuck_radius
            j = i + stuck_steps
            while j < N and np.linalg.norm(coords[j] - center) <= stuck_radius:
                j += 1
            i = j
        else:
            i += 1
    return stuck_count

def compute_metrics(
    runs: List[Dict],
    robot_radius: float,
    safe_threshold: float = 0.6,
    warning_threshold: float = 0.3,
    stuck_radius: float = 0.5,
    stuck_time: float = 8.0
) -> Dict:
    """
    Compute aggregated and per-run metrics for navigation performance validation.
    """
    per_run_metrics = []
    num_success = 0
    path_length_ratios = []
    times_to_goal = []
    stuck_events_list = []
    collision_events_list = []
    min_obstacle_distances = []
    mean_obstacle_distances = []
    warning_zone_steps_list = []
    danger_zone_steps_list = []
    smoothness_list = []
    for run in runs:
        traj = run['trajectory']
        start = run['start']
        goal = run['goal']
        obstacles = run.get('obstacles', [])
        success = run.get('success', False)
        dt = run['dt']
        # Path length
        path_length = compute_path_length(traj)
        # Straight line
        straight_line = np.linalg.norm(np.array(goal) - np.array(start))
        # Path length ratio
        if straight_line > 1e-8 and success:
            path_length_ratio = path_length / straight_line
        else:
            path_length_ratio = None
        # Time to goal
        if success and len(traj) >= 2:
            time_taken = traj[-1]['t'] - traj[0]['t']
        else:
            time_taken = None
        # Stuck events
        stuck_events = compute_stuck_events(traj, stuck_radius, stuck_time, dt)
        # Obstacle metrics
        min_dist, mean_dist, min_distances_t = compute_obstacle_metrics(traj, obstacles, robot_radius)
        # Safety zones
        warning_zone_steps = 0
        danger_zone_steps = 0
        if min_distances_t is not None:
            for d in min_distances_t:
                if d < warning_threshold:
                    danger_zone_steps += 1
                elif d < safe_threshold:
                    warning_zone_steps += 1
        
        # Normalize safety zones (percentage of time steps)
        total_steps = len(traj)
        if total_steps > 0:
            warning_zone_ratio = warning_zone_steps / total_steps
            danger_zone_ratio = danger_zone_steps / total_steps
        else:
            warning_zone_ratio = 0.0
            danger_zone_ratio = 0.0

        # Collision events (distinct intervals where distance < 0)
        collision_events = 0
        in_collision = False
        if min_distances_t is not None:
            for d in min_distances_t:
                if d < 0:
                    if not in_collision:
                        collision_events += 1
                        in_collision = True
                else:
                    in_collision = False

        # Smoothness
        smoothness = compute_curvature_smoothness(traj, dt)
        # Per-run dict
        per_run = {
            'success': success,
            'path_length': path_length,
            'path_length_ratio': path_length_ratio,
            'time_taken': time_taken,
            'stuck_events': stuck_events,
            'collision_events': collision_events,
            'min_obstacle_distance': min_dist,
            'mean_obstacle_distance': mean_dist,
            'warning_zone_steps': warning_zone_steps,
            'danger_zone_steps': danger_zone_steps,
            'warning_zone_ratio': warning_zone_ratio, 
            'danger_zone_ratio': danger_zone_ratio,
            'smoothness': smoothness
        }
        per_run_metrics.append(per_run)
        # Aggregation
        if success:
            num_success += 1
            if path_length_ratio is not None:
                path_length_ratios.append(path_length_ratio)
            if time_taken is not None:
                times_to_goal.append(time_taken)
        if stuck_events is not None:
            stuck_events_list.append(stuck_events)
        if min_dist is not None:
            min_obstacle_distances.append(min_dist)
        if mean_dist is not None:
            mean_obstacle_distances.append(mean_dist)
        
        # Track collision events for aggregation
        collision_events_list.append(collision_events)

        warning_zone_steps_list.append(warning_zone_steps)
        danger_zone_steps_list.append(danger_zone_steps)
        if smoothness is not None:
            smoothness_list.append(smoothness)
    N = len(runs)
    # Aggregated metrics
    # Aggregated Metrics across all runs
    # Use Median for Time and Path Ratio to be robust against outliers
    success_rates = [r['success'] for r in per_run_metrics]
    
    successful_runs = [r for r in per_run_metrics if r['success']]
    median_path_length_ratio = (np.median([r['path_length_ratio'] for r in successful_runs if r['path_length_ratio'] is not None]) 
                              if successful_runs else None)
    median_time_to_goal = (np.median([r['time_taken'] for r in successful_runs if r['time_taken'] is not None]) 
                         if successful_runs else None)

    avg_stuck_events = (np.mean(stuck_events_list) if stuck_events_list else None)
    avg_min_obstacle_distance = (np.mean(min_obstacle_distances) if min_obstacle_distances else None)
    avg_mean_obstacle_distance = (np.mean(mean_obstacle_distances) if mean_obstacle_distances else None)
    avg_warning_zone_steps = (np.mean(warning_zone_steps_list) if warning_zone_steps_list else None)
    avg_danger_zone_steps = (np.mean(danger_zone_steps_list) if danger_zone_steps_list else None)
    avg_warning_zone_ratio = (np.mean([r['warning_zone_ratio'] for r in per_run_metrics]) if per_run_metrics else None)
    avg_danger_zone_ratio = (np.mean([r['danger_zone_ratio'] for r in per_run_metrics]) if per_run_metrics else None)
    avg_smoothness = (np.mean(smoothness_list) if smoothness_list else None)
    avg_collision_events = (np.mean(collision_events_list) if collision_events_list else 0.0)
    
    return {
        'success_rate': np.mean(success_rates) if success_rates else None,
        'median_path_length_ratio_success_only': median_path_length_ratio,
        'median_time_to_goal_success_only': median_time_to_goal,
        'avg_stuck_events': avg_stuck_events,
        'avg_collision_events': avg_collision_events,
        'avg_min_obstacle_distance': avg_min_obstacle_distance,
        'avg_mean_obstacle_distance': avg_mean_obstacle_distance,
        'avg_warning_zone_steps': avg_warning_zone_steps,
        'avg_danger_zone_steps': avg_danger_zone_steps,
        'avg_warning_zone_ratio': avg_warning_zone_ratio, 
        'avg_danger_zone_ratio': avg_danger_zone_ratio,
        'avg_smoothness': avg_smoothness,
        'per_run_metrics': per_run_metrics
    }


def run_simulation_task(scenario_name: str, scenario_id: int, n_obstacles: int, run_idx: int) -> Dict:
    """
    Worker function to run a single simulation in a separate process.
    """
    import sys
    import os
    import multiprocessing
    
    # Configure Joblib/Loky inside the worker process to avoid warnings
    # MUST be done before importing modules that use joblib
    os.environ["LOKY_MAX_CPU_COUNT"] = str(multiprocessing.cpu_count())
    
    # Suppress warnings
    import warnings
    warnings.filterwarnings("ignore")

    from simulation import SimulationController

    # Suppress stdout to avoid clutter from parallel processes
    try:
        sys.stdout = open(os.devnull, 'w')
    except:
        pass

    try:
        controller = SimulationController(
            l5_variant='vo',
            path_mode='straight',
            n_obstacles=n_obstacles,
            steps=10000  # Increased limit to 10000 steps (1000s)
        )
        controller.n_obstacles = n_obstacles
        controller.reset_scenario(scenario_id)

        trajectory_points = []
        goal_reached = False
        max_steps = 10000 # Increased limit
        
        for step in range(max_steps):
            data = controller.step(step)
            
            # Collect Trajectory Data
            agv_pos = data['agv_pos']
            agv_vel = data['agv_vel']
            current_time = data['time']
            
            trajectory_points.append({
                't': float(current_time), 
                'x': float(agv_pos[0]), 
                'y': float(agv_pos[1]), 
                'vx': float(agv_vel[0]), 
                'vy': float(agv_vel[1])
            })
            
            if data.get('goal_reached', False):
                goal_reached = True
                break
        
        # Collect Obstacles
        gt_obstacles = data.get('ground_truth_obstacles', [])
        obs_list = []
        for obs in gt_obstacles:
            obs_list.append({
                'x': float(obs['center'][0]),
                'y': float(obs['center'][1]),
                'radius': float(obs.get('radius', 0.3))
            })
            
        return {
            'trajectory': trajectory_points,
            'start': (0.0, 0.0), 
            'goal': (10.0, 5.0), 
            'obstacles': obs_list,
            'success': goal_reached,
            'collision': False, 
            'dt': controller.dt,
            'scenario': scenario_name
        }
    except Exception as e:
        return {'error': str(e), 'scenario': scenario_name}
    finally:
        # Restore stdout (though process will likely end)
        sys.stdout = sys.__stdout__

if __name__ == "__main__":
    import argparse
    import json
    import pprint
    import sys
    import os
    import multiprocessing
    import concurrent.futures

    # Fix for Windows multiprocessing
    multiprocessing.freeze_support()

    # Suppress warnings
    import warnings
    warnings.filterwarnings("ignore")
    
    # Configure Joblib/Loky to avoid "physical cores" warning
    import os
    os.environ["LOKY_MAX_CPU_COUNT"] = str(multiprocessing.cpu_count())

    parser = argparse.ArgumentParser(description="Compute and save navigation metrics.")
    parser.add_argument('--obstacles', type=int, default=3, help='Number of obstacles (for dummy data)')
    parser.add_argument('--runs', type=int, default=10, help='Number of simulations per scenario')
    parser.add_argument('--output', type=str, default=None, help='Output file name (default: metrics_output_obstacles_<N>.json)')
    parser.add_argument('--jobs', type=int, default=None, help='Number of parallel jobs (default: all available cores)')
    args = parser.parse_args()

    # Detect Cores
    available_cores = multiprocessing.cpu_count()
    if args.jobs:
        num_cores = args.jobs
        print(f"Using {num_cores} workers (user specified). Available cores: {available_cores}")
    else:
        num_cores = available_cores
        print(f"Detected {available_cores} CPU cores. Using all.")
        
    print(f"Starting parallel execution on {num_cores} workers...")

    # Map scenarios to simulation IDs (1: Static, 2: Dynamic, 3: Mixed)
    scenario_map = {
        'static': 1,
        'dynamic': 2,
        'mixed': 3
    }
    scenarios = ['static', 'dynamic', 'mixed']
    total_runs = args.runs * len(scenarios)

    runs = []
    
    # Run simulations in parallel
    with concurrent.futures.ProcessPoolExecutor(max_workers=num_cores) as executor:
        futures = []
        for scenario_name in scenarios:
            scenario_id = scenario_map[scenario_name]
            for i in range(args.runs):
                futures.append(
                    executor.submit(run_simulation_task, scenario_name, scenario_id, args.obstacles, i)
                )
        
        completed_count = 0
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result and 'error' not in result:
                runs.append(result)
            elif result and 'error' in result:
                print(f"\nError in simulation run: {result['error']}")
            
            completed_count += 1
            sys.stdout.write(f"\rProgress: {completed_count}/{total_runs} ({(completed_count/total_runs)*100:.1f}%)")
            sys.stdout.flush()

    print("\nBatch simulation completed.")
    robot_radius = 0.3

    # Disgrega per scenario
    output_dir = "metrics_output"
    os.makedirs(output_dir, exist_ok=True)
    
    for scenario in scenarios:
        runs_scenario = [r for r in runs if r.get('scenario') == scenario]
        if not runs_scenario:
            print(f"No successful runs for scenario '{scenario}'")
            continue
            
        metrics = compute_metrics(runs_scenario, robot_radius)
        
        filename = f"metrics_output_obstacles_{args.obstacles}_{scenario}.json"
        if args.output:
             filename = f"{args.output}_{scenario}.json"
        
        output_file = os.path.join(output_dir, filename)
        
        with open(output_file, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"Metrics for scenario '{scenario}' saved to {output_file}")
        pprint.pprint(metrics)
