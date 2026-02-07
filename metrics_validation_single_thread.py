"""
metrics_validation_single_thread.py

Single-threaded version of metrics_validation.py for debugging purposes.
Updated to match latest logic:
- 10000 steps limit
- Real simulation integration
- Median metrics
- Collision event counting
"""

from typing import List, Dict, Tuple, Optional
import numpy as np
import os
import argparse
import json
import pprint
import sys
import warnings

# Suppress warnings
warnings.filterwarnings("ignore")

from simulation import SimulationController

def compute_path_length(trajectory: List[Dict[str, float]]) -> float:
    if len(trajectory) < 2:
        return 0.0
    coords = np.array([[s['x'], s['y']] for s in trajectory])
    diffs = np.diff(coords, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    return float(np.sum(dists))

def compute_curvature_smoothness(trajectory: List[Dict[str, float]], dt: float, epsilon: float = 1e-8) -> Optional[float]:
    if len(trajectory) < 3:
        return None
    x = np.array([s['x'] for s in trajectory])
    y = np.array([s['y'] for s in trajectory])
    dx = np.gradient(x, dt)
    dy = np.gradient(y, dt)
    ddx = np.gradient(dx, dt)
    ddy = np.gradient(dy, dt)
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = np.power(dx**2 + dy**2, 1.5) + epsilon
    kappa = numerator / denominator
    kappa = np.nan_to_num(kappa, nan=0.0, posinf=0.0, neginf=0.0)
    return float(np.mean(kappa))

def compute_obstacle_metrics(trajectory, obstacles, robot_radius):
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

def compute_stuck_events(trajectory, stuck_radius, stuck_time, dt):
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
    per_run_metrics = []
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
        
        path_length = compute_path_length(traj)
        straight_line = np.linalg.norm(np.array(goal) - np.array(start))
        
        path_length_ratio = (path_length / straight_line) if (straight_line > 1e-8 and success) else None
        time_taken = (traj[-1]['t'] - traj[0]['t']) if (success and len(traj) >= 2) else None
        
        stuck_events = compute_stuck_events(traj, stuck_radius, stuck_time, dt)
        min_dist, mean_dist, min_distances_t = compute_obstacle_metrics(traj, obstacles, robot_radius)
        
        warning_zone_steps = 0
        danger_zone_steps = 0
        collision_events = 0
        in_collision = False

        if min_distances_t is not None:
            for d in min_distances_t:
                if d < warning_threshold:
                    danger_zone_steps += 1
                elif d < safe_threshold:
                    warning_zone_steps += 1
                
                if d < 0:
                    if not in_collision:
                        collision_events += 1
                        in_collision = True
                else:
                    in_collision = False

        total_steps = len(traj)
        warning_zone_ratio = (warning_zone_steps / total_steps) if total_steps > 0 else 0.0
        danger_zone_ratio = (danger_zone_steps / total_steps) if total_steps > 0 else 0.0
        
        smoothness = compute_curvature_smoothness(traj, dt)
        
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
        
        if stuck_events is not None: stuck_events_list.append(stuck_events)
        collision_events_list.append(collision_events)
        if min_dist is not None: min_obstacle_distances.append(min_dist)
        if mean_dist is not None: mean_obstacle_distances.append(mean_dist)
        warning_zone_steps_list.append(warning_zone_steps)
        danger_zone_steps_list.append(danger_zone_steps)
        if smoothness is not None: smoothness_list.append(smoothness)

    # Aggregated Metrics (Median)
    success_rates = [r['success'] for r in per_run_metrics]
    successful_runs = [r for r in per_run_metrics if r['success']]
    
    median_path_length_ratio = (np.median([r['path_length_ratio'] for r in successful_runs if r['path_length_ratio'] is not None]) 
                              if successful_runs else None)
    median_time_to_goal = (np.median([r['time_taken'] for r in successful_runs if r['time_taken'] is not None]) 
                         if successful_runs else None)

    return {
        'success_rate': np.mean(success_rates) if success_rates else None,
        'median_path_length_ratio_success_only': median_path_length_ratio,
        'median_time_to_goal_success_only': median_time_to_goal,
        'avg_stuck_events': (np.mean(stuck_events_list) if stuck_events_list else None),
        'avg_collision_events': (np.mean(collision_events_list) if collision_events_list else 0.0),
        'avg_min_obstacle_distance': (np.mean(min_obstacle_distances) if min_obstacle_distances else None),
        'avg_mean_obstacle_distance': (np.mean(mean_obstacle_distances) if mean_obstacle_distances else None),
        'avg_warning_zone_steps': (np.mean(warning_zone_steps_list) if warning_zone_steps_list else None),
        'avg_danger_zone_steps': (np.mean(danger_zone_steps_list) if danger_zone_steps_list else None),
        'avg_warning_zone_ratio': (np.mean([r['warning_zone_ratio'] for r in per_run_metrics]) if per_run_metrics else None),
        'avg_danger_zone_ratio': (np.mean([r['danger_zone_ratio'] for r in per_run_metrics]) if per_run_metrics else None),
        'avg_smoothness': (np.mean(smoothness_list) if smoothness_list else None),
        'per_run_metrics': per_run_metrics
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compute metrics (Single Threaded Debug Mode)")
    parser.add_argument('--obstacles', type=int, default=3)
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--navigation', type=str, default='vo', choices=['vo', 'dwa', 'vfh', 'gapnav'],
                        help='Navigation algorithm: vo (Velocity Obstacles), dwa (Dynamic Window Approach), vfh (Vector Field Histogram), gapnav (Gap Navigation)')
    args = parser.parse_args()

    # Simulation Integration
    scenario_map = {'static': 1, 'dynamic': 2, 'mixed': 3}
    scenarios = ['static', 'dynamic', 'mixed']
    
    print(f"Navigation algorithm: {args.navigation.upper()}")
    
    controller = SimulationController(
        l5_variant=args.navigation, 
        path_mode='straight', 
        n_obstacles=args.obstacles,
        steps=10000 # 10000 steps limit
    )

    output_dir = "metrics_output"
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Starting Single-Threaded Validation: {args.runs} runs per scenario")

    for scenario in scenarios:
        runs_data = []
        scenario_id = scenario_map[scenario]
        
        for i in range(args.runs):
            print(f"Running {scenario} ({i+1}/{args.runs})...")
            
            # Reset
            controller.n_obstacles = args.obstacles
            # Suppress simulation print
            old_stdout = sys.stdout
            try:
                sys.stdout = open(os.devnull, 'w')
                controller.reset_scenario(scenario_id)
            finally:
                sys.stdout = old_stdout
            
            trajectory_points = []
            goal_reached = False
            
            for step in range(10000):
                data = controller.step(step)
                
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
                
            runs_data.append({
                'trajectory': trajectory_points,
                'start': (0.0, 0.0),
                'goal': (10.0, 5.0),
                'obstacles': obs_list,
                'success': goal_reached,
                'dt': controller.dt,
                'scenario': scenario
            })
            
        metrics = compute_metrics(runs_data, 0.3)
        filename = f"metrics_output_{args.navigation}_obstacles_{args.obstacles}_{scenario}_single.json"
        
        with open(os.path.join(output_dir, filename), 'w') as f:
            json.dump(metrics, f, indent=2)
            
        print(f"Saved {scenario} metrics (Success: {metrics['success_rate']:.2f})")
