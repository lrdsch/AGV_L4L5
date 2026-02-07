import json
import os
import re
import pandas as pd
import glob

# Configuration
METRICS_DIR = r"c:\Users\Leonardo\Desktop\Progetti\AGV_L4L5\metrics_output"
OUTPUT_FILE = r"c:\Users\Leonardo\Desktop\Progetti\AGV_L4L5\metric_comparison.md"

# Regex to parse filename: metrics_output_[algo_]obstacles_[count]_[scenario].json
# Examples:
# metrics_output_obstacles_10_dynamic.json -> algo=None (Default), count=10, scenario=dynamic
# metrics_output_dwa_obstacles_10_dynamic.json -> algo=dwa, count=10, scenario=dynamic
FILENAME_PATTERN = re.compile(r"metrics_output_(?:(?P<algo>[a-z0-9]+)_)?obstacles_(?P<count>\d+)_(?P<scenario>[a-z]+)\.json")

def parse_filename(filename):
    match = FILENAME_PATTERN.match(filename)
    if match:
        data = match.groupdict()
        algo = data['algo'] if data['algo'] else "Default"
        # Normalize Algorithm names for display
        algo_map = {
            "Default": "Default (VO)",
            "dwa": "DWA",
            "gapnav": "GapNav",
            "vfh": "VFH"
        }
        return algo_map.get(algo, algo.capitalize()), int(data['count']), data['scenario']
    return None, None, None

def main():
    print(f"Scanning {METRICS_DIR}...")
    files = glob.glob(os.path.join(METRICS_DIR, "*.json"))
    
    # Collect all possible values to build a complete grid
    found_algos = set()
    found_counts = set()
    found_scenarios = set()
    
    parsed_files = {} # Key: (Algo, Count, Scenario) -> filepath
    
    for filepath in files:
        filename = os.path.basename(filepath)
        algo, count, scenario = parse_filename(filename)
        if algo and count and scenario:
            found_algos.add(algo)
            found_counts.add(count)
            found_scenarios.add(scenario)
            parsed_files[(algo, count, scenario)] = filepath
            
    print(f"Found {len(parsed_files)} valid metric files.")
    
    # Define the structure for the final table
    # We want to iterate through all combinations to handle missing files (empty cells)
    table_rows = []
    
    # Sort keys for consistent output
    sorted_algos = sorted(list(found_algos))
    sorted_counts = sorted(list(found_counts))
    sorted_scenarios = sorted(list(found_scenarios))
    
    for count in sorted_counts:
        for algo in sorted_algos:
            for scenario in sorted_scenarios:
                key = (algo, count, scenario)
                row = {
                    "Algorithm": algo,
                    "Obstacles": count,
                    "Scenario": scenario,
                }
                
                if key in parsed_files:
                    try:
                        with open(parsed_files[key], 'r') as f:
                            data = json.load(f)
                        
                        row.update({
                            "Success Rate": f"{data.get('success_rate', 0):.3f}",
                            "Time (s)": f"{data.get('median_time_to_goal_success_only', 0):.2f}",
                            "Path Ratio": f"{data.get('median_path_length_ratio_success_only', 0):.3f}",
                            "Collisions": f"{data.get('avg_collision_events', 0):.3f}",
                            "Stuck": f"{data.get('avg_stuck_events', 0):.3f}",
                            "Smoothness": f"{data.get('avg_smoothness', 0):.3f}",
                            "Min Dist": f"{data.get('avg_min_obstacle_distance', 0):.3f}"
                        })
                    except Exception as e:
                        print(f"Error reading {key}: {e}")
                        row.update({k: "ERR" for k in ["Success Rate", "Time (s)", "Path Ratio", "Collisions", "Stuck", "Smoothness", "Min Dist"]})
                else:
                    # Empty cells for missing files
                    row.update({k: "-" for k in ["Success Rate", "Time (s)", "Path Ratio", "Collisions", "Stuck", "Smoothness", "Min Dist"]})
                
                table_rows.append(row)

    if not table_rows:
        print("No data collected.")
        return

    df = pd.DataFrame(table_rows)
    
    # Create Markdown content
    markdown_content = f"""# Navigation Metrics Report
    
**Generated on:** {pd.Timestamp.now().isoformat()}
**Source Directory:** `{METRICS_DIR}`

## Comprehensive Metrics Table

This table includes all found combinations of Algorithms, Obstacle Counts, and Scenarios. Missing files are valid combinations that were not found in the directory.

"""
    markdown_content += df.to_markdown(index=False)
    
    # Save to file
    os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
    with open(OUTPUT_FILE, "w", encoding='utf-8') as f:
        f.write(markdown_content)
        
    print(f"Successfully generated metrics comparison at: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
