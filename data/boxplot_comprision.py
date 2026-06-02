#!/usr/bin/env python3
"""
Visualize resource usage data from Docker container logs.
Creates grouped boxplots for CPU and memory usage across different modes and frequencies.
"""

import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Configuration
ROOT_FOLDER = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data"  # Change this to your root folder path
MODES = ["centered", "distributed1", "distributed2"]
FREQUENCIES = ["standby", "30hz", "60hz"]
ROUNDS = ["r1", "r2", "r3"]
RESOURCE_USAGE_FOLDER = "resource_usage_final"

# Color scheme for modes
MODE_COLORS = {
    "centered": "#2E86AB",      # Blue
    "distributed1": "#A23B72",  # Purple
    "distributed2": "#F18F01"   # Orange
}


def parse_log_file(filepath):
    """
    Parse a single log file and extract timestamp, cpu_percent, and mem_process_mib.
    
    Returns:
        DataFrame with columns: timestamp, cpu_percent, mem_process_mib, container_name
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Extract container name
    container_name = None
    for line in lines:
        if line.startswith("monitor_ubuntu_simulator"):
            container_name = 'simulator'
        elif line.startswith("monitor_ubuntu_teleop_"):
            match = re.search(r'container_name:\s*(.+)', line)
            if match:
                container_name = match.group(1).strip()
                # Clean up container name
                container_name = container_name.replace("monitor_", "")
                container_name = container_name.replace("ubuntu_", "")
                container_name = container_name.replace("teleop-", "")
                container_name = container_name.replace("teleop_", "")
                container_name = container_name.replace('components', 'algorithm')
                break
    
    if not container_name:
        return None
    
    # Parse data lines
    data = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("monitor_ubuntu_teleop_"):
            continue
        
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                timestamp = float(parts[0].strip())
                cpu_percent = float(parts[1].strip())
                mem_process_mib = float(parts[3].strip())
                data.append({
                    'timestamp': timestamp,
                    'cpu_percent': cpu_percent,
                    'mem_process_mib': mem_process_mib,
                    'container_name': container_name
                })
            except ValueError:
                continue
    
    if not data:
        return None
    
    df = pd.DataFrame(data)
    return df


def filter_data(df, seconds_to_remove=5):
    """
    Remove first and last N seconds of data and remove zero values.
    """
    if df is None or len(df) == 0:
        return None
    
    # Get timestamp range
    min_ts = df['timestamp'].min()
    max_ts = df['timestamp'].max()
    
    # Filter by time
    df = df[(df['timestamp'] >= min_ts + seconds_to_remove) & 
            (df['timestamp'] <= max_ts - seconds_to_remove)]
    
    # Remove zeros
    df = df[(df['cpu_percent'] > 0) & (df['mem_process_mib'] > 0)]
    
    return df if len(df) > 0 else None


def collect_data(root_folder):
    """
    Collect all data from the folder structure.
    
    Returns:
        Dictionary: {frequency: {mode: {container: [data_points]}}}
    """
    data_structure = {freq: {mode: {} for mode in MODES} for freq in FREQUENCIES}
    
    for mode in MODES:
        mode_path = Path(root_folder) / mode / RESOURCE_USAGE_FOLDER
        
        if not mode_path.exists():
            print(f"Warning: Path not found: {mode_path}")
            continue
        
        for freq in FREQUENCIES:
            for round_num in ROUNDS:
                if mode=='centered' and freq == '30hz' and round_num == 'r1':
                    print(mode, freq, round_num)
                    continue
                
                subfolder_name = f"monitor_{freq}_{round_num}"

                subfolder_path = mode_path / subfolder_name
                
                if not subfolder_path.exists():
                    print(f"Warning: Subfolder not found: {subfolder_path}")
                    continue
                
                # Find all log files in the subfolder
                log_files = list(subfolder_path.glob("*.log"))
                if not log_files:
                    log_files = list(subfolder_path.glob("*"))  # Try any file
                
                for log_file in log_files:
                    if log_file.is_file():
                        df = parse_log_file(log_file)
                        if df is not None:
                            df = filter_data(df)
                            if df is not None:
                                container = df['container_name'].iloc[0]
                                
                                if container not in data_structure[freq][mode]:
                                    data_structure[freq][mode][container] = {
                                        'cpu_percent': [],
                                        'mem_process_mib': []
                                    }
                                
                                # Append all data points (blending rounds)
                                data_structure[freq][mode][container]['cpu_percent'].extend(
                                    df['cpu_percent'].tolist()
                                )
                                data_structure[freq][mode][container]['mem_process_mib'].extend(
                                    df['mem_process_mib'].tolist()
                                )
    
    return data_structure


def create_grouped_boxplot(data_structure, metric, ylabel, filename):
    """
    Create grouped boxplots for a specific metric.
    
    Args:
        data_structure: Dictionary with collected data
        metric: 'cpu_percent' or 'mem_process_mib'
        ylabel: Label for y-axis
        filename: Output filename
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    fig.suptitle(f'{ylabel} by Container and Mode', fontsize=16, fontweight='bold')
    
    for idx, freq in enumerate(FREQUENCIES):
        ax = axes[idx]
        
        # Get all containers for this frequency
        all_containers = set()
        for mode in MODES:
            all_containers.update(data_structure[freq][mode].keys())
        
        containers = sorted(list(all_containers))
        
        if not containers:
            ax.set_title(f'{freq.upper()}')
            ax.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax.transAxes)
            continue
        
        # Prepare data for boxplot
        positions = []
        data_to_plot = []
        colors = []
        labels = []
        
        num_modes = len(MODES)
        group_width = num_modes * 1  # Width of each group
        group_spacing = 1  # Spacing between groups
        
        for i, container in enumerate(containers):
            group_center = i * (group_width + group_spacing)
            
            for j, mode in enumerate(MODES):
                if container in data_structure[freq][mode]:
                    values = data_structure[freq][mode][container][metric]
                    if values:
                        pos = group_center + (j - num_modes/2 + 0.5) * 0.8
                        positions.append(pos)
                        data_to_plot.append(values)
                        colors.append(MODE_COLORS[mode])
        
        # Create boxplot
        bp = ax.boxplot(data_to_plot, positions=positions, widths=0.6,
                        patch_artist=True, showfliers=True,
                        boxprops=dict(linewidth=1.5),
                        medianprops=dict(color='black', linewidth=2),
                        whiskerprops=dict(linewidth=1.5),
                        capprops=dict(linewidth=1.5))
        
        # Color the boxes
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        
        # Set x-axis labels
        group_centers = [i * (group_width + group_spacing) for i in range(len(containers))]
        ax.set_xticks(group_centers)
        ax.set_xticklabels(containers, rotation=15, ha='right')
        
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(f'{freq.upper()}', fontsize=14, fontweight='bold')
        ax.grid(axis='y', alpha=0.3, linestyle='--')
    
    # Create legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=MODE_COLORS[mode], alpha=0.7, label=mode) 
                      for mode in MODES]
    fig.legend(handles=legend_elements, loc='upper center', 
              bbox_to_anchor=(0.5, -0.02), ncol=3, fontsize=12)
    
    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()


def main():
    print("Collecting data from log files...")
    data_structure = collect_data(ROOT_FOLDER)
    
    # # Check if data was collected
    # data_found = False
    # for freq in FREQUENCIES:
    #     for mode in MODES:
    #         if data_structure[freq][mode]:
    #             data_found = True
    #             break
    #     if data_found:
    #         break
    
    # if not data_found:
    #     print("No data found! Please check:")
    #     print(f"1. ROOT_FOLDER is set correctly (currently: {ROOT_FOLDER})")
    #     print(f"2. Folder structure is: <mode>/{RESOURCE_USAGE_FOLDER}/monitor_<freq>_<round>/*.log")
    #     print(f"3. Log files exist in the expected locations")
    #     return
    
    print("\nCreating visualizations...")
    
    # Create CPU boxplot
    create_grouped_boxplot(
        data_structure,
        metric='cpu_percent',
        ylabel='CPU Usage (%)',
        filename='cpu_usage_boxplot_comprision.png'
    )
    
    # Create Memory boxplot
    create_grouped_boxplot(
        data_structure,
        metric='mem_process_mib',
        ylabel='Memory Usage (MiB)',
        filename='memory_usage_boxplot_comprision.png'
    )
    
    print("\nVisualization complete!")
    print("Generated files:")
    print("  - cpu_usage_boxplot.png")
    print("  - memory_usage_boxplot.png")


if __name__ == "__main__":
    main()