import os
import re
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def clean_container_name(name):
    """Remove monitor, ubuntu, teleop strings from container name."""
    name = name.replace('monitor_', '').replace('ubuntu_', '').replace('teleop_', '')
    name = name.replace('monitor-', '').replace('ubuntu-', '').replace('teleop-', '')
    name = name.replace('config-', '').replace('files-', '').replace('-1', '')
    name = name.replace('config_', '').replace('files_', '').replace('_1', '')
    name = name.replace('components', 'algorithm')
    name = name.replace('-', ' ').replace('_', ' ')
    return name.strip()

def parse_log_file(filepath):
    """Parse a single log file and extract resource usage data."""
    container_name = None
    data = []
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Extract container name from first line
    if lines and 'container_name:' in lines[0]:
        container_name = lines[0].split('container_name:')[-1].strip()
        container_name = clean_container_name(container_name)
    
    # Parse data lines (skip header lines)
    for line in lines:
        line = line.strip()
        if not line or 'monitor_' in line or 'container_id' in line or 'timestamp' in line or 'samples:' in line:
            continue
        
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                timestamp = float(parts[0].strip())
                cpu_percent = float(parts[1].strip())
                mem_process_mib = float(parts[3].strip())
                data.append((timestamp, cpu_percent, mem_process_mib))
            except ValueError:
                continue
    
    return container_name, data

def filter_time_range(data):
    """Remove first 5 and last 5 seconds of data."""
    if not data:
        return []
    
    timestamps = [d[0] for d in data]
    start_time = min(timestamps)
    end_time = max(timestamps)
    
    filtered_data = [d for d in data if start_time + 5 <= d[0] <= end_time - 5]
    return filtered_data

def remove_zeros(values):
    """Remove zero values from the list."""
    return [v for v in values if v != 0]

def collect_data(root_folder, target_subfolders):
    """Collect data from all relevant subfolders."""
    # Structure: mode -> frequency -> with_dc -> round -> container_name -> data
    all_data = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(list)))))
    
    modes = ['centered', 'distributed1', 'distributed2']
    
    for mode in modes:
        mode_path = os.path.join(root_folder, mode, 'resource_usage_final')
        if not os.path.exists(mode_path):
            print(f"Warning: {mode_path} does not exist")
            continue
        
        for subfolder in target_subfolders:
            subfolder_path = os.path.join(mode_path, subfolder)
            if not os.path.exists(subfolder_path):
                print(f"Warning: {subfolder_path} does not exist")
                continue
            
            # Parse subfolder name to extract frequency, round, and DC status
            # Format: monitor_30hz_r1, monitor_wDC_30hz_r1, etc.
            parts = subfolder.replace('monitor_', '').split('_')
            
            with_dc = 'wDC' in subfolder
            
            # Extract frequency and round
            freq = None
            round_num = None
            for part in parts:
                if 'hz' in part:
                    freq = part
                if part.startswith('r') and len(part) == 2:
                    round_num = part
            
            if not freq or not round_num:
                print(f"Warning: Could not parse {subfolder}")
                continue
            
            # Process all log files in this subfolder
            for filename in os.listdir(subfolder_path):
                if filename.endswith('.log') or filename.startswith('monitor_'):
                    filepath = os.path.join(subfolder_path, filename)
                    try:
                        container_name, data = parse_log_file(filepath)
                        if container_name and data:
                            filtered_data = filter_time_range(data)
                            all_data[mode][freq][with_dc][round_num][container_name].extend(filtered_data)
                    except Exception as e:
                        print(f"Error processing {filepath}: {e}")
    
    return all_data

def create_grouped_boxplots(all_data, metric_name, metric_index):
    """Create grouped boxplots for a specific metric with 3 subplots (one per mode)."""
    modes = ['centered', 'distributed1', 'distributed2']
    frequencies = ['30hz', '60hz']
    
    # Color scheme: different colors for DC conditions, same across all modes
    color_without_dc = '#3498db'  # Blue for without DC
    color_with_dc = '#e74c3c'     # Red for with DC
    
    for freq in frequencies:
        # Create figure with 3 subplots (one per mode)
        fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
        
        # Collect all container names across all modes
        all_containers = set()
        for mode in modes:
            for with_dc in [False, True]:
                for round_num in ['r1', 'r2', 'r3']:
                    if mode in all_data and freq in all_data[mode] and with_dc in all_data[mode][freq]:
                        if round_num in all_data[mode][freq][with_dc]:
                            all_containers.update(all_data[mode][freq][with_dc][round_num].keys())
        
        containers = sorted(list(all_containers))
        
        if not containers:
            print(f"No data for frequency {freq}")
            continue
        
        # Plot for each mode
        for mode_idx, mode in enumerate(modes):
            ax = axes[mode_idx]
            
            # Prepare data for plotting this mode
            box_data = []
            box_positions = []
            box_colors = []
            tick_positions = []
            tick_labels = []
            
            box_width = 0.3
            group_gap = 1.0  # Gap between container groups
            within_group_gap = 0.1  # Gap between w/o DC and w/ DC boxes
            
            for i, container in enumerate(containers):
                base_pos = i * group_gap
                tick_positions.append(base_pos + box_width / 2)
                tick_labels.append(container)
                
                # Without DC
                without_dc_data = []
                for round_num in ['r1', 'r2', 'r3']:
                    if (mode in all_data and freq in all_data[mode] and 
                        False in all_data[mode][freq] and round_num in all_data[mode][freq][False] and
                        container in all_data[mode][freq][False][round_num]):
                        data = all_data[mode][freq][False][round_num][container]
                        values = [d[metric_index] for d in data]
                        values = remove_zeros(values)
                        without_dc_data.extend(values)
                
                # With DC
                with_dc_data = []
                for round_num in ['r1', 'r2', 'r3']:
                    if (mode in all_data and freq in all_data[mode] and 
                        True in all_data[mode][freq] and round_num in all_data[mode][freq][True] and
                        container in all_data[mode][freq][True][round_num]):
                        data = all_data[mode][freq][True][round_num][container]
                        values = [d[metric_index] for d in data]
                        values = remove_zeros(values)
                        with_dc_data.extend(values)
                
                # Add boxes (only if data exists)
                if without_dc_data:
                    box_data.append(without_dc_data)
                    box_positions.append(base_pos - within_group_gap / 2)
                    box_colors.append(color_without_dc)
                
                if with_dc_data:
                    box_data.append(with_dc_data)
                    box_positions.append(base_pos + box_width + within_group_gap / 2)
                    box_colors.append(color_with_dc)
            
            if not box_data:
                print(f"No valid data for {metric_name} at {freq} for mode {mode}")
                continue
            
            # Create boxplots
            bp = ax.boxplot(box_data, positions=box_positions, widths=box_width,
                            patch_artist=True, showfliers=True,
                            boxprops=dict(linewidth=1.5),
                            medianprops=dict(linewidth=2, color='black'),
                            whiskerprops=dict(linewidth=1.5),
                            capprops=dict(linewidth=1.5),
                            flierprops=dict(marker='o', markersize=3, alpha=0.5))
            
            # Color the boxes
            for patch, color in zip(bp['boxes'], box_colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.8)
            
            # Set x-axis
            ax.set_xticks(tick_positions)
            ax.set_xticklabels(tick_labels, rotation=25, ha='right', fontsize=10)
            
            # Labels and title
            ax.set_xlabel('Container', fontsize=11, fontweight='bold')
            if mode_idx == 0:
                ax.set_ylabel(metric_name, fontsize=12, fontweight='bold')
            ax.set_title(f'{mode.capitalize()}', fontsize=13, fontweight='bold')
            ax.grid(axis='y', alpha=0.3, linestyle='--')
        
        # Create a single legend for the entire figure
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=color_without_dc, alpha=0.8, label='w/o DC', edgecolor='black'),
            Patch(facecolor=color_with_dc, alpha=0.8, label='with DC', edgecolor='black'),
        ]
        
        fig.legend(handles=legend_elements, loc='upper center', fontsize=11, ncol=2, 
                   bbox_to_anchor=(0.5, 0.98), frameon=True)
        
        # Add overall title
        fig.suptitle(f'{metric_name} at {freq}', fontsize=15, fontweight='bold', y=1.02)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(f'{metric_name.replace(" ", "_").replace("(", "").replace(")", "")}_{freq}_dc_boxplot.png', 
                    dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Saved {metric_name} {freq} boxplot")

def main():
    # Define root folder and target subfolders
    root_folder = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data"
    
    target_subfolders = [
        'monitor_30hz_r1', 'monitor_30hz_r2', 'monitor_30hz_r3',
        'monitor_60hz_r1', 'monitor_60hz_r2', 'monitor_60hz_r3',
        'monitor_wDC_30hz_r1', 'monitor_wDC_30hz_r2', 'monitor_wDC_30hz_r3',
        'monitor_wDC_60hz_r1', 'monitor_wDC_60hz_r2', 'monitor_wDC_60hz_r3'
    ]
    
    print("Collecting data...")
    all_data = collect_data(root_folder, target_subfolders)
    
    print("\nGenerating CPU usage boxplots...")
    create_grouped_boxplots(all_data, 'CPU Percent', 1)
    
    print("\nGenerating Memory usage boxplots...")
    create_grouped_boxplots(all_data, 'Memory Process (MiB)', 2)
    
    print("\nVisualization complete! Check the output folder for generated plots.")

if __name__ == '__main__':
    main()