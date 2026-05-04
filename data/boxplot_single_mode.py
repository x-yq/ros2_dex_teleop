import os
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(filepath):
    """Parse a single log file and extract time series data, excluding first and last 5 seconds."""
    data = []
    container_name = None
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        if 'container_name:' in line:
            # Extract container name
            container_name = line.split('container_name:')[1].strip()
        elif line.strip() and not line.startswith('monitor_') and ',' in line:
            # Parse data lines
            try:
                parts = line.strip().split(',')
                if len(parts) == 5 and parts[0].replace('.', '').isdigit():
                    timestamp, cpu, mem_total, mem_process, cache = parts
                    data.append({
                        'timestamp': float(timestamp),
                        'cpu_percent': float(cpu),
                        'mem_total_mib': float(mem_total),
                        'mem_process_mib': float(mem_process),
                        'cache_mib': float(cache)
                    })
            except (ValueError, IndexError):
                continue
    
    df = pd.DataFrame(data)
    
    # Filter out first and last 5 seconds
    if not df.empty and len(df) > 0:
        min_timestamp = df['timestamp'].min()
        max_timestamp = df['timestamp'].max()
        
        # Keep only data between (min + 5) and (max - 5) seconds
        df = df[(df['timestamp'] >= min_timestamp + 5) & 
                (df['timestamp'] <= max_timestamp - 5)]
    
    return df, container_name

def clean_container_name(name):
    """Remove monitor, ubuntu, teleop strings from container name."""
    name = name.replace('monitor_', '').replace('ubuntu_', '').replace('teleop_', '')
    name = name.replace('monitor-', '').replace('ubuntu-', '').replace('teleop-', '')
    name = name.replace('config-', '').replace('files-', '').replace('-1', '')
    name = name.replace('config_', '').replace('files_', '').replace('_1', '')
    name = name.replace('components', 'algorithm')

    name = name.replace('-', ' ').replace('_', ' ')
    return name.strip()

def load_all_data(data_folder, subfolders):
    """Load data from all subfolders."""
    all_data = []
    
    for subfolder in subfolders:
        folder_path = os.path.join(data_folder, subfolder)
        if not os.path.exists(folder_path):
            print(f"Warning: Folder {folder_path} does not exist, skipping...")
            continue
        
        # Determine frequency and round
        if 'standby' in subfolder:
            frequency = 'standby'
        elif '30hz' in subfolder:
            frequency = '30hz'
        elif '60hz' in subfolder:
            frequency = '60hz'
        else:
            continue
        
        # Extract round number
        round_num = subfolder.split('_r')[-1] if '_r' in subfolder else '1'
        
        # Read all log files in the subfolder
        for filename in os.listdir(folder_path):
            if filename.endswith('.log') or filename.startswith('monitor_'):
                filepath = os.path.join(folder_path, filename)
                df, container_name = parse_log_file(filepath)
                
                if not df.empty and container_name:
                    df['container_name'] = clean_container_name(container_name)
                    df['frequency'] = frequency
                    df['round'] = round_num
                    df['subfolder'] = subfolder
                    all_data.append(df)
    
    if all_data:
        return pd.concat(all_data, ignore_index=True)
    else:
        return pd.DataFrame()

def create_grouped_boxplot(data, metric, ylabel, title, filename):
    """Create a grouped boxplot with proper spacing."""
    # Get unique containers and frequencies
    containers = sorted(data['container_name'].unique())
    frequencies = ['standby', '30hz', '60hz']
    
    # Color scheme
    colors = {'standby': '#3498db', '30hz': '#e74c3c', '60hz': '#2ecc71'}
    
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Set positions for boxes
    n_containers = len(containers)
    n_freqs = len(frequencies)
    
    # Spacing parameters
    box_width = 0.25
    group_spacing = 1.0  # Space between container groups
    within_group_spacing = 0.3  # Space between boxes within a group
    
    positions = []
    box_data = []
    box_colors = []
    labels = []
    
    for i, container in enumerate(containers):
        base_pos = i * (n_freqs * within_group_spacing + group_spacing)
        
        for j, freq in enumerate(frequencies):
            # Filter data for this container and frequency
            filtered = data[(data['container_name'] == container) & 
                          (data['frequency'] == freq)][metric]
            
            if len(filtered) > 0:
                pos = base_pos + j * within_group_spacing
                positions.append(pos)
                box_data.append(filtered.values)
                box_colors.append(colors[freq])
                labels.append(f"{container}\n{freq}")
    
    # Create boxplot
    bp = ax.boxplot(box_data, positions=positions, widths=box_width,
                    patch_artist=True, showfliers=True,
                    boxprops=dict(linewidth=1.5),
                    medianprops=dict(color='black', linewidth=2),
                    whiskerprops=dict(linewidth=1.5),
                    capprops=dict(linewidth=1.5))
    
    # Color the boxes
    for patch, color in zip(bp['boxes'], box_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    # Set x-axis labels (container names)
    container_positions = []
    for i, container in enumerate(containers):
        base_pos = i * (n_freqs * within_group_spacing + group_spacing)
        center_pos = base_pos + (n_freqs - 1) * within_group_spacing / 2
        container_positions.append(center_pos)
    
    ax.set_xticks(container_positions)
    ax.set_xticklabels(containers, fontsize=11, fontweight='bold')
    
    # Labels and title
    ax.set_ylabel(ylabel, fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    
    # Legend
    legend_elements = [plt.Rectangle((0, 0), 1, 1, facecolor=colors[freq], 
                                    alpha=0.7, label=freq) 
                      for freq in frequencies]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10,
             title='Frequency', title_fontsize=11)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved plot: {filename}")
    plt.close()

def main():
    # Configuration
    mode = "distributed2"
    data_folder = f'/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/{mode}/resource_usage_final'  # Change this to your data folder path
    subfolders = [
        'monitor_30hz_r1', 'monitor_30hz_r2', 'monitor_30hz_r3',
        'monitor_60hz_r1', 'monitor_60hz_r2', 'monitor_60hz_r3',
        'monitor_standby_r1', 'monitor_standby_r2', 'monitor_standby_r3'
    ]
    
    # Load all data
    print("Loading data...")
    df = load_all_data(data_folder, subfolders)
    
    if df.empty:
        print("No data loaded. Please check your data folder and file paths.")
        return
    
    print(f"Loaded {len(df)} data points")
    print(f"Containers found: {df['container_name'].unique()}")
    print(f"Frequencies: {df['frequency'].unique()}")
    
    # Create visualizations
    print("\nCreating visualizations...")
    
    # CPU percent boxplot
    create_grouped_boxplot(
        df, 
        'cpu_percent', 
        'CPU Usage (%)', 
        f'CPU Usage by Container and Frequency of Mode {mode.capitalize()}',
        f'{data_folder}/cpu_usage_boxplot_{mode}.png'
    )
    
    # Memory process boxplot
    create_grouped_boxplot(
        df, 
        'mem_process_mib', 
        'Memory Usage (MiB)', 
        f'Memory Usage by Container and Frequency of Mode {mode.capitalize()}',
        f'{data_folder}/memory_usage_boxplot_{mode}.png'
    )
    
    print("\nVisualization complete!")
    
    # Print summary statistics
    print("\n" + "="*60)
    print("Summary Statistics:")
    print("="*60)
    for container in sorted(df['container_name'].unique()):
        print(f"\n{container}:")
        for freq in ['standby', '30hz', '60hz']:
            subset = df[(df['container_name'] == container) & 
                       (df['frequency'] == freq)]
            if len(subset) > 0:
                print(f"  {freq}:")
                print(f"    CPU: mean={subset['cpu_percent'].mean():.2f}%, "
                      f"median={subset['cpu_percent'].median():.2f}%")
                print(f"    Memory: mean={subset['mem_process_mib'].mean():.2f} MiB, "
                      f"median={subset['mem_process_mib'].median():.2f} MiB")

if __name__ == "__main__":
    main()
