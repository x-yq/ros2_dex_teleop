import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import re

def extract_translation_from_pose(pose_data):
    """
    Extract translation (x, y, z) from pose data.
    Assumes pose data is in format [x, y, z, qx, qy, qz, qw] (7D).
    """
    if pose_data.ndim == 2:
        return pose_data[:, :3]
    else:
        return pose_data[:3]

def load_hdf5_data(h5_path, feature_name):
    """
    Load timestamps and data from HDF5 file for a given feature.
    """
    with h5py.File(h5_path, "r") as h5f:
        group = h5f
        for part in feature_name.split("/"):
            if part in group:
                group = group[part]
            else:
                raise KeyError(f"Feature {feature_name} not found in HDF5 file")
        
        timestamps = group["timestamps"][:]
        data = group["data"][:]
    
    print(f"length of data of {feature_name} = {len(timestamps)}")
    return timestamps, data

def parse_latency_log(log_path):
    """
    Parse latency log file to extract RTT and timestamps.
    
    Returns:
        rtt_values: array of RTT values in milliseconds
        timestamps: array of start timestamps in milliseconds
    """
    rtt_values = []
    timestamps = []
    
    try:
        with open(log_path, 'r') as f:
            for line in f:
                # Parse: RTT = X.XXX ms; Start Timestamp = XXXXXXXXX.XXXX ms
                rtt_match = re.search(r'RTT = ([\d.]+) ms', line)
                ts_match = re.search(r'Start Timestamp = ([\d.]+) ms', line)
                
                if rtt_match and ts_match:
                    rtt_values.append(float(rtt_match.group(1)))
                    timestamps.append(float(ts_match.group(1)))
        
        rtt_values = np.array(rtt_values)
        timestamps = np.array(timestamps)
        
        print(f"✓ Loaded {len(rtt_values)} RTT samples from {Path(log_path).name}")
        print(f"  RTT range: {rtt_values.min():.2f} - {rtt_values.max():.2f} ms")
        print(f"  RTT mean: {rtt_values.mean():.2f} ms, std: {rtt_values.std():.2f} ms")
        
        return rtt_values, timestamps
        
    except FileNotFoundError:
        print(f"✗ Latency log not found: {log_path}")
        return np.array([]), np.array([])
    except Exception as e:
        print(f"✗ Error parsing latency log: {e}")
        return np.array([]), np.array([])

def plot_robot_and_gripper_with_latency(h5_path, robot_latency_path, gripper_latency_path, 
                                        demo_id=None, save_path=None):
    """
    Plot robot pose (translation) and gripper joint angles with RTT data overlaid.
    Command and state are plotted on the same figure for comparison.
    RTT is shown on secondary y-axis.
    
    Args:
        h5_path: Path to HDF5 buffer file
        robot_latency_path: Path to robot latency log file
        gripper_latency_path: Path to gripper latency log file
        demo_id: Demo ID for title (optional)
        save_path: Path to save figure (optional)
    """
    fig, axes = plt.subplots(2, 1, figsize=(16, 10))
    
    # Get reference timestamp from first data point
    try:
        timestamps_ref, _ = load_hdf5_data(h5_path, "robot_control_cmd/pose")
        reference_time_ns = timestamps_ref[0]
        reference_time_ms = reference_time_ns / 1e6
    except:
        reference_time_ms = 0
    
    # ========== Plot 1: Robot End-Effector Translation with RTT ==========
    ax_robot = axes[0]
    ax_robot_rtt = ax_robot.twinx()

    # Robot state
    try:
        timestamps_state, data_state = load_hdf5_data(h5_path, "robot_state/end_effector_pose")
        time_state = (timestamps_state - timestamps_state[0]) / 1e9
        translation_state = extract_translation_from_pose(data_state)
        
        ax_robot.plot(time_state, translation_state[:, 0], 
                     label='State X', linewidth=1.5, linestyle='-', alpha=0.9, color='C0')
        ax_robot.plot(time_state, translation_state[:, 1], 
                     label='State Y', linewidth=1.5, linestyle='-', alpha=0.9, color='C1')
        ax_robot.plot(time_state, translation_state[:, 2], 
                     label='State Z', linewidth=1.5, linestyle='-', alpha=0.9, color='C2')
        
        print(f"✓ Robot state: {len(timestamps_state)} samples, {time_state[-1]:.2f}s")
    except Exception as e:
        print(f"✗ Error loading robot state: {e}")
    
    # Robot control command
    try:
        timestamps_cmd, data_cmd = load_hdf5_data(h5_path, "robot_control_cmd/pose")
        time_cmd = (timestamps_cmd - timestamps_state[0]) / 1e9
        translation_cmd = extract_translation_from_pose(data_cmd)
        
        ax_robot.plot(time_cmd, translation_cmd[:, 0], 
                     label='Command X', linewidth=2, linestyle='--', alpha=0.8, color='C0')
        ax_robot.plot(time_cmd, translation_cmd[:, 1], 
                     label='Command Y', linewidth=2, linestyle='--', alpha=0.8, color='C1')
        ax_robot.plot(time_cmd, translation_cmd[:, 2], 
                     label='Command Z', linewidth=2, linestyle='--', alpha=0.8, color='C2')
        
        print(f"✓ Robot command: {len(timestamps_cmd)} samples, {time_cmd[-1]:.2f}s")
    except Exception as e:
        print(f"✗ Error loading robot command: {e}")
    
    # Robot RTT
    # robot_rtt, robot_ts = parse_latency_log(robot_latency_path)
    # if len(robot_rtt) > 0:
    #     robot_time = (robot_ts - reference_time_ms) / 1000.0  # Convert to seconds
    #     ax_robot_rtt.scatter(robot_time, robot_rtt, 
    #                         label='Robot RTT', s=30, alpha=0.6, marker='o', 
    #                         color='red', edgecolors='darkred', linewidth=0.5)
        
    #     # Add statistics text
    #     stats_text = f"RTT: μ={robot_rtt.mean():.2f}ms, σ={robot_rtt.std():.2f}ms, max={robot_rtt.max():.2f}ms"
    #     ax_robot.text(0.02, 0.98, stats_text, 
    #                  transform=ax_robot.transAxes,
    #                  verticalalignment='top',
    #                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
    #                  fontsize=9, family='monospace')
    robot_rtt, robot_ts = parse_latency_log(robot_latency_path)

    if len(robot_rtt) > 0:
        robot_time = (robot_ts - reference_time_ms) / 1000.0  # seconds

        # filter out robot_time < 0
        valid_mask = robot_time >= 0
        robot_time = robot_time[valid_mask]
        robot_rtt = robot_rtt[valid_mask]

        if len(robot_rtt) > 0:
            ax_robot_rtt.scatter(
                robot_time, robot_rtt,
                label='Robot RTT', s=30, alpha=0.6, marker='o',
                color='red', edgecolors='darkred', linewidth=0.5
            )

            stats_text = (
                f"RTT: μ={robot_rtt.mean():.2f}ms, "
                f"σ={robot_rtt.std():.2f}ms, "
                f"max={robot_rtt.max():.2f}ms"
            )

            ax_robot.text(
                0.02, 0.98, stats_text,
                transform=ax_robot.transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                fontsize=9, family='monospace'
            )
    ax_robot.set_xlabel('Time (s)', fontsize=12)
    ax_robot.set_ylabel('Translation (m)', fontsize=12, color='black')
    ax_robot_rtt.set_ylabel('RTT (ms)', fontsize=12, color='red')
    ax_robot_rtt.tick_params(axis='y', labelcolor='red')
    ax_robot.set_title('Robot End-Effector Position (Command vs State) with RTT', 
                       fontsize=14, fontweight='bold')
    
    # Combine legends
    lines1, labels1 = ax_robot.get_legend_handles_labels()
    lines2, labels2 = ax_robot_rtt.get_legend_handles_labels()
    ax_robot.legend(lines1 + lines2, labels1 + labels2, loc='upper right', ncol=2, fontsize=10)
    ax_robot.grid(True, alpha=0.3)
    
    # ========== Plot 2: Gripper Joint Angles with RTT ==========
    ax_gripper = axes[1]
    ax_gripper_rtt = ax_gripper.twinx()
    
    # Gripper state
    try:
        timestamps_state, data_state = load_hdf5_data(h5_path, "gripper_state/joint_states")
        time_state = (timestamps_state - timestamps_state[0]) / 1e9
        
        if data_state.ndim == 1:
            data_state = data_state.reshape(-1, 1)
        
        for joint_idx in range(data_state.shape[1]):
            ax_gripper.plot(time_state, data_state[:, joint_idx], 
                          label=f'State Joint {joint_idx}', 
                          linewidth=1.5, linestyle='-', alpha=0.9, color=f'C{joint_idx}')
        
        print(f"✓ Gripper state: {len(timestamps_state)} samples, {data_state.shape[1]} joints")
    except Exception as e:
        print(f"✗ Error loading gripper state: {e}")

        # Gripper control command
    try:
        timestamps_cmd, data_cmd = load_hdf5_data(h5_path, "gripper_control_cmd/joint_angles")
        time_cmd = (timestamps_cmd - timestamps_state[0]) / 1e9
        
        if data_cmd.ndim == 1:
            data_cmd = data_cmd.reshape(-1, 1)
        
        for joint_idx in range(data_cmd.shape[1]):
            ax_gripper.plot(time_cmd, data_cmd[:, joint_idx], 
                          label=f'Command Joint {joint_idx}', 
                          linewidth=2, linestyle='--', alpha=0.8, color=f'C{joint_idx}')
        
        print(f"✓ Gripper command: {len(timestamps_cmd)} samples, {data_cmd.shape[1]} joints")
    except Exception as e:
        print(f"✗ Error loading gripper command: {e}")
    
    # Gripper RTT
    gripper_rtt, gripper_ts = parse_latency_log(gripper_latency_path)
    if len(gripper_rtt) > 0:
        gripper_time = (gripper_ts - reference_time_ms) / 1000.0  # Convert to seconds
        ax_gripper_rtt.scatter(gripper_time, gripper_rtt, 
                              label='Gripper RTT', s=30, alpha=0.6, marker='s', 
                              color='red', edgecolors='darkred', linewidth=0.5)
        
        # Add statistics text
        stats_text = f"RTT: μ={gripper_rtt.mean():.2f}ms, σ={gripper_rtt.std():.2f}ms, max={gripper_rtt.max():.2f}ms"
        ax_gripper.text(0.02, 0.98, stats_text, 
                       transform=ax_gripper.transAxes,
                       verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                       fontsize=9, family='monospace')
    
    ax_gripper.set_xlabel('Time (s)', fontsize=12)
    ax_gripper.set_ylabel('Joint Angle (rad)', fontsize=12, color='black')
    ax_gripper_rtt.set_ylabel('RTT (ms)', fontsize=12, color='red')
    ax_gripper_rtt.tick_params(axis='y', labelcolor='red')
    ax_gripper.set_title('Gripper Joint Angles (Command vs State) with RTT', 
                        fontsize=14, fontweight='bold')
    
    # Combine legends
    lines1, labels1 = ax_gripper.get_legend_handles_labels()
    lines2, labels2 = ax_gripper_rtt.get_legend_handles_labels()
    ax_gripper.legend(lines1 + lines2, labels1 + labels2, loc='upper right', ncol=2, fontsize=10)
    ax_gripper.grid(True, alpha=0.3)
    
    # Main title
    title = f'Trajectory Analysis with Latency - {demo_id}' if demo_id else 'Trajectory Analysis with Latency'
    fig.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()
    
    # Save figure
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n✓ Figure saved to: {save_path}")
    else:
        h5_dir = Path(h5_path).parent
        save_file = h5_dir / f"{demo_id}_trajectory_with_latency.png"
        plt.savefig(save_file, dpi=300, bbox_inches='tight')
        print(f"\n✓ Figure saved to: {save_file}")
    
    plt.close()

def plot_detailed_comparison(h5_path, demo_id=None, save_path=None):
    """
    Create a detailed 3x2 plot showing each axis separately for robot position.
    """
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Load robot data
    try:
        timestamps_cmd, data_cmd = load_hdf5_data(h5_path, "robot_control_cmd/pose")
        time_cmd = (timestamps_cmd - timestamps_cmd[0]) / 1e9
        translation_cmd = extract_translation_from_pose(data_cmd)
        
        timestamps_state, data_state = load_hdf5_data(h5_path, "robot_state/end_effector_pose")
        time_state = (timestamps_state - timestamps_state[0]) / 1e9
        translation_state = extract_translation_from_pose(data_state)
        
        # Plot X, Y, Z separately
        axes_labels = ['X Position', 'Y Position', 'Z Position']
        for i in range(3):
            ax = fig.add_subplot(gs[i, 0])
            ax.plot(time_cmd, translation_cmd[:, i], 
                   label='Command', linewidth=2, linestyle='--', alpha=0.8)
            ax.plot(time_state, translation_state[:, i], 
                   label='State', linewidth=1.5, linestyle='-', alpha=0.9)
            ax.set_xlabel('Time (s)', fontsize=11)
            ax.set_ylabel('Position (m)', fontsize=11)
            ax.set_title(f'Robot {axes_labels[i]}', fontsize=12, fontweight='bold')
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)
            
            # Calculate tracking error
            cmd_interp = np.interp(time_state, time_cmd, translation_cmd[:, i])
            error = translation_state[:, i] - cmd_interp
            
            ax_error = fig.add_subplot(gs[i, 1])
            ax_error.plot(time_state, error * 1000, linewidth=1.5, color='red')
            ax_error.axhline(y=0, color='black', linestyle='--', linewidth=0.8, alpha=0.5)
            ax_error.set_xlabel('Time (s)', fontsize=11)
            ax_error.set_ylabel('Tracking Error (mm)', fontsize=11)
            ax_error.set_title(f'Robot {axes_labels[i]} Error', fontsize=12, fontweight='bold')
            ax_error.grid(True, alpha=0.3)
            
            # Print statistics
            print(f"  {axes_labels[i]} - Mean error: {np.mean(np.abs(error))*1000:.3f} mm, "
                  f"Max error: {np.max(np.abs(error))*1000:.3f} mm")
        
        print(f"✓ Robot detailed comparison completed")
        
    except Exception as e:
        print(f"✗ Error in detailed comparison: {e}")
    
    title = f'Detailed Trajectory Analysis - {demo_id}' if demo_id else 'Detailed Trajectory Analysis'
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n✓ Figure saved to: {save_path}")
    else:
        h5_dir = Path(h5_path).parent
        save_file = h5_dir / f"{demo_id}_detailed_analysis.png"
        plt.savefig(save_file, dpi=300, bbox_inches='tight')
        print(f"\n✓ Figure saved to: {save_file}")
    
    plt.close()
    
def evaluate_timing_quality(timestamps_ns, expected_freq, name="signal"):
    """
    Evaluate timing quality of a timestamp sequence.

    Args:
        timestamps_ns: 1D numpy array, nanoseconds
        expected_freq: expected frequency in Hz
        name: name for logging

    Returns:
        dict with timing metrics
    """
    timestamps_ns = np.asarray(timestamps_ns)

    if len(timestamps_ns) < 2:
        print(f"[{name}] Not enough samples for timing analysis")
        return None

    # convert to seconds
    t = timestamps_ns / 1e9
    dt = np.diff(t)

    expected_dt = 1.0 / expected_freq

    monotonic = np.all(dt > 0)
    backward_count = np.sum(dt <= 0)

    metrics = {
        "num_samples": len(timestamps_ns),
        "duration_s": t[-1] - t[0],
        "mean_dt": np.mean(dt),
        "std_dt": np.std(dt),
        "max_jitter": np.max(np.abs(dt - expected_dt)),
        "expected_dt": expected_dt,
        "monotonic": monotonic,
        "backward_count": backward_count,
    }

    print(f"\n=== Timing Evaluation: {name} ===")
    print(f"Samples        : {metrics['num_samples']}")
    print(f"Duration       : {metrics['duration_s']:.3f} s")
    print(f"Expected Δt    : {expected_dt*1000:.3f} ms")
    print(f"Mean Δt        : {metrics['mean_dt']*1000:.3f} ms")
    print(f"Std(Δt)        : {metrics['std_dt']*1000:.3f} ms")
    print(f"Max |Δt-Δt₀|   : {metrics['max_jitter']*1000:.3f} ms")
    print(f"Monotonic      : {monotonic}")
    if backward_count > 0:
        print(f"⚠ Time went backward {backward_count} times")

    return metrics

def evaluate_cross_topic_alignment(
    timestamps_state_ns,
    timestamps_cmd_ns,
    name="state-command alignment"
):
    """
    Evaluate timestamp alignment between two topics with different frequencies.

    Uses nearest-neighbor matching.

    Args:
        timestamps_state_ns: state timestamps (ns)
        timestamps_cmd_ns: command timestamps (ns)

    Returns:
        dict with alignment metrics
    """
    t_state = np.asarray(timestamps_state_ns) / 1e9
    t_cmd = np.asarray(timestamps_cmd_ns) / 1e9

    if len(t_state) == 0 or len(t_cmd) == 0:
        print(f"[{name}] Empty timestamps")
        return None

    deltas = []

    cmd_idx = 0
    for ts in t_state:
        while cmd_idx + 1 < len(t_cmd) and abs(t_cmd[cmd_idx + 1] - ts) < abs(t_cmd[cmd_idx] - ts):
            cmd_idx += 1
        deltas.append(abs(ts - t_cmd[cmd_idx]))

    deltas = np.asarray(deltas)

    metrics = {
        "num_pairs": len(deltas),
        "mean_abs_dt": np.mean(deltas),
        "std_abs_dt": np.std(deltas),
        "max_abs_dt": np.max(deltas),
        "p95_abs_dt": np.percentile(deltas, 95),
    }

    print(f"\n=== Cross-Topic Alignment: {name} ===")
    print(f"Pairs          : {metrics['num_pairs']}")
    print(f"Mean |Δt|      : {metrics['mean_abs_dt']*1000:.3f} ms")
    print(f"Std  |Δt|      : {metrics['std_abs_dt']*1000:.3f} ms")
    print(f"P95  |Δt|      : {metrics['p95_abs_dt']*1000:.3f} ms")
    print(f"Max  |Δt|      : {metrics['max_abs_dt']*1000:.3f} ms")

    return metrics


def inspect_hdf5_structure(h5_path):
    """
    Print the structure of the HDF5 file to see all available features.
    """
    print(f"\n=== HDF5 File Structure: {h5_path} ===\n")
    
    def print_structure(name, obj):
        indent = "  " * name.count('/')
        if isinstance(obj, h5py.Dataset):
            print(f"{indent}{name}: Dataset, shape={obj.shape}, dtype={obj.dtype}")
        elif isinstance(obj, h5py.Group):
            print(f"{indent}{name}: Group")
    
    with h5py.File(h5_path, "r") as h5f:
        h5f.visititems(print_structure)
    print()

if __name__ == "__main__":
    # Configuration
    h5_path = "/home/x-yq/teleop_dev/data/distributed1/motion_acc_latency/HZ_d1_60/HZ_d1_60_buffer.h5"
    demo_id = "HZ_d1_60"
    robot_latency_path = "/home/x-yq/teleop_dev/data/distributed1/motion_acc_latency/latency_60hz/LATENCY_robot_control_cmd.log"
    gripper_latency_path = "/home/x-yq/teleop_dev/data/distributed1/motion_acc_latency/latency_60hz/LATENCY_gripper_control_cmd.log"
    
    # Inspect structure
    inspect_hdf5_structure(h5_path)
    
    # Main plot: Robot translation + Gripper joints with RTT overlaid
    print("\n=== Creating Main Trajectory Plot with Latency ===")
    plot_robot_and_gripper_with_latency(
        h5_path, 
        robot_latency_path, 
        gripper_latency_path,
        demo_id=demo_id
    )
    
    # Detailed analysis with tracking errors
    print("\n=== Creating Detailed Analysis Plot ===")
    plot_detailed_comparison(h5_path, demo_id=demo_id)
    
    print("\n✓ All plots completed!")
    
    # ===== Timing quality =====
    ts_cmd, _ = load_hdf5_data(h5_path, "robot_control_cmd/pose")
    ts_state, _ = load_hdf5_data(h5_path, "robot_state/end_effector_pose")
    
    evaluate_timing_quality(
	    ts_cmd,
	    expected_freq=60,
	    name="Robot Command"
	)

    evaluate_timing_quality(
	    ts_state,
	    expected_freq=90,
	    name="Robot State"
	)

    # ===== Cross-topic alignment =====
    evaluate_cross_topic_alignment(
	    ts_state,
	    ts_cmd,
	    name="Robot State vs Command"
	)
