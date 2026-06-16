import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import re


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
    # Configuration, use no monitor and no latency
    h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/centered/motion_acc_latency/HZ_center_noLatency_noMonitor_30/HZ_center_noLatency_30_buffer.h5"
    # h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/centered/motion_acc_latency/HZ_center_noLatency_noMonitor_60/HZ_center_noLatency_60_buffer.h5"
    # h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/distributed1/motion_acc_latency/HZ_d1_noLatency_noMonitor_60/HZ_d1_noLatency_noMonitor_60_r2_buffer.h5"
    # h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/distributed1/motion_acc_latency/HZ_d1_noLatency_noMonitor_30/HZ_d1_noLatency_noMonitor_30_buffer.h5"
    # h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/distributed2/motion_acc_latency/HZ_d2_noLatency_noMonitor_30/HZ_d2_noLatency_noMonitor_30_buffer.h5"
    # h5_path = "/Users/x-yq/Desktop/MasterThesis/teleop_dev/data/distributed2/motion_acc_latency/HZ_d2_noLatency_noMonitor_60/HZ_d2_noLatency_noMonitor_60_buffer.h5"
    # Inspect structure
    inspect_hdf5_structure(h5_path)
    
    # ===== Timing quality =====
    ts_cmd, _ = load_hdf5_data(h5_path, "robot_control_cmd/pose")
    ts_state, _ = load_hdf5_data(h5_path, "robot_state/end_effector_pose")
    
    evaluate_timing_quality(
	    ts_state,
	    expected_freq=90,
	    name="Robot State"
	)

    HZ_int = int(h5_path.split("/")[9].split("_")[-1])
    evaluate_timing_quality(
	    ts_cmd,
	    expected_freq=HZ_int,
	    name="Robot Command"
	)

    # ===== Cross-topic alignment =====
    evaluate_cross_topic_alignment(
	    ts_state,
	    ts_cmd,
	    name="Robot State vs Command"
	)
