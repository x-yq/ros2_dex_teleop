import os
import tempfile
import time

import numpy as np

import robodm
import robodm
import numpy as np
import matplotlib.pyplot as plt

def collect_data_from_dict(path):
    data_dict = {
        "observation/image": [
            np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
            for _ in range(10)
        ],
        "observation/state": [
            np.random.rand(10).astype(np.float32)
            for _ in range(10)
        ],
        "action": [
            np.random.rand(7).astype(np.float32)
            for _ in range(10)
        ]

    }

    fps_dict={
        "observation/image":30,
        "observation/state":10,
        "action":10
    }


    traj = robodm.Trajectory(path=path, mode="w")

    traj.from_dict_of_lists(data=data_dict,path=path,fps=fps_dict)
    traj.close()


def collect_data_diff_freq(path):
    traj = robodm.Trajectory(path=path, mode="w")

    # attrs = vars(traj)
    # print(attrs)

    # Add some data
    for i in range(10):
        traj.add("observation/image",np.full((224, 224, 3), i, dtype=np.uint8))
        traj.add("observation/image",np.full((224, 224, 3), i, dtype=np.uint8))
        traj.add("observation/image",np.full((224, 224, 3), i, dtype=np.uint8))


    for i in range(30):
        traj.add("observation/state", np.full(10, i, dtype=np.float32))
        
    for i in range(30):
        traj.add("action", np.full(10, i, dtype=np.float32)
)

    # Close the trajectory
    traj.close()
    print(f"Trajectory saved to {path}")


def load_data(path):
    # Load the trajectory
    traj = robodm.Trajectory(path=path, mode="r")

    ## desired_frequency = 10 == None in this case
    ## desired_frequency = 1 := each key only include one time serie
    data = traj.load(desired_frequency=None)
    # data = traj.load()

    print(f"Data structure: {data.keys()}")
    for k,v in data.items():
        print(f"{k} shape: {v.shape}")
        print("----------example------------")
        print(v[:2])

def collect_data_with_initial_stream_test():

    # pre-define feature streams
    feature_spec = {
        "camera_front": robodm.FeatureType(shape=(480, 640, 3), dtype="uint8"),
        "lidar_points": robodm.FeatureType(shape=(100, 3), dtype="float32")
    }

    path = "/Users/x-yq/Desktop/MasterThesis/datasets/roboDm/data_collection/data.mkv"

    traj = robodm.Trajectory(path, mode="w", video_codec="libx265",
                        codec_options={
                            "crf": "23",        # Quality setting (lower = higher quality)
                            "preset": "fast"    # Encoding speed
                        })
    traj.init_feature_streams(feature_spec)

    # 后续添加数据时无需检查/创建流
    for i in range(10):
        traj.add("camera_front", np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8))
        traj.add("lidar_points", np.random.rand(100, 3).astype(np.float32))
        # Close the trajectory
    traj.close()

TRAJ_PATH = "/root/dev_ws/data/demos/demo_003/demo_003.vla"

FEATURE_CMD = "robot_control_cmd/pose"
FEATURE_STATE = "robot_state/end_effector_pose"


def load_feature(traj, feature_name):
    """
    Load timestamps and values for a single feature.
    Returns:
        t: (N,) timestamps starting from 0 (seconds)
        xyz: (N, 3) translation
    """
    timestamps = []
    values = []

    for sample in traj.iter_feature(feature_name):
        timestamps.append(sample.timestamp)
        values.append(sample.value)

    timestamps = np.asarray(timestamps, dtype=np.float64)
    values = np.asarray(values)

    # robodm timestamps are in milliseconds → convert to seconds
    timestamps_sec = timestamps / 1e3

    # normalize to start from 0
    timestamps_sec -= timestamps_sec[0]

    # translation = x, y, z
    xyz = values[:, :3]

    return timestamps_sec, xyz


def main():
    RESAMPLE_FREQ = 57.0
    traj = robodm.Trajectory(path=TRAJ_PATH, mode="r")

    # Load all data as numpy arrays
    data = traj.load(return_type="numpy", desired_frequency=RESAMPLE_FREQ)

    cmd = data[FEATURE_CMD]      # (N, 7)
    state = data[FEATURE_STATE]  # (M, 7)

    # Time axes (start from 0)
    t_cmd = np.arange(cmd.shape[0]) / RESAMPLE_FREQ
    t_state = np.arange(state.shape[0]) / RESAMPLE_FREQ

    labels = ["x", "y", "z"]

    for i in range(3):
        plt.figure(figsize=(10, 4))

        plt.plot(
            t_cmd,
            cmd[:, i],
            label=f"{FEATURE_CMD} ({labels[i]})",
            linewidth=2
        )

        plt.plot(
            t_state,
            state[:, i],
            label=f"{FEATURE_STATE} ({labels[i]})",
            linestyle="--",
            linewidth=2
        )

        plt.xlabel("Time (s)")
        plt.ylabel(f"Translation {labels[i]} (m)")
        plt.title(f"Resampled Translation {labels[i]} @ {RESAMPLE_FREQ} Hz")
        plt.legend()
        plt.grid(True)

        out_path = f"translation_{labels[i]}_{int(RESAMPLE_FREQ)}Hz.png"
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close()

        print(f"Saved {out_path}")



if __name__ == "__main__":

    # path = '/Users/x-yq/Desktop/MasterThesis/datasets/roboDm/data_collection/test_trajectory.vla'
    # path = os.path.join(path, "test_trajectory.vla")
    path = '/root/dev_ws/data/demos/demo_003/demo_003.vla'

    # collect_data_diff_freq(path)
    load_data(path)

    main()
    # collect_data_diff_freq('/root/dev_ws/data/demos/demo_001/demo_002.vla')

    # Clean up
    # os.remove(path)
