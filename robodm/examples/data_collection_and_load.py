import os
import tempfile
import time

import numpy as np

import robodm

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
        print(v)

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

if __name__ == "__main__":

    path = '/Users/x-yq/Desktop/MasterThesis/datasets/roboDm/data_collection/test_trajectory.vla'
    # path = os.path.join(path, "test_trajectory.vla")

    collect_data_diff_freq(path)
    load_data(path)

    # Clean up
    # os.remove(path)
