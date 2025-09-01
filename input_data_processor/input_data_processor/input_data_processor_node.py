
import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.mocap_data_processor import RococoHandKeypointProcessor
from rclpy.executors import MultiThreadedExecutor

@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node", config_name="input_control_data_processor")
def main(cfg: DictConfig):

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:
        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # Instantiate the driver using Hydra
        input_data_processor = RococoHandKeypointProcessor(
            layer_cn_id=cfg.layer_cn_id,
            ros2_domain_id=cfg.get("ros2_domain_id", None),
        )

        input_data_processor.logging("RococoHandKeypointProcessor node started via Hydra", "INFO")
        
        # Spin the node
        # rclpy.spin(input_data_processor)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(input_data_processor)
        executor.spin()
        
    except KeyboardInterrupt:
        if 'input_data_processor' in locals():
            input_data_processor.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'input_data_processor' in locals():
            input_data_processor.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        input_data_processor.urgent_stop()
        input_data_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()