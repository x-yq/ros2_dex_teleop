# kpi_monitor/kpi_monitor/monitor_interface_node.py

import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate



@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node/robot_driver.yaml", config_name="robot_driver")
def main(cfg: DictConfig):
    """
    Main entrypoint for XArm5Driver Node.
    Loads plugin config via Hydra and instantiates the selected driver class.
    """

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:
        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # Instantiate the driver using Hydra (with simulated=True by default)
        robot_driver = instantiate(
            plugin_cfg,
            layer_cn_id=cfg.layer_cn_id,
            simulated=True,  # Set to False for physical robot
            ros2_domain_id=cfg.get("ros2_domain_id", None),
        )

        robot_driver.logging("XArm5Driver node started via Hydra", "INFO")
        
        # Spin the node
        rclpy.spin(robot_driver)
        
    except KeyboardInterrupt:
        if 'robot_driver' in locals():
            robot_driver.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'robot_driver' in locals():
            robot_driver.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        if 'robot_driver' in locals():
            robot_driver.urgent_stop()
            robot_driver.destroy_node()
        rclpy.shutdown()