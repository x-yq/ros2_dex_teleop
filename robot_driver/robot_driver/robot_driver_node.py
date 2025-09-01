import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.xarm5_driver import XArm5Driver
from rclpy.executors import MultiThreadedExecutor

@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node", config_name="robot_driver")
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

        # Instantiate the driver using Hydra
        robot_driver = XArm5Driver(
            layer_cn_id=cfg.layer_cn_id,
            simulated=cfg.simulated,  # Set to False for physical robot
            ros2_domain_id=cfg.get("ros2_domain_id", None),
        )

        robot_driver.logging("XArm5Driver node started via Hydra", "INFO")
        
        # Spin the node
        # rclpy.spin(robot_driver)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(robot_driver)
        executor.spin()
        
    except KeyboardInterrupt:
        if 'robot_driver' in locals():
            robot_driver.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'robot_driver' in locals():
            robot_driver.handle_error(f"Unexpected error: {str(e)}")
        else:
            print(f"Error creating node: {e}")
    finally:
        robot_driver.urgent_stop()
        robot_driver.destroy_node()
        rclpy.shutdown()

# def main(args=None):
#     """Main entry point for SystemManager node"""
#     rclpy.init(args=args)
    
#     try:
#         node = XArm5Driver()
#         node.get_logger().info("robot_driver node initialized, spinning...")
#         rclpy.spin(node)
        
#     except KeyboardInterrupt:
#         node.get_logger().info("SystemManager node interrupted by user")
#     except Exception as e:
#         node.get_logger().error(f"SystemManager node error: {e}")
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()
#         print("SystemManager node shutdown complete")

# if __name__ == '__main__':
#     main()