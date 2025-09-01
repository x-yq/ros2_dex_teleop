import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.render_pc_driver import RenderPCDriver
from rclpy.executors import MultiThreadedExecutor


@hydra.main(version_base='1.2', config_path="/root/dev_ws/src/config_files/component_node", config_name="feedback_projector_driver")
def main(cfg: DictConfig):
    """
    Main entrypoint for Node.
    Loads plugin config via Hydra and instantiates the selected monitor class.
    """

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:
        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # # Instantiate the driver using Hydra
        # feedback_projector_driver = instantiate(
        #     plugin_cfg,
        #     layer_cn_id=cfg.layer_cn_id,
        #     ros2_domain_id=cfg.get("ros2_domain_id", None),
        # )
        feedback_projector_driver = RenderPCDriver(layer_cn_id=cfg.layer_cn_id, ros2_domain_id=cfg.get("ros2_domain_id", None))

        feedback_projector_driver.logging("RenderPCProjecterDriver node started via Hydra", "INFO")
        
        # Spin the node
        # rclpy.spin(feedback_projector_driver)
                
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(feedback_projector_driver)
        executor.spin()
        
        
    except KeyboardInterrupt:
        if 'feedback_projector_driver' in locals():
            feedback_projector_driver.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'feedback_projector_driver' in locals():
            feedback_projector_driver.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        feedback_projector_driver.urgent_stop()
        feedback_projector_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()