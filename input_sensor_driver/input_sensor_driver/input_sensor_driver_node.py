
import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate


@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node/input_sensor_driver_rococo.yaml", config_name="input_sensor_driver_rococo")
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

        # Instantiate the driver using Hydra
        input_sensor_driver = instantiate(
            plugin_cfg,
            layer_cn_id=cfg.layer_cn_id,
            ros2_domain_id=cfg.get("ros2_domain_id", None),
        )

        input_sensor_driver.logging("RococoDriver node started via Hydra", "INFO")
        
        # Spin the node
        rclpy.spin(input_sensor_driver)
        
    except KeyboardInterrupt:
        if 'input_sensor_driver' in locals():
            input_sensor_driver.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'input_sensor_driver' in locals():
            input_sensor_driver.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        if 'input_sensor_driver' in locals():
            input_sensor_driver.urgent_stop()
            input_sensor_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
