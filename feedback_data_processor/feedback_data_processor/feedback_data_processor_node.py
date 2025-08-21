
import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.multi_feedback_data_processor import MultiFeedbackDataProcessor

@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node", config_name="feedback_data_processor")
def main(cfg: DictConfig):

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:
        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # Instantiate the driver using Hydra
        feedback_data_processor = MultiFeedbackDataProcessor(
            layer_cn_id=cfg.layer_cn_id,
            ros2_domain_id=cfg.get("ros2_domain_id", None)
        )

        feedback_data_processor.logging("MultiFeedbackDataProcessor node started via Hydra", "INFO")
        
        # Spin the node
        rclpy.spin(feedback_data_processor)
        
    except KeyboardInterrupt:
        if 'feedback_data_processor' in locals():
            feedback_data_processor.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'feedback_data_processor' in locals():
            feedback_data_processor.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        feedback_data_processor.urgent_stop()
        feedback_data_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()