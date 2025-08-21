
import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.motion_retarget_ot import MotionRetargetOT

@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node", config_name="motion_retarget")
def main(cfg: DictConfig):

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:
        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # Instantiate the driver using Hydra
        motion_retarget = MotionRetargetOT(
            layer_cn_id=cfg.layer_cn_id,
            ros2_domain_id=cfg.get("ros2_domain_id", None)
        )

        motion_retarget.logging("MultiFeedbackDataProcessor node started via Hydra", "INFO")
        
        # Spin the node
        rclpy.spin(motion_retarget)
        
    except KeyboardInterrupt:
        if 'motion_retarget' in locals():
            motion_retarget.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        if 'motion_retarget' in locals():
            motion_retarget.logging(f"Unexpected error: {str(e)}", "ERROR")
        else:
            print(f"Error creating node: {e}")
    finally:
        motion_retarget.urgent_stop()
        motion_retarget.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()