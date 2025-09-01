# kpi_monitor/kpi_monitor/kpi_monitor_node.py

import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate
from .plugins.docker_monitor import DockerMonitor
from rclpy.executors import MultiThreadedExecutor


@hydra.main(version_base=None, config_path="/root/dev_ws/src/config_files/component_node", config_name="monitor_0")
def main(cfg: DictConfig):
    """
    Main entrypoint for KPI Monitor Node.
    Loads plugin config via Hydra and instantiates the selected monitor class.
    """

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    try:

        # Pick default plugin from cfg.plugins
        plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

        # NOTE: KPIMonitorBase constructor signature changed:
        #   (layer_cn_id: str, ros2_domain_id: Optional[int] = None)
        monitor = DockerMonitor(
            layer_cn_id=cfg.layer_cn_id,
            ros2_domain_id=cfg.get("ros2_domain_id", None),
        )

        # rclpy.spin(monitor)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(monitor)
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
