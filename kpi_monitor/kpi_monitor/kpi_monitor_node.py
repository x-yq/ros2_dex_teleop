# kpi_monitor/kpi_monitor/monitor_interface_node.py

import hydra
from omegaconf import OmegaConf, DictConfig
import rclpy
from hydra.utils import instantiate


@hydra.main(version_base=None, config_path=None, config_name=None)
def main(cfg: DictConfig):
    """
    Main entrypoint for KPI Monitor Node.
    Loads plugin config via Hydra and instantiates the selected monitor class.
    """

    # If user provides external config file
    if "config_path" in cfg and cfg.config_path:
        cfg = OmegaConf.load(cfg.config_path)

    rclpy.init()

    # Pick default plugin from cfg.plugins
    plugin_cfg = next(p for p in cfg.plugins if p.id == cfg.default_plugin)

    # NOTE: KPIMonitorBase constructor signature changed:
    #   (layer_cn_id: str, ros2_domain_id: Optional[int] = None)
    monitor = instantiate(
        plugin_cfg,
        layer_cn_id=cfg.layer_cn_id,
        ros2_domain_id=cfg.get("ros2_domain_id", None),
    )

    rclpy.spin(monitor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
