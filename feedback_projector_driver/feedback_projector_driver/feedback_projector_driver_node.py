import rclpy
from rclpy.executors import MultiThreadedExecutor
from .plugins.render_pc_driver import RenderPCDriver
from .plugins.vr_driver import VRDriver
import sys


def main(args=None):
    rclpy.init(args=args)
    
    if args:
        # Parse custom arguments
        for i, arg in enumerate(args):
            if arg == "--layer_cn_id" and i + 1 < len(args):
                layer_cn_id = args[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(args):
                ros2_domain_id = int(args[i + 1])
    else:
        # If no args provided, try to get from sys.argv
        for i, arg in enumerate(sys.argv):
            if arg == "--layer_cn_id" and i + 1 < len(sys.argv):
                layer_cn_id = sys.argv[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(sys.argv):
                ros2_domain_id = int(sys.argv[i + 1])


    # These parameters will be set via the launch file
    node = VRDriver(
        layer_cn_id=layer_cn_id,          # to be overridden by launch
        ros2_domain_id=ros2_domain_id        # to be overridden by launch
    )

    try:
        node.logging(f"VRDriver node started with layer_cn_id: {layer_cn_id}", "INFO")

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        node.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        node.logging(f"Unexpected error: {str(e)}", "ERROR")
    finally:
        node.urgent_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
