import rclpy
from .data_collector_cls_dm import DataCollector
# from data_collector_cls_log import DataCollector

from rclpy.executors import MultiThreadedExecutor
import sys

def main(args=None):
    rclpy.init(args=args)

    layer_cn_id = 'data_collector'
    ros2_domain_id = 0

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

    # Create node instance
    node = DataCollector(layer_cn_id=layer_cn_id, ros2_domain_id=ros2_domain_id)

    try:
        node.logging(f"DataCollection node started with layer_cn_id: {layer_cn_id}", "INFO")

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

if __name__ == '__main__':
    main()