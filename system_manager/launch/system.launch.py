from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Shared argument
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    config_folder_path = LaunchConfiguration("config_folder_path")

    # Per-node layer_cn_id arguments
    
    input_sensor_driver = LaunchConfiguration("input_sensor_driver")
    input_control_data_processor = LaunchConfiguration("input_control_data_processor")
    motion_retarget = LaunchConfiguration("motion_retarget")
    robot_driver = LaunchConfiguration("robot_driver")
    feedback_sensor_driver = LaunchConfiguration("feedback_sensor_driver")
    feedback_data_processor = LaunchConfiguration("feedback_data_processor")
    feedback_projector_driver = LaunchConfiguration("feedback_projector_driver")
    kpi_monitor = LaunchConfiguration("kpi_monitor")

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("ros_domain_id", default_value="0", description="ROS 2 Domain ID"),

        DeclareLaunchArgument("config_folder_path", default_value="/root/dev_ws/src/config_files"),

        DeclareLaunchArgument("input_sensor_driver", default_value="input_sensor_driver_rococo"),
        DeclareLaunchArgument("input_control_data_processor", default_value="input_control_data_processor"),
        DeclareLaunchArgument("motion_retarget", default_value="motion_retarget"),
        DeclareLaunchArgument("robot_driver", default_value="robot_driver"),
        DeclareLaunchArgument("feedback_sensor_driver", default_value="feedback_sensor_driver_realsense"),
        DeclareLaunchArgument("feedback_data_processor", default_value="feedback_data_processor"),
        DeclareLaunchArgument("feedback_projector_driver", default_value="feedback_projector_driver"),
        DeclareLaunchArgument("kpi_monitor", default_value="monitor_0"),

        # Set ROS domain
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),

        # === Nodes ===
        Node(
            package="system_manager",
            executable="system_manager_node",
            name="system_manager",
            output="screen",
            arguments=["--config_folder_path", config_folder_path, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="input_sensor_driver",
            executable="input_sensor_driver_node",
            name="input_sensor_driver",
            output="screen",
            arguments=["--layer_cn_id", input_sensor_driver, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="input_data_processor",
            executable="input_data_processor_node",
            name="input_data_processor",
            output="screen",
            arguments=["--layer_cn_id", input_control_data_processor, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="motion_retarget",
            executable="motion_retarget_node",
            name="motion_retarget",
            output="screen",
            arguments=["--layer_cn_id", motion_retarget, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="robot_driver",
            executable="robot_driver_node",
            name="robot_driver",
            output="screen",
            arguments=["--layer_cn_id", robot_driver, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_sensor_driver",
            executable="feedback_sensor_driver_node",
            name="feedback_sensor_driver",
            output="screen",
            arguments=["--layer_cn_id", feedback_sensor_driver, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_data_processor",
            executable="feedback_data_processor_node",
            name="feedback_data_processor",
            output="screen",
            arguments=["--layer_cn_id", feedback_data_processor, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_projector_driver",
            executable="feedback_projector_driver_node",
            name="feedback_projector_driver",
            output="screen",
            arguments=["--layer_cn_id", feedback_projector_driver, "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="kpi_monitor",
            executable="kpi_monitor_node",
            name="monitor_0",
            output="screen",
            arguments=["--layer_cn_id", "monitor_0", "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="kpi_monitor",
            executable="kpi_monitor_node",
            name="monitor_1",
            output="screen",
            arguments=["--layer_cn_id", "monitor_1", "--ros2_domain_id", ros_domain_id]
        ),
    ])