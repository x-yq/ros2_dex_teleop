#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction
from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file
from launch.actions import TimerAction

    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    load_controller = LaunchConfiguration('load_controller', default=True)
    show_rviz = LaunchConfiguration('show_rviz', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}
    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')


    if not moveit_config_dict:
        # ros2 control params
        ros2_control_params = generate_ros2_control_params_temp_file(
            os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
            prefix=prefix.perform(context), 
            add_gripper=add_gripper.perform(context) in ('True', 'true'),
            add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
            ros_namespace=ros_namespace,
            update_rate=1000,
            use_sim_time=True,
            robot_type=robot_type.perform(context)
        )
        # robot_description
        robot_description = {
            'robot_description': get_xacro_content(
                context,
                xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro', 
                dof=dof,
                robot_type=robot_type,
                prefix=prefix,
                hw_ns=hw_ns,
                limited=limited,
                effort_control=effort_control,
                velocity_control=velocity_control,
                model1300=model1300,
                robot_sn=robot_sn,
                attach_to=attach_to,
                attach_xyz=attach_xyz,
                attach_rpy=attach_rpy,
                kinematics_suffix=kinematics_suffix,
                ros2_control_plugin=ros2_control_plugin,
                ros2_control_params=ros2_control_params,
                add_gripper=add_gripper,
                add_vacuum_gripper=add_vacuum_gripper,
                add_bio_gripper=add_bio_gripper,
                add_realsense_d435i=add_realsense_d435i,
                add_d435i_links=add_d435i_links,
                add_other_geometry=add_other_geometry,
                geometry_type=geometry_type,
                geometry_mass=geometry_mass,
                geometry_height=geometry_height,
                geometry_radius=geometry_radius,
                geometry_length=geometry_length,
                geometry_width=geometry_width,
                geometry_mesh_filename=geometry_mesh_filename,
                geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
                geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
                geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
                geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
            )
        }
        moveit_config_dict = robot_description
    else:
        robot_description = {'robot_description': moveit_config_dict['robot_description']}

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': xarm_gazebo_world,
            'server_required': 'true',
            'gui_required': gazebo_gui,
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            # '-entity', '{}'.format(xarm_type),
            '-entity', 'UF_ROBOT',
            '-x', '-0.2',
            '-y', '-0.54' if robot_type.perform(context) == 'uf850' else '-0.5',
            '-z', '1.021',
            '-Y', '1.571',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Camera spawn
    camera_model_path = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'sim_camera.sdf'])
    gazebo_spawn_camera_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'sim_camera',
            '-file', camera_model_path,
        ],
        output='screen',
        shell=False  # Set to True if you encounter command parsing issues
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict.get('robot_description', ''),
                'robot_description_semantic': moveit_config_dict.get('robot_description_semantic', ''),
                'robot_description_kinematics': moveit_config_dict.get('robot_description_kinematics', {}),
                'robot_description_planning': moveit_config_dict.get('robot_description_planning', {}),
                'planning_pipelines': moveit_config_dict.get('planning_pipelines', {}),
                'use_sim_time': True
            }
        ],
        # condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        '{}{}_cartesian_motion_controller'.format(prefix.perform(context), xarm_type),
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
        # 'cartesian_impedance_controller',
        # 'pose_broadcaster',
        # 'gravity_compensation',
        # 'joint_impedance_controller',
    ]
    if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    
    controller_nodes = []
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controllers:
            arguments = [
                controller,
                '--controller-manager', f'{ros_namespace}/controller_manager',
            ]

            # Make xarm5_traj_controller inactive
            if controller == 'xarm5_traj_controller':
                arguments.append('--inactive')

            controller_nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=arguments,
                parameters=[{'use_sim_time': True}],
            ))

    if len(controller_nodes) > 0:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_camera_process,  # ADD THIS LINE - using process instead of node
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=gazebo_spawn_entity_node,
            #         on_exit=controller_nodes,
            #     )
            # ),

            TimerAction(
                period=10.0,  # xArm + Gazebo，5 秒是经验安全值
                actions=controller_nodes,
            ),
            robot_state_publisher_node,
            # gazebo_launch,
            # gazebo_spawn_entity_node,
        ]
    else:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_camera_process,  # ADD THIS LINE - using process instead of node
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            robot_state_publisher_node,
            # gazebo_launch,
            # gazebo_spawn_entity_node,
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


# !/usr/bin/env python3
# Software License Agreement (BSD License)

# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.

# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

# import os
# import yaml
# from pathlib import Path
# from ament_index_python import get_package_share_directory
# from launch.launch_description_sources import load_python_launch_file_as_module
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, LogInfo
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch.conditions import IfCondition
# from launch_ros.substitutions import FindPackageShare
# from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
# from launch.actions import OpaqueFunction, SetEnvironmentVariable
# from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file
# from launch.actions import TimerAction

    
# def launch_setup(context, *args, **kwargs):
#     prefix = LaunchConfiguration('prefix', default='')
#     hw_ns = LaunchConfiguration('hw_ns', default='xarm')
#     limited = LaunchConfiguration('limited', default=False)
#     effort_control = LaunchConfiguration('effort_control', default=False)
#     velocity_control = LaunchConfiguration('velocity_control', default=False)
#     add_gripper = LaunchConfiguration('add_gripper', default=False)
#     add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
#     add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
#     dof = LaunchConfiguration('dof', default=7)
#     robot_type = LaunchConfiguration('robot_type', default='xarm')
#     ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
#     add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
#     add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
#     model1300 = LaunchConfiguration('model1300', default=False)
#     robot_sn = LaunchConfiguration('robot_sn', default='')
#     attach_to = LaunchConfiguration('attach_to', default='world')
#     attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
#     attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

#     add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
#     geometry_type = LaunchConfiguration('geometry_type', default='box')
#     geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
#     geometry_height = LaunchConfiguration('geometry_height', default=0.1)
#     geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
#     geometry_length = LaunchConfiguration('geometry_length', default=0.1)
#     geometry_width = LaunchConfiguration('geometry_width', default=0.1)
#     geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
#     geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
#     geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
#     geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
#     geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
#     kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
#     load_controller = LaunchConfiguration('load_controller', default=True)
#     show_rviz = LaunchConfiguration('show_rviz', default=False)
#     no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

#     ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
#     moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')

#     moveit_config_dump = moveit_config_dump.perform(context)
#     moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}
#     moveit_config_package_name = 'xarm_moveit_config'
#     xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
#     gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')

#     if not moveit_config_dict:
#         # ros2 control params
#         ros2_control_params = generate_ros2_control_params_temp_file(
#             os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
#             prefix=prefix.perform(context), 
#             add_gripper=add_gripper.perform(context) in ('True', 'true'),
#             add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
#             ros_namespace=ros_namespace,
#             update_rate=1000,
#             use_sim_time=True,
#             robot_type=robot_type.perform(context)
#         )
#         # robot_description
#         robot_description = {
#             'robot_description': get_xacro_content(
#                 context,
#                 xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro', 
#                 dof=dof,
#                 robot_type=robot_type,
#                 prefix=prefix,
#                 hw_ns=hw_ns,
#                 limited=limited,
#                 effort_control=effort_control,
#                 velocity_control=velocity_control,
#                 model1300=model1300,
#                 robot_sn=robot_sn,
#                 attach_to=attach_to,
#                 attach_xyz=attach_xyz,
#                 attach_rpy=attach_rpy,
#                 kinematics_suffix=kinematics_suffix,
#                 ros2_control_plugin='gazebo_ros2_control/GazeboSystem',
#                 ros2_control_params=ros2_control_params,
#                 add_gripper=add_gripper,
#                 add_vacuum_gripper=add_vacuum_gripper,
#                 add_bio_gripper=add_bio_gripper,
#                 add_realsense_d435i=add_realsense_d435i,
#                 add_d435i_links=add_d435i_links,
#                 add_other_geometry=add_other_geometry,
#                 geometry_type=geometry_type,
#                 geometry_mass=geometry_mass,
#                 geometry_height=geometry_height,
#                 geometry_radius=geometry_radius,
#                 geometry_length=geometry_length,
#                 geometry_width=geometry_width,
#                 geometry_mesh_filename=geometry_mesh_filename,
#                 geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
#                 geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
#                 geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
#                 geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
#             )
#         }
#         moveit_config_dict = robot_description
#     else:
#         robot_description = {'robot_description': moveit_config_dict['robot_description']}

#     # robot state publisher node
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': True}, robot_description],
#         remappings=[
#             ('/tf', 'tf'),
#             ('/tf_static', 'tf_static'),
#         ]
#     )

#     # gazebo launch
#     xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
#         launch_arguments={
#             'world': xarm_gazebo_world,
#             'server_required': 'true',
#             'gui_required': gazebo_gui,
#             'verbose': 'true',
#         }.items(),
#     )

#     # gazebo spawn entity node
#     gazebo_spawn_entity_node = Node(
#         package="gazebo_ros",
#         executable="spawn_entity.py",
#         output='screen',
#         arguments=[
#             '-topic', 'robot_description',
#             '-entity', 'UF_ROBOT',
#             '-x', '-0.2',
#             '-y', '-0.54' if robot_type.perform(context) == 'uf850' else '-0.5',
#             '-z', '1.021',
#             '-Y', '1.571',
#             '-timeout', '30',
#         ],
#         parameters=[{'use_sim_time': True}],
#     )

#     # Camera spawn
#     camera_model_path = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'sim_camera.sdf'])
#     gazebo_spawn_camera_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'sim_camera',
#             '-file', camera_model_path.perform(context),
#             '-timeout', '30',
#         ],
#         output='screen',
#     )

#     # rviz with moveit configuration
#     rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
#     rviz2_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_file],
#         parameters=[
#             {
#                 'robot_description': moveit_config_dict.get('robot_description', ''),
#                 'robot_description_semantic': moveit_config_dict.get('robot_description_semantic', ''),
#                 'robot_description_kinematics': moveit_config_dict.get('robot_description_kinematics', {}),
#                 'robot_description_planning': moveit_config_dict.get('robot_description_planning', {}),
#                 'planning_pipelines': moveit_config_dict.get('planning_pipelines', {}),
#                 'use_sim_time': True
#             }
#         ],
#         condition=IfCondition(show_rviz),
#         remappings=[
#             ('/tf', 'tf'),
#             ('/tf_static', 'tf_static'),
#         ]
#     )

#     # Simple Python script to wait for service
#     wait_script_path = os.path.join(os.path.dirname(__file__), 'wait_for_service.py')
    
#     # If script doesn't exist, create it temporarily
#     if not os.path.exists(wait_script_path):
#         wait_script_content = '''#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import sys
# import time

# def main():
#     rclpy.init()
#     node = Node('wait_for_service')
    
#     from rclpy.wait_for_service import wait_for_service
    
#     service_name = '/controller_manager/list_controllers'
#     timeout = 30.0
    
#     node.get_logger().info(f'Waiting for service: {service_name}')
    
#     try:
#         wait_for_service(service_name, node=node, timeout=timeout)
#         node.get_logger().info(f'Service {service_name} is available!')
#         return 0
#     except Exception as e:
#         node.get_logger().error(f'Failed to wait for service: {e}')
#         return 1
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     sys.exit(main())
# '''
#         # Create temporary script in current directory
#         temp_script_path = '/tmp/wait_for_controller_manager.py'
#         with open(temp_script_path, 'w') as f:
#             f.write(wait_script_content)
#         os.chmod(temp_script_path, 0o755)
#         wait_script_path = temp_script_path

#     wait_for_controller_manager = ExecuteProcess(
#         cmd=['python3', wait_script_path],
#         name='wait_for_controller_manager',
#         output='screen',
#     )

#     return [
#         # Start robot state publisher
#         robot_state_publisher_node,
        
#         # Start Gazebo with delay
#         TimerAction(
#             period=2.0,
#             actions=[gazebo_launch]
#         ),
        
#         # Spawn robot after Gazebo
#         TimerAction(
#             period=5.0,
#             actions=[gazebo_spawn_entity_node]
#         ),
        
#         # Spawn camera
#         TimerAction(
#             period=6.0,
#             actions=[gazebo_spawn_camera_node]
#         ),
        
#         # Wait for controller manager (critical!)
#         TimerAction(
#             period=8.0,
#             actions=[wait_for_controller_manager]
#         ),
        
#         # Spawn controllers after waiting for controller manager
#         # Using correct spawner arguments
#         TimerAction(
#             period=10.0,
#             actions=[
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     name='spawner_joint_state_broadcaster',
#                     output='screen',
#                     arguments=[
#                         'joint_state_broadcaster',
#                         '--controller-manager', '/controller_manager',
#                         '--controller-manager-timeout', '30',  # CORRECT ARGUMENT NAME
#                     ],
#                     parameters=[{'use_sim_time': True}],
#                 )
#             ]
#         ),
        
#         # Spawn motion controller
#         TimerAction(
#             period=12.0,
#             actions=[
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     name='spawner_xarm_cartesian_motion_controller',
#                     output='screen',
#                     arguments=[
#                         '{}{}_cartesian_motion_controller'.format(prefix.perform(context), xarm_type),
#                         '--controller-manager', '/controller_manager',
#                         '--controller-manager-timeout', '30',
#                     ],
#                     parameters=[{'use_sim_time': True}],
#                 )
#             ]
#         ),
        
#         # Spawn gripper controller if needed
#         TimerAction(
#             period=14.0,
#             actions=[
#                 Node(
#                     package='controller_manager',
#                     executable='spawner',
#                     name='spawner_xarm_gripper_traj_controller',
#                     output='screen',
#                     arguments=[
#                         '{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)),
#                         '--controller-manager', '/controller_manager',
#                         '--controller-manager-timeout', '30',
#                     ],
#                     parameters=[{'use_sim_time': True}],
#                 ) if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true') else LogInfo(msg='No gripper controller to spawn')
#             ]
#         ),
        
#         # Start RViz if requested
#         TimerAction(
#             period=16.0,
#             actions=[rviz2_node],
#             condition=IfCondition(show_rviz)
#         ),
#     ]


# def generate_launch_description():
#     return LaunchDescription([
#         OpaqueFunction(function=launch_setup)
#     ])