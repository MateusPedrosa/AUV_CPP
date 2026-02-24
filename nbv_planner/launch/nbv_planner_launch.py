import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # 1. Get the path to your config file
    pkg_share = get_package_share_directory('nbv_planner')
    default_params_path = os.path.join(pkg_share, 'config', 'nbv_planner_params_oculus_1200d.yaml')

    # 2. Add a Launch Argument so you can swap the YAML file easily
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use'
    )

    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sonar_point_cloud',
        description='Input point cloud topic'
    )

    #################################
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    #################################

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # NBV Planner Node
    nbv_planner_node = Node(
        package='nbv_planner',
        executable='nbv_planner_node',
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
        ]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '$(find nbv_planner)/rviz/nbv_planner.rviz'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    # static_transform_publisher_sonar_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=[
    #         '--x',           '0.1',
    #         '--y',           '0.0',
    #         '--z',           '0.2',
    #         '--roll',        '0.0',
    #         '--pitch',       '0.0',
    #         '--yaw',         '0.0',
    #         '--frame-id',    'base_link',
    #         '--child-frame-id', 'sonar_link'
    #     ]
    # )

    sonar_tf_publisher_node = Node(
        package='sonar_mapping',
        executable='sonar_tf_publisher',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    static_transform_publisher_forward_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.3',
            '--y', '0.0',
            '--z', '0.1',
            '--roll', '1.5708',  # 90 degrees in radians
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_forward_link'
        ]
    )

    static_transform_publisher_bottom_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.3',
            '--y', '0.0',
            '--z', '0.1',
            '--roll', '1.5708',  # 90 degrees in radians
            '--pitch', '1.5708', # 90 degrees in radians
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_bottom_link'
        ]
    )

    sonar_point_cloud_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud',
        output='screen',
        # This now loads the parameters from the YAML file passed to the launch script
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        # Declare arguments
        declare_params_file,
        declare_cloud_topic,
        declare_use_rviz,
        declare_log_level,

        # Launch nodes
        nbv_planner_node,
        rviz_node,
        # static_transform_publisher_sonar_node,
        static_transform_publisher_forward_camera_node,
        static_transform_publisher_bottom_camera_node,
        sonar_point_cloud_node,
        sonar_tf_publisher_node,
    ])