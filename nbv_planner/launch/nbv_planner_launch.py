import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 1. Get the path to your config file
    pkg_share = get_package_share_directory('nbv_planner')
    default_params_path = os.path.join(pkg_share, 'config', 'nbv_planner_params_oceansim.yaml')

    # 2. Add a Launch Argument so you can swap the YAML file easily
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use'
    )

    declare_simulator = DeclareLaunchArgument(
        'simulator',
        default_value='false',
        description='If true, runs sonar_point_cloud_oceansim. If false, runs sonar_point_cloud.'
    )

    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sonar_point_cloud',
        description='Input point cloud topic'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # Set to true since you are using MCAP playback
        description='Use simulation (log) clock if true'
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
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
        ]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    sonar_point_cloud_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('simulator')),
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # Simulator Sonar Node (Runs if simulator is true)
    sonar_point_cloud_oceansim_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud_oceansim',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulator')),
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        # Declare arguments
        declare_params_file,
        declare_simulator,
        declare_cloud_topic,
        declare_use_rviz,
        declare_log_level,
        declare_use_sim_time,

        # Launch nodes
        nbv_planner_node,
        rviz_node,
        # static_transform_publisher_sonar_node,
        static_transform_publisher_forward_camera_node,
        static_transform_publisher_bottom_camera_node,
        sonar_point_cloud_node,
        sonar_point_cloud_oceansim_node,
        sonar_tf_publisher_node,
    ])