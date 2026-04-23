import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('nbv_planner')

    blueye_params_path   = os.path.join(pkg_share, 'config', 'nbv_planner_params_oculus_m750d.yaml')
    oceansim_params_path = os.path.join(pkg_share, 'config', 'nbv_planner_params_oceansim.yaml')
    bluerov_params_path  = os.path.join(pkg_share, 'config', 'nbv_planner_params_bluerov.yaml')

    declare_sonar = DeclareLaunchArgument(
        'sonar',
        default_value='blueye',
        description='Sonar backend: "blueye" (Oculus M750d), "oceansim" (Isaac Sim), or "bluerov" (Sonoptix ECHO).'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PythonExpression([
            "'", blueye_params_path,   "' if '", LaunchConfiguration('sonar'), "' == 'blueye' else ",
            "('", oceansim_params_path, "' if '", LaunchConfiguration('sonar'), "' == 'oceansim' else ",
            "'", bluerov_params_path, "')",
        ]),
        description='Full path to the ROS 2 parameters file to use'
    )

    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sonar_point_cloud',
        description='Input point cloud topic'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression([
            "'true' if '", LaunchConfiguration('sonar'), "' == 'oceansim' else 'false'"
        ]),
        description='Use simulation clock if true. Always false for real hardware.'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # NBV Planner Node
    nbv_planner_node = Node(
        package='nbv_planner',
        executable='nbv_planner_node',
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

    # BlueEye — Oculus M750d
    sonar_point_cloud_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'true' if '", LaunchConfiguration('sonar'), "' == 'blueye' else 'false'"
        ])),
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # Isaac Sim sonar node
    sonar_point_cloud_oceansim_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud_oceansim',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'true' if '", LaunchConfiguration('sonar'), "' == 'oceansim' else 'false'"
        ])),
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # BlueROV + Sonoptix ECHO node
    sonar_point_cloud_bluerov_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud_bluerov',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'true' if '", LaunchConfiguration('sonar'), "' == 'bluerov' else 'false'"
        ])),
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        # Declare arguments
        declare_sonar,
        declare_params_file,
        declare_cloud_topic,
        declare_use_rviz,
        declare_log_level,
        declare_use_sim_time,

        # Launch nodes
        nbv_planner_node,
        rviz_node,
        static_transform_publisher_forward_camera_node,
        static_transform_publisher_bottom_camera_node,
        sonar_point_cloud_node,
        sonar_point_cloud_oceansim_node,
        sonar_point_cloud_bluerov_node,
        sonar_tf_publisher_node,
    ])
