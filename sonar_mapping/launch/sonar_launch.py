import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    return LaunchDescription([
        log_level_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.3',
                '--y', '0.0',
                '--z', '0.3',
                '--roll', '0.0',
                '--pitch', '0.785398',  # 45 degrees in radians
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'sonar_link'
            ]
        ),
        Node(
            package='sonar_mapping',
            executable='sonar_point_cloud',
            name='sonar_point_cloud_node',
            output='screen',
            parameters=[{
                'sonar_topic': '/oceansim/robot/imaging_sonar',
                'pose_topic': '/oceansim/robot/pose',
                'resolution': 0.1,        # 10cm voxels
                'max_range': 3.0,        # Match your Octomap range
                'horizontal_fov': 130.0,  # Or 60.0 if using HF
                'vertical_fov': 20.0,
                'min_intensity_short_range': 0.4,
                'min_intensity_long_range': 0.2,
                'use_sim_time': True,
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])