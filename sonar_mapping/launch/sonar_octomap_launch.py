import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            }]
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='sonar_octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.1,                 # Voxel size (matches your sonar voxel size)
                'frame_id': 'map',                 # Fixed global frame (accumulates data here)
                'base_frame_id': 'base_link',      # Robot base frame (for clearing free space)
                
                # Range parameters
                'sensor_model/max_range': 3.0,    # Should match your sonar max_range
                
                # Probabilistic parameters (Tuning for Sonar Noise)
                # "Hit": Probability increase if a point is detected
                # "Miss": Probability decrease if a ray passes through
                # "Min/Max": Clamping thresholds
                'sensor_model/hit': 0.7,           # Lower than default (0.9) to reduce noise artifacts
                'sensor_model/miss': 0.4,          # Default is 0.4
                'sensor_model/min': 0.12,
                'sensor_model/max': 0.97,
                
                # filtering
                'filter_ground': False,            # Set True only if you want to remove the sea floor
                'point_cloud_min_z': -100.0,
                'point_cloud_max_z': 100.0,
            }],
            remappings=[
                # Remap the default input topic 'cloud_in' to your topic
                ('cloud_in', '/sonar_point_cloud') 
            ]
        )
    ])