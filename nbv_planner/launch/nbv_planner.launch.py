from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Frame ID for the map'
    )
    
    declare_robot_frame = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Frame ID for the robot base'
    )
    
    declare_octree_resolution = DeclareLaunchArgument(
        'octree_resolution',
        default_value='0.1',
        description='Octree resolution in meters'
    )
    
    declare_planning_frequency = DeclareLaunchArgument(
        'planning_frequency',
        default_value='0.5',
        description='Planning frequency in Hz'
    )
    
    declare_sensor_range = DeclareLaunchArgument(
        'sensor_range',
        default_value='3.0',
        description='Maximum sensor range in meters'
    )
    
    declare_sensor_min_range = DeclareLaunchArgument(
        'sensor_min_range',
        default_value='0.2',
        description='Minimum sensor range in meters'
    )
    
    declare_max_free_space = DeclareLaunchArgument(
        'max_free_space',
        default_value='0.0',
        description='Maximum distance to mark as free space (0 = unlimited)'
    )

    declare_sensor_hfov = DeclareLaunchArgument(
        'sensor_hfov',
        default_value=str(130.0 * 3.14159 / 180.0),  # 130 degrees in radians
        description='Sensor horizontal field of view in radians'
    )

    declare_sensor_vfov = DeclareLaunchArgument(
        'sensor_vfov',
        default_value=str(20.0 * 3.14159 / 180.0),  # 20 degrees in radians
        description='Sensor vertical field of view in radians'
    )

    declare_min_height_free_space = DeclareLaunchArgument(
        'min_height_free_space',
        default_value='0.0',
        description='Minimum height for free space marking'
    )
    
    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sonar_point_cloud',
        description='Input point cloud topic'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # NBV Planner Node
    nbv_planner_node = Node(
        package='nbv_planner',
        executable='nbv_planner_node',
        name='nbv_planner',
        output='screen',
        parameters=[{
            'map_frame': LaunchConfiguration('map_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'octree_resolution': LaunchConfiguration('octree_resolution'),
            'planning_frequency': LaunchConfiguration('planning_frequency'),
            'sensor_range': LaunchConfiguration('sensor_range'),
            'sensor_min_range': LaunchConfiguration('sensor_min_range'),
            'max_free_space': LaunchConfiguration('max_free_space'),
            'min_height_free_space': LaunchConfiguration('min_height_free_space'),
            'sensor_hfov': LaunchConfiguration('sensor_hfov'),
            'sensor_vfov': LaunchConfiguration('sensor_vfov'),
        }],
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

    return LaunchDescription([
        # Declare arguments
        declare_map_frame,
        declare_robot_frame,
        declare_octree_resolution,
        declare_planning_frequency,
        declare_sensor_range,
        declare_sensor_min_range,
        declare_max_free_space,
        declare_min_height_free_space,
        declare_sensor_hfov,
        declare_sensor_vfov,
        declare_cloud_topic,
        declare_use_rviz,
        
        # Launch nodes
        nbv_planner_node,
        rviz_node,
    ])