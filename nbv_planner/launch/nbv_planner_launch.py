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

    # Declare launch arguments
    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='odom',
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
    
    declare_exploration_sensor_max_range = DeclareLaunchArgument(
        'exploration_sensor_max_range',
        default_value='10.0',
        description='Maximum exploration sensor range in meters'
    )
    
    declare_exploration_sensor_min_range = DeclareLaunchArgument(
        'exploration_sensor_min_range',
        default_value='0.1',
        description='Minimum exploration sensor range in meters'
    )
    
    declare_max_free_space = DeclareLaunchArgument(
        'max_free_space',
        default_value='0.0',
        description='Maximum distance to mark as free space (0 = unlimited)'
    )

    declare_min_height_free_space = DeclareLaunchArgument(
        'min_height_free_space',
        default_value='0.0',
        description='Minimum height for free space marking'
    )

    declare_exploration_sensor_hfov = DeclareLaunchArgument(
        'exploration_sensor_hfov',
        default_value=str(130.0 * 3.14159 / 180.0),  # 130 degrees in radians
        description='Exploration sensor horizontal field of view in radians'
    )

    declare_exploration_sensor_vfov = DeclareLaunchArgument(
        'exploration_sensor_vfov',
        default_value=str(20.0 * 3.14159 / 180.0),  # 20 degrees in radians
        description='Exploration sensor vertical field of view in radians'
    )

    declare_inspection_sensor_frames = DeclareLaunchArgument(
        'inspection_sensor_frames',
        default_value="['camera_forward_link', 'camera_bottom_link']",
        description='Frame IDs for the inspection sensors'
    )

    declare_exploration_sensor_frame = DeclareLaunchArgument(
        'exploration_sensor_frame',
        default_value='sonar_link',
        description='Frame ID for the exploration sensor'
    )

    declare_camera_fx = DeclareLaunchArgument(
        'camera_fx',
        default_value='731.79',
        description='Camera focal length in x'
    )

    declare_camera_fy = DeclareLaunchArgument(
        'camera_fy',
        default_value='731.79',
        description='Camera focal length in y'
    )

    declare_camera_cx = DeclareLaunchArgument(
        'camera_cx',
        default_value='970.94',
        description='Camera principal point x'
    )

    declare_camera_cy = DeclareLaunchArgument(
        'camera_cy',
        default_value='600.37',
        description='Camera principal point y'
    )

    declare_camera_width = DeclareLaunchArgument(
        'camera_width',
        default_value='1216',
        description='Camera image width'
    )

    declare_camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value='1936',
        description='Camera image height'
    )

    declare_camera_max_range = DeclareLaunchArgument(
        'camera_max_range',
        default_value='2.0',
        description='Maximum range for camera sensor'
    )
    
    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sonar_point_cloud',
        description='Input point cloud topic'
    )

    declare_sonar_x = DeclareLaunchArgument('sonar_x', default_value='0.3')
    declare_sonar_y = DeclareLaunchArgument('sonar_y', default_value='0.0')
    declare_sonar_z = DeclareLaunchArgument('sonar_z', default_value='0.3')
    declare_sonar_roll = DeclareLaunchArgument('sonar_roll', default_value='0.0')
    declare_sonar_pitch = DeclareLaunchArgument('sonar_pitch', default_value='0.0')
    declare_sonar_yaw = DeclareLaunchArgument('sonar_yaw', default_value='0.0')

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
        parameters=[
            {
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'octree_resolution': LaunchConfiguration('octree_resolution'),
                'planning_frequency': LaunchConfiguration('planning_frequency'),
                'exploration_sensor_max_range': LaunchConfiguration('exploration_sensor_max_range'),
                'exploration_sensor_min_range': LaunchConfiguration('exploration_sensor_min_range'),
                'max_free_space': LaunchConfiguration('max_free_space'),
                'min_height_free_space': LaunchConfiguration('min_height_free_space'),
                'exploration_sensor_hfov': LaunchConfiguration('exploration_sensor_hfov'),
                'exploration_sensor_vfov': LaunchConfiguration('exploration_sensor_vfov'),
                'inspection_sensor_frames': LaunchConfiguration('inspection_sensor_frames'),
                'exploration_sensor_frame': LaunchConfiguration('exploration_sensor_frame'),
                'camera_fx': LaunchConfiguration('camera_fx'),
                'camera_fy': LaunchConfiguration('camera_fy'),
                'camera_cx': LaunchConfiguration('camera_cx'),
                'camera_cy': LaunchConfiguration('camera_cy'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_max_range': LaunchConfiguration('camera_max_range'),
            },
            LaunchConfiguration('params_file')
        ],
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

    static_transform_publisher_sonar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # We reference the arguments from the LaunchConfigurations 
        # defined at the top of your file
        arguments=[
            '--x', LaunchConfiguration('sonar_x'),
            '--y', LaunchConfiguration('sonar_y'),
            '--z', LaunchConfiguration('sonar_z'),
            '--roll', LaunchConfiguration('sonar_roll'),
            '--pitch', LaunchConfiguration('sonar_pitch'),
            '--yaw', LaunchConfiguration('sonar_yaw'),
            '--frame-id', LaunchConfiguration('robot_frame'),
            '--child-frame-id', LaunchConfiguration('exploration_sensor_frame')
        ]
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
        name='sonar_point_cloud_node',
        output='screen',
        parameters=[{
            'sonar_topic': '/blueye/sensor/sonar/ping',
            'pose_topic': '/blueye/sensor/ekf/pose',
            'resolution': 0.1,        # 10cm voxels
            'max_range': 10.0,        # Match your Octomap range
            'min_range': 0.1,
            'horizontal_fov': 60.0,  # Or 60.0 if using HF
            'vertical_fov': 12.0,
            'min_intensity_short_range': 0.4,
            'min_intensity_long_range': 0.2,
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    sonar_point_cloud_node = Node(
        package='sonar_mapping',
        executable='sonar_point_cloud',
        name='sonar_point_cloud_node',
        output='screen',
        # This now loads the parameters from the YAML file passed to the launch script
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        # Declare arguments
        declare_params_file,
        declare_map_frame,
        declare_robot_frame,
        declare_octree_resolution,
        declare_planning_frequency,
        declare_exploration_sensor_max_range,
        declare_exploration_sensor_min_range,
        declare_max_free_space,
        declare_min_height_free_space,
        declare_exploration_sensor_hfov,
        declare_exploration_sensor_vfov,
        declare_inspection_sensor_frames,
        declare_exploration_sensor_frame,
        declare_camera_fx,
        declare_camera_fy,
        declare_camera_cx,
        declare_camera_cy,
        declare_camera_width,
        declare_camera_height,
        declare_camera_max_range,
        declare_cloud_topic,
        declare_use_rviz,
        declare_log_level,
        declare_sonar_x,
        declare_sonar_y,
        declare_sonar_z,
        declare_sonar_roll,
        declare_sonar_pitch,
        declare_sonar_yaw,

        # Launch nodes
        nbv_planner_node,
        rviz_node,
        static_transform_publisher_sonar_node,
        static_transform_publisher_forward_camera_node,
        static_transform_publisher_bottom_camera_node,
        sonar_point_cloud_node,
    ])