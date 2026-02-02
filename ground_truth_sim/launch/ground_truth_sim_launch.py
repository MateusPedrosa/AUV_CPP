from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare the argument (variable)
    ply_path_arg = DeclareLaunchArgument(
        'ply_path',
        default_value='/home/mateus/oceanslam_ws/oceansim_scripts/voxel_map.ply',
        description='Absolute path to the voxel map .ply file'
    )

    # 2. Define the Node using the configuration
    voxel_node = Node(
        package='ground_truth_sim',
        executable='voxel_map_ground_truth',
        name='voxel_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'ply_path': LaunchConfiguration('ply_path')}
        ]
    )

    return LaunchDescription([
        ply_path_arg,
        voxel_node
    ])