from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # config_file_path = PathJoinSubstitution([
    #     FindPackageShare('drone_bringup'), 'config', 'my_cartographer_config.lua'
    # ])
    config_file_path = os.path.join(
        get_package_share_directory('drone_bringup'),
        'config'
    )

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': False,  # Example parameter
                # Add other parameters as needed
            }],
            arguments=[
                '-configuration_directory', config_file_path,
                '-configuration_basename', 'my_cartographer_config.lua',
                # Add other command line arguments as needed
            ],
            remappings=[
                ('/points2', '/merged/point_cloud'),
                ('odom', '/zed/zed_node/odom'),
                ('imu', '/zed/zed_node/imu/data_raw'),
                # Add other remappings as needed
            ],
            output='screen'
        ),
        # Add other nodes or launch actions as needed
    ])
