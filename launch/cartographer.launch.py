from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = PathJoinSubstitution([
        FindPackageShare('drone_bringup'), 'config', 'my_cartographer_config.lua'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Path to the cartographer configuration file.'
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'configuration_basename': LaunchConfiguration('config_file')}],
            remappings=[
                # Remap the merged point cloud topic if necessary
                ('/points2', '/merged/point_cloud'),
                # ('/points2_1','/lidar/points_vt'),
                # ('points2_2','/lidar/points_hz'),
                # ('points2_3','/zed/zed_node/point_cloud/cloud_registered'),
                ('odom','/zed/zed_node/odom'),
                ('imu','/zed/zed_node/imu/data')
                # Add other necessary remappings here
            ],
        ),
        # Optionally launch rviz for visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', PathJoinSubstitution([
        #         FindPackageShare('drone_bringup'), 'rviz', 'drone_slam.rviz'
        #     ])],
        # ),
    ])
