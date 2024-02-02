from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory('drone_bringup'),
        'config'
    )
    map_file_path = LaunchConfiguration('map_file_path', 
                                        default='map.pbstream')
    
    finish_trajectory_cmd = [
        'ros2', 'service', 'call', '/finish_trajectory',
        'cartographer_ros_msgs/srv/FinishTrajectory', '\'{"trajectory_id: 0"}\''
    ]

    write_state_cmd = [
        'ros2', 'service', 'call', '/write_state',
        'cartographer_ros_msgs/srv/WriteState',
        '\'{"filename": "', map_file_path, '", "include_unfinished_submaps": true}\''
    ]
    
    # Cartographer node
    cartographer_node = Node(
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
            # ('/points2', '/merged/point_cloud'),
            ('/odom', '/zed/zed_node/odom'),
            ('/imu', '/zed/zed_node/imu/data_raw'),
            ('/points2_1','/lidar/points_hz'),
            ('/points2_2','/lidar/points_vt'),
            ('/points2_3','/zed/zed_node/point_cloud/cloud_registered')
            # Add other remappings as needed
        ],
        output='screen'
    )
    
    # Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{
            'use_sim_time': False,
            # Add other parameters specific to the occupancy grid node as needed
            'resolution': 0.05,  # Example parameter for grid resolution
        }],
        remappings=[
            ('/map', '/map'),  # You might not need to remap this if default is fine
            # Add other remappings as needed
        ],
        output='screen'
    )
    # Command to finish the trajectory
    finish_trajectory = ExecuteProcess(
        cmd=finish_trajectory_cmd,
        name='finish_trajectory',
    )

    # Command to write the state to a file
    write_state = ExecuteProcess(
        cmd=write_state_cmd,
        name='write_state',
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file_path',
            default_value=config_file_path,
            description='Path to the Cartographer configuration file.',
        ),
        DeclareLaunchArgument(
            'map_file_path',
            default_value=map_file_path,
            description='Path where the map should be saved.',
        ),
        cartographer_node,
        occupancy_grid_node,
        finish_trajectory,
        write_state,
        # Add other nodes or launch actions as needed
    ])