from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
import datetime


def generate_launch_description():
    sensor_integration_dir_launch = os.path.join(
        get_package_share_directory('sensor_integration_suite'),
        'launch'
    )
    package_dir = get_package_share_directory('drone_bringup')
    assets_dir = os.path.join(package_dir,'assets')
    package_launch_dir = os.path.join(package_dir,'launch')
    now = datetime.datetime.now()
    bag_file_name = now.strftime("%Y%m%d%H%M%S")
    bag_file_path = os.path.join(assets_dir,'recorded_data_'+ bag_file_name +'.bag')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    record_bag_arg = DeclareLaunchArgument(
        'record_data', default='false',
        description='record data from lidar and zed camera'
    )   
    
    
    # Record Bag file
    record_bag = ExecuteProcess(
        cmd=[  'ros2', 'bag', 'record', '-o',
                bag_file_path,
                '/merged/point_cloud', '/zed/zed_node/odom',
                '/zed/zed_node/imu/data','/tf','/tf_static',
                '/lidar/points_hz','/lidar/points_vt',
                '/zed/zed_node/point_cloud/cloud_registered'
            ],
            output='screen'
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_launch_dir, '/rsp.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )
    sensor_integration_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_integration_dir_launch, '/sensor_integration.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    clock_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_launch_dir, '/clock_publisher.launch.py']),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    conditional_actions = GroupAction(
        actions=[
            LogInfo(msg=["Starting the recording of bag files"]),
            record_bag# Add record_bag action here
        ],
        condition=IfCondition(LaunchConfiguration('record_data'))
    )
    always_included_actions = [
        robot_state_publisher,# Add robot_state_publisher action here
        sensor_integration_suite# Add sensor_integration_suite action here
    ]
    return LaunchDescription([
        use_sim_time_arg,
        clock_publisher,
        conditional_actions,
        *always_included_actions]
    )
