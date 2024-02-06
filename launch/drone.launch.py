from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import LogInfo
import os
import datetime


def generate_launch_description():
    sensor_integration_dir_launch = os.path.join(
        get_package_share_directory('sensor_integration_suite'),
        'launch'
    )
    gazebo_ros_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'),
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
    
    
    # Record Bag file
    record_bag = ExecuteProcess(
        cmd=[  'ros2', 'bag', 'record', '-o',
                bag_file_path,
                '/merged/point_cloud', '/zed/zed_node/odom',
                '/zed/zed_node/imu/data','/tf','/tf_static'
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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_launch_dir,'/gazebo.launch.py'])
    )
    launch_list = [use_sim_time_arg]
    if(LaunchConfiguration('use_sim_time')):
        LogInfo(msg=["This is a message printed from the launch file!",LaunchConfiguration('use_sim_time')])
        launch_list.append(gazebo_launch)
        launch_list.append(record_bag)
    launch_list.append(robot_state_publisher)
    launch_list.append(sensor_integration_suite)

    return LaunchDescription(launch_list)
