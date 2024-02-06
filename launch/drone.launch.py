from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import datetime


def generate_launch_description():
    sensor_integration_dir_launch = os.path.join(
        get_package_share_directory('sensor_integration_suite'),
        'launch'
    )
    package_dir = get_package_share_directory('drone_bringup')
    assets_dir = os.path.join(package_dir,'assets')
    config_dir = os.path.join(package_dir,'config')
    now = datetime.datetime.now()
    bag_file_name = now.strftime("%Y%m%d%H%M%S")
    bag_file_path = os.path.join(assets_dir,'recorded_data_'+ bag_file_name +'.bag')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
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
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rsp.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )
    sensor_integration_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/sensor_integration.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )


    return LaunchDescription([
        use_sim_time_arg,
        record_bag,
        robot_state_publisher,
        sensor_integration_suite
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([drone_launch_dir, '/rtabmap.launch.py'])
        # )

    ])
