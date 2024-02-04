from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
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
    #Define to assets dir
    assets_dir = os.path.join(package_dir,'assets')
    now = datetime.datetime.now()
    bag_file_name = now.strftime("%Y%m%d%H%M%S")

    bag_file_path = os.path.join(assets_dir,'recorded_data_'+ bag_file_name +'.bag')
    # Define a launch argument to control bag recording
    # record_bag_arg = DeclareLaunchArgument(
    #     'record_bag', default_value='true',
    #     description='Set to "true" to record a bag file during this launch session.'
    # )
    
    # Use the launch argument value to conditionally start bag recording
    record_bag = ExecuteProcess(
        cmd=[  'ros2', 'bag', 'record', '-o',
                bag_file_path,
                '/merged/point_cloud', '/zed/zed_node/odom',
                '/zed/zed_node/imu/data','/tf','/tf_static'
            ],
            output='screen'
    )


    return LaunchDescription([
        record_bag,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rsp.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sensor_integration_dir_launch, '/sensor_integration.launch.py'])
        )
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([drone_launch_dir, '/rtabmap.launch.py'])
        # )

    ])
