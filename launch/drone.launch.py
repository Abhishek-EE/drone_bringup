from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    sensor_integration_dir_launch = os.path.join(
        get_package_share_directory('sensor_integration_suite'),
        'launch'
    )
    drone_launch_dir = os.path.join(
        get_package_share_directory('drone_bringup'),
        'launch'
    )

    return LaunchDescription([
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
