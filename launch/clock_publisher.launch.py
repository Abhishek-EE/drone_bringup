from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_bringup',  # Replace with your package name
            executable='clock_publisher',
            name='clock_publisher',
            output='screen',
            emulate_tty=True,  # To ensure stdout and stderr are unbuffered
        ),
    ])