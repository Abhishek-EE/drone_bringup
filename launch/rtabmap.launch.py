from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments to allow for custom configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_scan': True,
                'subscribe_stereo': True,
                'approx_sync': True,
                # Additional RTAB-Map parameters...
            }],
            remappings=[
                ('scan_cloud', '/merged/point_cloud'), # if pointcloud format
                ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                ('depth/image', '/zed/depth/image'),
                ('left/image_rect', '/zed/zed_node/left/image_rect_color'),
                ('right/image_rect', '/zed/zed_node/right/image_rect_color'),
                ('left/camera_info', '/zed/zed_node/left/camera_info'),
                ('right/camera_info', '/zed/zed_node/right/camera_info'),
                ('odom', '/zed/zed_node/odom')
            ],
            arguments=['-d', LaunchConfiguration('use_sim_time')]
        ),

        # Add other necessary nodes (like odometry)
    ])
