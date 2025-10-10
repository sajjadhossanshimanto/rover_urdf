from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('image', '/camera/depth_image'),   # Depth raw image
                # ('image', '/camera/images/points'),
                ('camera_info', '/camera/images/camera_info'),
                ('scan', '/scan')   # Output
            ],
            parameters=[{
                'output_frame': 'base_link',
                'range_min': 0.6,
                'range_max': 6.0,
                'scan_time': 0.033,
                'scan_height': 2
            }],
            # arguments=['--ros-args', '--log-level', 'debug']# for debugging only
        )
    ])
