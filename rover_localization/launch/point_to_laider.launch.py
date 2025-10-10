from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/camera/images/points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'use_sim_time': True,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.01,
                'min_height': -0.3,
                'max_height': 2.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.033,
                'range_min': 0.3,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        )
    ])

"""
TODO: error
[pointcloud_to_laserscan]: New subscription discovered on topic '/scan', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
"""