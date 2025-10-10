from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def declare_arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=default, description=desc)

def ld_params():
    return [
        declare_arg('use_sim_time', 'true', 'Use Gazebo sim time'),
        declare_arg('frame_id', 'base_link', 'Robot base frame (base_link or base_footprint)'),
        declare_arg('odom_frame_id', 'odom', 'Odom frame'),
        declare_arg('map_frame_id', 'map', 'Map frame'),
        declare_arg('rgb_topic', '/camera/image', 'RGB image topic'),
        declare_arg('depth_topic', '/camera/depth_image', 'Depth image topic'),
        declare_arg('camera_info_topic', '/camera/images/camera_info', 'Camera info topic'),
        declare_arg('approx_sync', 'true', 'Approximate time sync'),
        declare_arg('queue_size', '30', 'Sync queue size'),
        declare_arg('qos', '2', 'QoS: 0=system default, 1=SensorData (best effort), 2=Default (reliable)'),
    ]

def generate_launch_description():
    use_sim_time     = LaunchConfiguration('use_sim_time')
    frame_id         = LaunchConfiguration('frame_id')
    odom_frame_id    = LaunchConfiguration('odom_frame_id')
    map_frame_id     = LaunchConfiguration('map_frame_id')
    rgb_topic        = LaunchConfiguration('rgb_topic')
    depth_topic      = LaunchConfiguration('depth_topic')
    camera_info      = LaunchConfiguration('camera_info_topic')
    approx_sync      = LaunchConfiguration('approx_sync')
    queue_size       = LaunchConfiguration('queue_size')
    qos              = LaunchConfiguration('qos')

    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,            # robot base
            'approx_sync': approx_sync,
            'queue_size': queue_size,
            'qos': qos,
            'subscribe_rgbd': False,
            'subscribe_depth': True,
            'Reg/Force3DoF': True            # planar robot
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info),
            # Odometry topic will be /rtabmap/odom by default; leave as-is
        ]
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'map_frame_id': map_frame_id,
            'approx_sync': approx_sync,
            'queue_size': queue_size,
            'qos': qos,
            'subscribe_depth': True,
            'subscribe_rgbd': False,
            'Reg/Force3DoF': True,           # planar robot
            'Mem/IncrementalMemory': True,
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info),
            ('odom', '/rtabmap/odom'),       # use odom from rgbd_odometry
        ]
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'approx_sync': approx_sync,
            'queue_size': queue_size,
            'qos': qos,
            'subscribe_depth': True,
            'subscribe_rgbd': False,
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info),
            ('odom', '/rtabmap/odom'),
        ]
    )

    return LaunchDescription(ld_params() + [rgbd_odom, rtabmap, rtabmap_viz])
