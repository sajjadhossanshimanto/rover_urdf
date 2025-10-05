from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def debug_path(root_dir_sub):
    from launch import LaunchContext
    context = LaunchContext()
    root_dir = root_dir_sub.perform(context)
    
    return root_dir

def generate_launch_description():
    root_dir = FindPackageShare('rover_description')
    config_dir = PathJoinSubstitution([root_dir, 'config'])
    urdf_path = PathJoinSubstitution([
        root_dir,
        'urdf',
        'robot.xacro'
    ])
    # print(repr(root_dir))

    rviz_config = PathJoinSubstitution([
        config_dir,
        'urdf_config_new.rviz'
    ])
    bridge_config = PathJoinSubstitution([
        config_dir,
        'gazebo_bridge.yaml'
    ])
    world_map =  PathJoinSubstitution([
        root_dir, 'worlds', 'empty.sdf'
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_path,
            ' rover_path:=', root_dir # passing var to urdf
        ]),
        value_type=str
    )

    # gazebo setup
    create_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description',
            # '-z', '0.52'  # Set robot 0.6 meters above ground
        ],
        output='screen'
    )
    print("GAZEBO_MODEL_PATH is set to:", debug_path(root_dir))

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            # 'gz_args': 'empty.sdf -r' # -r for start the simlation
            # 'gz_args': 'empty.sdf'
            'gz_args': world_map,
            # 'gz_args': 'spherical_coordinates.sdf'
            # 'gz_args': 'depth_camera_sensor.sdf'
            'output': 'screen'
        }.items()
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen' # only for debug to confirm running
    )

    wheel_controller_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    keleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        # output='screen',
        prefix='xterm -e',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
        wheel_controller_gui,

        # # gazebo launch
        gz_sim_launch,
        create_entity_cmd,
        ros_gz_bridge,

        # # keleop keyboard
        keleop
    ])

if __name__=="__main__":
    generate_launch_description()

'''
/camera/depth_image
/camera/image
/camera/images/camera_info
/camera/images/points

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/image \
    depth_topic:=/camera/depth_image \
    camera_info_topic:=/camera/images/camera_info \
    frame_id:=camera_sensor \
    use_sim_time:=true \
    approx_sync:=true qos:=2 rviz:=true queue_size:=30

'''