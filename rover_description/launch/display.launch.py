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
    urdf_path = PathJoinSubstitution([
        root_dir,
        'urdf',
        'robot.xacro'
    ])
    # print(repr(root_dir))
    rviz_config = PathJoinSubstitution([
        root_dir,
        'config',
        'urdf_config.rviz'
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
            '-z', '0.52'  # Set robot 0.6 meters above ground
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
            # 'gz_args': 'empty.sdf -r'
            'gz_args': 'empty.sdf'
        }.items()
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # gazebo launch
        gz_sim_launch,
        create_entity_cmd,
    ])

if __name__=="__main__":
    generate_launch_description()