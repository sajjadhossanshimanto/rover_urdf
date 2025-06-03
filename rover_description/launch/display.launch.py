from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root_dir = FindPackageShare('rover_description')
    urdf_path = PathJoinSubstitution([
        root_dir,
        'urdf',
        'urdf_wheel.urdf'
    ])
    rviz_config = PathJoinSubstitution([
        root_dir,
        'rviz',
        'urdf_config.rviz'
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name='cat'), ' ', urdf_path]),
        value_type=str  # Tell ROS it's a string
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
        )
    ])
