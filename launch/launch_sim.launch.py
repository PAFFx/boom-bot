import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "boom_bot"

    # Check if we're told to use sim time
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","rsp.launch.py"
                    )]
            ),
            launch_arguments={"use_ros2_control":use_ros2_control, "sim_mode":'true'}.items()
        )

    gazebo_params_file = os.path.join(
            get_package_share_directory(package_name),"config","gazebo_params.yaml"
            )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory("gazebo_ros"),"launch","gazebo.launch.py"
                    )]
                ),
            launch_arguments={"extra_gazebo_args":"--ros-args --params-file " + gazebo_params_file}.items()
            )

    # Run the spawner node from the gazebo_ros pacakge.
    spawn_entity = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'boom_bot'],
                        output="screen")

    diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"]
            )

    joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"]
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='use gazebo control if set to false'),
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ])

