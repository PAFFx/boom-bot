import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "boom_bot"

    robot_launch = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","launch_robot.launch.py"
                    )]
            )

    depth = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","camera.launch.py"
                    )],
            )

    neural_net_nav = Node(
            package=package_name,
            executable="neural_net_nav.py",
            )

    

    return LaunchDescription([
        robot_launch,
        depth,
        neural_net_nav,
        ])
