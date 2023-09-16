import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "boom_bot"

    robot_launch = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","launch_robot.launch.py"
                    )]
            )


    neural_net_nav = Node(
            package=package_name,
            executable="neural_net_nav.py",
            )

    

    return LaunchDescription([
        robot_launch,
        neural_net_nav,
        ])
