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

    laser_frame = LaunchConfiguration('laser_frame_id')
    laser_port = LaunchConfiguration('laser_port')

    robot_launch = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","launch_robot.launch.py"
                    )]
            )

    lidar = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","hlslidar.launch.py"
                    )],
            launch_arguments={"port":laser_port, "laser_frame":laser_frame}.items()
            )

    costmap = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","online_async.launch.py"
                    )],
            launch_arguments={"slam_params_file":os.path.join(
                get_package_share_directory(package_name), "config", "navigation_params_online_async.yaml"
                )}.items()
            )

    amcl = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","localization.launch.py"
                    )],
            )

    navigation = IncludeLaunchDescription(
                [os.path.join(
                    get_package_share_directory(package_name), "launch","navigation.launch.py"
                    )],
                )


    

    return LaunchDescription([
        DeclareLaunchArgument(
            'laser_port',
            default_value='/dev/ttyUSB1',
            description='Lidar\'s port'
            ),
        DeclareLaunchArgument(
            'laser_frame_id',
            default_value='laser_frame',
            description='laser frame name'
            ),
        robot_launch,
        lidar,
        costmap,
        amcl,
        navigation
        ])
