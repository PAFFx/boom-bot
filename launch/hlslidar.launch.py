import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.launch_introspector import ExecuteProcess
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def launch_setup(context: LaunchContext):
    port = LaunchConfiguration('port').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    
    package = 'hls_lfcd_lds_driver'
    launch_file = 'hlds_laser.launch.py'

    lidar = ExecuteProcess(
            cmd=[[
                f"ros2 launch {package} {launch_file} frame_id:={frame_id} port:={port}"
                ]],
            shell=True
            )
    return [lidar]

def generate_launch_description():
    
    package_name="boom_bot"
    laser_filter_params = os.path.join(get_package_share_directory(package_name), "config", "laser_filter.yaml")

    laser_filter=  Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[{"load":laser_filter_params}],
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Lidar\'s port'
            ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='laser frame name'
            ),
        OpaqueFunction(function=launch_setup),
        laser_filter

        ])
