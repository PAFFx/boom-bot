import os

from launch import LaunchDescription
from launch.launch_introspector import ExecuteProcess

def generate_launch_description():

    
    package = 'hls_lfcd_lds_driver'
    launch_file = 'hlds_laser.launch.py'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[[
                'ros2 launch ',package,' ',launch_file,
                ' frame_id:=laser_frame', ' port:=/dev/ttyUSB0'
                ]],
            shell=True
            )
        ])
