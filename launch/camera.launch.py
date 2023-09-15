from launch import LaunchDescription, LaunchContext
from launch.launch_introspector import ExecuteProcess
from launch.actions import OpaqueFunction


def launch_setup(context: LaunchContext):
    package = 'astra_camera'
    launch_file = 'astra_mini.launch.py'

    lidar = ExecuteProcess(
            cmd=[[
                f"ros2 launch {package} {launch_file}"
                ]],
            shell=True
            )
    return [lidar]

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
        ])
