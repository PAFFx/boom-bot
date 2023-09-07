import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    video_device = LaunchConfiguration('video_device')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='video device to launch'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [1280,720],
                'camera_frame_id': 'camera_link_optical',
                'video_device': video_device
                }]
            )
        ])
