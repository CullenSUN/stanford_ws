import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the dance file (if needed for customization)
    dance_file_arg = DeclareLaunchArgument(
        'dance_file',
        default_value='/home/ubuntu/minipupper/routines/dance1.py',
        description='Path to the dance file to execute'
    )

    # Define the dance node
    dance_node = Node(
        package='mini_pupper_dance_js',
        executable='mini_pupper_dance_js',
        name='mini_pupper_dance_js',
        parameters=[{
            'dance_file': LaunchConfiguration('dance_file'),
            'music_folder': '/home/ubuntu/minipupper/playlists',
        }]
    )

    return LaunchDescription([
        dance_file_arg,  # Declare the dance file argument
        dance_node       # Launch the dance node
    ])
