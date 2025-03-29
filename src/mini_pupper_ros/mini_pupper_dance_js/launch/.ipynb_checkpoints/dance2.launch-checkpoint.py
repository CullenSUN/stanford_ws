from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the dance file
    dance_file_arg = DeclareLaunchArgument(
        'dance_file',
        default_value='dance_file',
        description='Name of the dance file (Python module) to execute'
    )

    
    # Create the Node object for the dance subscriber
    dance_subscriber_node = Node(
        package='mini_pupper_dance_js',
        executable='mini_pupper_dance_js',
        name='mini_pupper_dance_js',
        parameters=[{
                'dance_file':'/home/ubuntu/minipupper/routines/dance1.py',
                'music_folder':'/home/ubuntu/minipupper/playlists',
                    }]
    )

    return LaunchDescription([
        dance_file_arg,
        dance_subscriber_node
    ])
