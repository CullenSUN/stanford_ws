from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_toggle_client',
            executable='servo_toggle_node',
            name='servo_toggle_client',
            output='screen',
            parameters=[{
                'deactivate': False  # Default action
            }]
        )
    ])