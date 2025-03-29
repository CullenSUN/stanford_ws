from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the command launcher node only once
    command_controller_node = Node(
        package='command_launcher',  # Replace with the actual package name
        executable='command_launcher_node',  # Replace with the executable name of your node
        name='command_launcher_node',
        output='screen',
        parameters=[]  # You can add parameter files or key-value pairs here
    )
    
    # Return the LaunchDescription with only the necessary nodes
    return LaunchDescription([
        command_controller_node
    ])
