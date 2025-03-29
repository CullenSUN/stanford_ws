from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the share directory of your package
    package_share_directory = get_package_share_directory('mini_pupper_dance_js')

    # Specify the file path relative to the package's share directory
    dance_file_path = os.path.join(package_share_directory, 'routines', 'dance1.py')
    music_file_path = os.path.join(package_share_directory, 'playlists')

    # Declare launch argument with default value
    dance_file_arg = DeclareLaunchArgument(
        'dance_file',
        default_value=dance_file_path,
        description='Path to the dance file to execute'
    )

    # Define the command launcher node
    command_controller_node = Node(
        package='command_launcher',
        executable='command_launcher_node',
        name='command_launcher_node',
        output='screen',
        parameters=[{
            'dance_file': launch.substitutions.LaunchConfiguration('dance_file'),
            'music_folder': music_file_path,
        }]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        dance_file_arg,
        command_controller_node
    ])
