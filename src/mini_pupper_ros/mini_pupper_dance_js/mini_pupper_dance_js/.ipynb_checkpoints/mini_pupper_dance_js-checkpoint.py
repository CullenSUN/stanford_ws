#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
from playsound import playsound
from mini_pupper_dance_js.Command_Sender import SocketSender  # Assuming Command_Sender is in your Python path
import threading



class MiniPupperDanceJS(Node):
    def __init__(self):
        super().__init__('mini_pupper_dance_js')

        # Declare a parameter for the dance file name
        self.declare_parameter('dance_file', 'dance_file')
        self.declare_parameter('music_folder', 'music_folder')
        self.dance_file_name = os.getenv('DANCE_FILE') or self.get_parameter('dance_file').get_parameter_value().string_value
        self.music_folder = self.get_parameter('music_folder').get_parameter_value().string_value

        dance_file_name = ""
        if self.dance_file_name :
            dance_file_name = os.path.basename(self.dance_file_name).replace('.py', '') 
            # Add the parent folder of DANCE_CONF to sys.path
            parent_folder = os.path.dirname(self.dance_file_name)
            if parent_folder not in sys.path:
                sys.path.append(parent_folder)
        else:
            raise EnvironmentError("Environment variable DANCE_CONF is not set. Please define it with the path to dance modules.")

        # Immediately process the dance file
        self.process_dance_file(dance_file_name)

    def process_dance_file(self, module_name):
        self.get_logger().info(f"Processing dance module: {module_name}")

        try:
            # Import the Python module dynamically
            dance_module = __import__(module_name)

            # Ensure 'level1' is defined in the module
            if not hasattr(dance_module, 'level1'):
                self.get_logger().error(f"'level1' is not defined in the module {module_name}.")
                return

            # Get the dance moves and send commands
            HOST = '127.0.0.1'
            level1 = dance_module.level1
            music_file = os.path.join(self.music_folder, dance_module.music_file)

            thread2 = threading.Thread(target=self.playmusic, args=(music_file, ))
            thread2.start()

            sender = SocketSender(HOST, level1)
            sender.command_dance()
        except ModuleNotFoundError:
            self.get_logger().error(f"Module {module_name} not found in path {DANCE_CONF}.")
        except Exception as e:
            self.get_logger().error(f"Error processing dance module {module_name}: {e}")


    def playmusic(self, file):
        # Define the command you want to run as a list of strings
        os.system("amixer -c 0 sset 'Headphone' 100%")
    
        if (len(file) >1):
            playsound(file)


def main(args=None):
    rclpy.init(args=args)
    node = MiniPupperDanceJS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
