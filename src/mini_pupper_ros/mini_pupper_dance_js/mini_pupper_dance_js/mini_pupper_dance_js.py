#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import threading
from playsound import playsound
from mini_pupper_dance_js.Command_Sender import SocketSender
from rclpy.executors import MultiThreadedExecutor

class MiniPupperDanceJS(Node):
    def __init__(self):
        super().__init__('mini_pupper_dance_js')
        
        # Declare parameters
        self.declare_parameter('dance_file', '')
        self.declare_parameter('music_folder', '')
        
        # Retrieve parameters
        self.dance_file_name = os.getenv('DANCE_FILE') or self.get_parameter('dance_file').get_parameter_value().string_value
        self.music_folder = self.get_parameter('music_folder').get_parameter_value().string_value

        if not self.dance_file_name:
            self.get_logger().error("DANCE_FILE is not set. Please define it as an environment variable or parameter.")
            raise RuntimeError("DANCE_FILE not set")  # This will trigger shutdown in main()

    def run_dance(self):
        """Main dance execution logic"""
        module_name = os.path.basename(self.dance_file_name).replace('.py', '')
        parent_folder = os.path.dirname(self.dance_file_name)
        
        if parent_folder and parent_folder not in sys.path:
            sys.path.append(parent_folder)

        self.get_logger().info(f"Processing dance module: {module_name}")

        try:
            dance_module = __import__(module_name)
            
            if not hasattr(dance_module, 'level1'):
                self.get_logger().error(f"'level1' is not defined in {module_name}.")
                return False

            HOST = '127.0.0.1'
            level1 = dance_module.level1
            music_file = os.path.join(self.music_folder, getattr(dance_module, 'music_file', ''))

            if not os.path.exists(music_file):
                self.get_logger().error(f"Music file {music_file} not found.")
                return False

            # Start music thread
            music_thread = threading.Thread(target=self.play_music, args=(music_file,))
            music_thread.start()

            # Execute dance
            sender = SocketSender(HOST, level1)
            sender.command_dance()

            # Wait for completion
            music_thread.join()
            self.get_logger().info("Dance completed successfully")
            return True

        except Exception as e:
            self.get_logger().error(f"Error in dance execution: {e}")
            return False

    def play_music(self, file_path):
        """Play music using system sound settings"""
        os.system("amixer -c 0 sset 'Headphone' 70%")
        if os.path.exists(file_path):
            playsound(file_path)

def main(args=None):
    rclpy.init(args=args)
    node = MiniPupperDanceJS()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Run the dance routine
        success = node.run_dance()
        
        # Shutdown regardless of success/failure
        node.get_logger().info("Shutting down node...")
        executor.shutdown()
        
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()