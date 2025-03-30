import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
import subprocess
import os, time
import signal


import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ToggleServoClient:
    """
    A class to interact with the toggle_servo service to turn the servo on/off.
    """
    def __init__(self, node, timeout_sec=5.0):
        self.node = node
        self.client = node.create_client(SetBool, 'toggle_servo')

        # Wait for the service to be available within the specified timeout
        start_time = node.get_clock().now().seconds_nanoseconds()[0]
        while not self.client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting...')
            if node.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout_sec:
                node.get_logger().error(f'Service still not available after {timeout_sec} seconds!')
                return

    def send_request(self, deactivate, callback=None):
        """
        Sends a request to the toggle servo service and attaches a done callback for the response.
        """
        req = SetBool.Request()
        req.data = deactivate
        self.node.get_logger().info(f'Sending request to deactivate: {deactivate}')

        # Call the service asynchronously
        future = self.client.call_async(req)

        # Attach a done callback to handle the response
        future.add_done_callback(lambda f: self.handle_service_response(req, f, callback))

    def handle_service_response(self, request, future, callback):
        """
        Callback function to handle the service response and pass the result to the provided callback.
        """
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'Success: {response.message}')
                if callback is not None:
                    callback(request.data, True, response.message)  # Pass success and message to the callback
            else:
                self.node.get_logger().warn(f'Failed: {response.message}')
                if callback is not None:
                    callback(request.data, False, response.message)  # Pass failure and message to the callback
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {str(e)}')
            if callback is not None:
                callback(request.data, False, str(e))  # Pass failure and error message to the callback


class CommandLauncherNode(Node):
    """
    A node that listens for commands to control the robot's servo and execute various tasks.
    """
    def __init__(self):
        super().__init__('command_launcher_node')
        #self.current_servo = None  # Initially, no servo is set
        self.current_servo = "champ"
        
        self.create_subscription(String, '/command_topic', self.command_callback, 10)
        
        # Subscription to image topic (not used, but can be expanded later)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Initialize the ToggleServoClient to control the servo
        self.servo_toggle = ToggleServoClient(self)

        # Clients for the dance command service
       # self.dance_command_cli = self.create_client(Trigger, 'dance_command')

        self.active_process = None
        self.get_logger().info("Command Launcher Node started.")

    def image_callback(self, msg):
        """
        Placeholder callback for image processing (not yet implemented).
        """
        pass


    def command_callback(self, msg):
        """
        Callback for the /command_topic. Decodes commands and starts respective actions.
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")
    
        # Stop the currently running launch file if any
        if self.active_process:
            self.stop_current_launch()
    
        # Execute the appropriate command based on the input
        if command == "reset":
            self.servo_toggle.send_request(False)

        elif command == "dance1":
            if self.current_servo == "champ":
                # Deactivate champ servo before switching to the next one
                
               self.servo_toggle.send_request(True, self.handle_servo_response)
                
    
            # if success:
            #     # Change the current servo to stanford
            #     self.current_servo = "stanford"
                
            #     # Now that we are sure the service call succeeded, launch the dance1.launch.py file
            #     self.launch_file("mini_pupper_dance_js", "dance1.launch.py")
    
            #     # Reactivate champ servo after launching the dance
            #     self.servo_toggle.send_request(False)  # Turn champ controller back on
            #     self.current_servo == "champ"
            # else:
            #     self.get_logger().error("Failed to deactivate champ servo. Dance1 launch aborted.")
    
        elif command == "dance2":
            success = False
            if self.current_servo == "champ":
                success = self.servo_toggle.send_request(True, self.handle_servo_response)
    
            if success:
                self.current_servo = "stanford"
                self.launch_file("mini_pupper_dance_js", "dance2.launch.py")
                self.servo_toggle.send_request(False, self.handle_servo_response)  # Turn champ controller back on
                self.current_servo == "champ"
            else:
                self.get_logger().error("Failed to deactivate champ servo. Dance2 launch aborted.")
    
        elif command == "stop":
            if self.current_servo == "champ":
                success = self.servo_toggle.send_request(True, self.handle_servo_response)
    
            self.launch_file("stop.launch.py")
        elif command == "line_follow":
            self.get_logger().info(f"Received current_servo: {self.current_servo}")
            self.current_servo = "champ"
            self.launch_file("mini_pupper_recognition", "line_follow.launch.py")
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def handle_servo_response(self, requested_deactivate, success, message):
        """
        Callback function to handle the result of the service response.
        """
        if success:
            self.get_logger().info(f"Servo toggled successfully: {message}")
            if requested_deactivate:
                self.current_servo = "stanford"
            else: 
                self.current_servo = "champ"
        else:
            self.get_logger().error(f"Failed to toggle servo: {message}")

    def launch_file(self, package_name, launch_file_name):
        """
        Launches a specified ROS 2 launch file.
        """
        try:
            self.get_logger().info(f"Launching {launch_file_name}")
            self.active_process = subprocess.Popen(
                ["ros2", "launch", package_name, launch_file_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # This allows us to kill the process group later
            )
        except Exception as e:
            self.get_logger().error(f"Failed to launch {launch_file_name}: {e}")

    
    def stop_current_launch(self):
        """
        Stops the currently active launch file process.
        """
        if self.active_process:
            try:
                self.get_logger().info("Stopping current launch file.")
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)  # Kill the process group
                self.active_process.wait()  # Wait for the process to terminate
                self.active_process = None
                self.get_logger().info("Current launch file stopped.")
            except Exception as e:
                self.get_logger().error(f"Failed to stop current launch file: {e}")

    def stop_launch_process(self, launch_file):
        """
        Helper function to stop a given launch file.
        """
        self.launch_file("mini_pupper_driver", launch_file)

    def shutdown(self):
        """
        Shutdown procedure for the node. Stops any active processes.
        """
        self.get_logger().info("Shutting down Command Launcher Node.")
        self.stop_current_launch()


def main(args=None):
    """
    Main entry point for the node.
    """
    rclpy.init(args=args)
    node = CommandLauncherNode()
    
    try:
        # Spin the node to keep it active and listen to callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
