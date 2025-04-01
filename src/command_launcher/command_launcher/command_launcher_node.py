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

    def send_request(self, deactivate, timeout_sec=10.0):
        """
        Sends a request to the toggle servo service and waits for a successful response.
        """
        req = SetBool.Request()
        req.data = deactivate
        self.node.get_logger().info(f'Sending request to deactivate: {deactivate}')

        future = self.client.call_async(req)
        start_time = self.node.get_clock().now().seconds_nanoseconds()[0]

        while not future.done():
            if self.node.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout_sec:
                self.node.get_logger().error("Service call timed out!")
                return False  # Timeout occurred
            rclpy.spin_once(self.node, timeout_sec=0.1)  # Process callbacks while waiting

        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'Success: {response.message}')
                return True
            else:
                self.node.get_logger().warn(f'Failed: {response.message}')
                return False
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {str(e)}')
            return False




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
        if command == "dance1":
            success = False
            if self.current_servo == "champ":
                # Deactivate champ servo before switching to the next one
                
                success = self.servo_toggle.send_request(True, 10)
                #success = True
                
    
            if success:
                # Change the current servo to stanford
                self.current_servo = "stanford"
                
                # Now that we are sure the service call succeeded, launch the dance1.launch.py file
                #self.launch_file("mini_pupper_dance_js", "dance1.launch.py")
                succ, msg, rc = self.launch_file("mini_pupper_dance_js", "dance1.launch.py", timeout_sec=15, check_completion=True)
                # Reactivate champ servo after launching the dance
                if (succ):
                    self.servo_toggle.send_request(False)  # Turn champ controller back on
                    self.current_servo == "champ"
            else:
                self.get_logger().error("Failed to deactivate champ servo. Dance1 launch aborted.")
    
        elif command == "dance2":
            success = False
            if self.current_servo == "champ":
                success = self.servo_toggle.send_request(True)
    
            if success:
                self.current_servo = "stanford"
                self.launch_file("mini_pupper_dance_js", "dance2.launch.py")
                self.servo_toggle.send_request(False)  # Turn champ controller back on
                self.current_servo == "champ"
            else:
                self.get_logger().error("Failed to deactivate champ servo. Dance2 launch aborted.")
    
        elif command == "stop":
            if self.current_servo == "champ":
                success = self.servo_toggle.send_request(True)
    
            self.launch_file("stop.launch.py")
        elif command == "line_follow":
            self.get_logger().info(f"Received current_servo: {self.current_servo}")
            self.current_servo = "champ"
            self.launch_file("mini_pupper_recognition", "line_follow.launch.py")
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def launch_file(self, package_name, launch_file_name, timeout_sec=15.0, check_completion=False):
        """
        Launches a ROS 2 launch file with comprehensive status monitoring.
        
        Args:
            package_name: Name of the ROS package
            launch_file_name: Name of the launch file
            timeout_sec: Timeout for process startup verification
            check_completion: If True, waits for process completion
        
        Returns:
            tuple: (success: bool, message: str, return_code: Optional[int])
                   return_code is None if process is still running
        """
        try:
            self.get_logger().info(f"Launching {package_name}/{launch_file_name}")
            self.active_process = subprocess.Popen(
                ["ros2", "launch", package_name, launch_file_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            # Immediate status check
            if self.active_process.poll() is not None:
                error = self.active_process.stderr.read().strip()
                return (False, f"Process terminated immediately: {error}", self.active_process.returncode)
            
            # Verify process stays running
            start_time = time.time()
            while time.time() - start_time < timeout_sec:
                if self.active_process.poll() is not None:
                    error = self.active_process.stderr.read().strip()
                    return (False, f"Process crashed during startup: {error}", self.active_process.returncode)
                time.sleep(0.1)
            
            # Option 1: Return immediately after successful launch
            if not check_completion:
                return (True, "Process launched successfully", None)
            
            # Option 2: Wait for completion
            try:
                stdout, stderr = self.active_process.communicate(timeout=timeout_sec)
                if self.active_process.returncode == 0:
                    return (True, "Process completed successfully", 0)
                else:
                    return (False, f"Process failed: {stderr.strip()}", self.active_process.returncode)
            except subprocess.TimeoutExpired:
                return (True, "Process still running after timeout", None)
                
        except Exception as e:
            error_msg = f"Launch failed: {str(e)}"
            if hasattr(self, 'active_process') and self.active_process:
                self.active_process.terminate()
            return (False, error_msg, None)    

    
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
