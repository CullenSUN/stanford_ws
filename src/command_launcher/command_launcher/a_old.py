import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
import subprocess
import os
import signal

class ToggleServoClient(Node):
    def __init__(self):
        super().__init__('toggle_servo_client')
        self.client = self.create_client(SetBool, 'toggle_servo')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')



    def send_request(self, deactivate):
        req = SetBool.Request()
        req.data = deactivate
        self.get_logger().info('Sending request...')
    
        future = self.client.call_async(req)
        timeout = 3.0  # Timeout in seconds
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        while not future.done():
            if self.get_clock().now().seconds_nanoseconds()[0] - start_time > timeout:
                self.get_logger().error("Service call timed out!")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
    
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
                
                return True
            else:
                self.get_logger().warn(f'Failed: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False



class CommandLauncherNode(Node):
    def __init__(self):
        super().__init__('command_launcher_node')
        self.current_servo = None
        self.subscription = self.create_subscription(
            String, '/command_topic', self.command_callback, 10)

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        # use this one to toggle champ servo, when it is called, the champ servo will be deactiviate
        self.servo_toggle =  ToggleServoClient()


        self.servo_cli = self.create_client(Trigger, 'dance_command')
        self.servoserver_cli = self.create_client(Trigger, 'dance_command')

        self.active_process = None
        self.get_logger().info("Command Launcher Node started.")

    def image_callback(self, msg):
        pass

    
    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")
        

        # Stop the currently running launch file if any
        if self.active_process:
            self.stop_current_launch()

        # Start the appropriate launch file
        if command == "dance1":
            # if the servo is using champ, we need to deactivate and turn on the stanford
            success = False
            if (self.current_servo == "champ"):
                # since we need to
                success =  self.servo_toggle.send_request(True)

            self.get_logger().info(f"Received staus: {success}")
            
            if (success):
                self.get_logger().info("deactiviate champ servo now")
                self.current_servo = "stanford"
                self.launch_file("mini_pupper_dance_js", "dance1.launch.py")

            self.servo_toggle.send_request(False)  # turn the champ controller on
                
        elif command == "dance2":
            success = False
            if (self.current_servo == "champ"):
                success =  self.servo_toggle.send_request(True)

            if (success):
                self.current_servo = "stanford"
                self.launch_file("mini_pupper_dance_js", "dance2.launch.py")


            self.servo_toggle.send_request(False)  # turn the champ controller on

        elif command == "stop":
            if (self.current_servo == "champ"):
                success =  self.servo_toggle.send_request(True)
                
            self.launch_file("stop.launch.py")
        elif command == "line_follow":
            self.get_logger().info(f"Received current_servo: {self.current_servo}")



            self.current_servo = "champ"
            #self.launch_file("mini_pupper_driver", "servo_interface.launch.py")
            self.launch_file("mini_pupper_recognition", "line_follow.launch.py")
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def launch_file(self, package_name, launch_file_name):
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
        if self.active_process:
            try:
                self.get_logger().info("Stopping current launch file.")
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)  # Kill the process group
                self.active_process.wait()  # Wait for the process to terminate
                self.active_process = None
                self.get_logger().info("Current launch file stopped.")
            except Exception as e:
                self.get_logger().error(f"Failed to stop current launch file: {e}")

    def stop_node_callback(self, request, response):
        self.get_logger().info("Service call received to stop the node.")
        if self.active_process:
            self.stop_current_launch()
            response.success = True
            response.message = "Node stopped successfully."
        else:
            response.success = False
            response.message = "No active node to stop."
        return response

    def shutdown(self):
        self.get_logger().info("Shutting down Command Launcher Node.")
        self.stop_current_launch()


def main(args=None):
    rclpy.init(args=args)
    node = CommandLauncherNode()
    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
