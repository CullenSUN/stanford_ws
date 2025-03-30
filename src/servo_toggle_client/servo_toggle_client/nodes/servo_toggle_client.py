#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class ServoToggleClient(Node):
    def __init__(self):
        super().__init__('servo_toggle_client')
        self.client = self.create_client(SetBool, 'toggle_servo')
        
        # Quality control parameters
        self.timeout_sec = 5.0
        self.max_retries = 3
        self.retry_delay = 1.0
        
        # Block until service is available
        if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().error("Service not available, shutting down")
            self.destroy_node()
            rclpy.shutdown()
            return
        
        # Execute and self-terminate
        self.execute_and_terminate(deactivate=True)

    def execute_and_terminate(self, deactivate=True):
        """Blocking execution with automatic termination"""
        result = False
        
        for attempt in range(self.max_retries):
            try:
                req = SetBool.Request()
                req.data = deactivate
                
                future = self.client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"Servo {'deactivated' if deactivate else 'activated'} successfully")
                        result = True
                        break
                    else:
                        self.get_logger().warning(f"Service reported failure: {response.message}")
                else:
                    self.get_logger().warning(f"Timeout on attempt {attempt + 1}")
                
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay)
                    
            except Exception as e:
                self.get_logger().error(f"Attempt {attempt + 1} failed: {str(e)}")
        
        # Self-terminate
        self.destroy_node()
        rclpy.shutdown()
        
        # Exit code for process control
        exit(0 if result else 1)

def main(args=None):
    rclpy.init(args=args)
    ServoToggleClient()  # Will self-terminate after execution

if __name__ == '__main__':
    main()