#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ServoToggleClient(Node):
    def __init__(self):
        super().__init__('servo_toggle_client')
        
        # Declare parameters
        self.declare_parameter('deactivate', True)
        
        # Create service client
        self.client = self.create_client(SetBool, 'toggle_servo')
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def toggle_servos(self, deactivate=None):
        """Toggle servos using parameter if None, otherwise use provided value"""
        if deactivate is None:
            deactivate = self.get_parameter('deactivate').value
        
        req = SetBool.Request()
        req.data = deactivate
        
        self.get_logger().info(f'{"Deactivating" if deactivate else "Activating"} servos...')
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().warn(f'Failed: {response.message}')
            return response.success
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ServoToggleClient()
        
        # Get parameter value
        deactivate = node.get_parameter('deactivate').value
        
        # Execute toggle
        success = node.toggle_servos(deactivate)
        
        # Shutdown
        node.destroy_node()
        rclpy.shutdown()
        
        return 0 if success else 1
        
    except Exception as e:
        print(f"Error in servo toggle client: {str(e)}")
        return 1

if __name__ == '__main__':
    main()