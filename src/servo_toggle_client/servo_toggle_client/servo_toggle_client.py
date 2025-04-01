from servo_toggle_client.nodes.servo_toggle_client import ServoToggleClient

def main():
    import rclpy
    rclpy.init()
    
    # Create node with parameters
    node = ServoToggleClient()
    
    # Declare the 'deactivate' parameter with default value True
    node.declare_parameter('deactivate', True)
    
    # Get the parameter value
    deactivate_param = node.get_parameter('deactivate').get_parameter_value().bool_value
    
    # Use the parameter value
    node.toggle_servos(deactivate=deactivate_param)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()