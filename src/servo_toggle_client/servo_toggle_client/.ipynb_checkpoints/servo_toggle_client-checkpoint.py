from servo_toggle_client.nodes.servo_toggle_client import ServoToggleClient

def main():
    import rclpy
    rclpy.init()
    node = ServoToggleClient()
    node.toggle_servos(deactivate=True)  # Default action
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()