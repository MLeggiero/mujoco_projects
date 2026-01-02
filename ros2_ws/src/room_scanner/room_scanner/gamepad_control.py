#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class GamepadControl(Node):
    """
    Control the Room Scanner using a Gamepad (via joy_node).
    Mappings (Xbox Style):
    - A (0): Resume Scanning
    - B (1): Pause Scanning
    - X (2): Save Map
    - Y (3): Reset Map
    """
    def __init__(self):
        super().__init__('gamepad_control')

        # Subscriptions
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Clients
        self.resume_client = self.create_client(Empty, '/rtabmap/resume')
        self.pause_client = self.create_client(Empty, '/rtabmap/pause')
        self.save_map_client = self.create_client(Empty, '/rtabmap/save_map')
        self.reset_client = self.create_client(Empty, '/rtabmap/reset')

        # Debounce/State tracking
        self.last_buttons = []
        
        self.get_logger().info('Gamepad Control Node Initialized')

    def joy_callback(self, msg):
        # Initialize button state if first run
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)
            return

        # Check for rising edge (button press)
        # Button A (Index 0) -> Resume
        if msg.buttons[0] and not self.last_buttons[0]:
            self.call_service(self.resume_client, "Resume")
            
        # Button B (Index 1) -> Pause
        if msg.buttons[1] and not self.last_buttons[1]:
            self.call_service(self.pause_client, "Pause")

        # Button X (Index 2) -> Save
        if msg.buttons[2] and not self.last_buttons[2]:
            self.call_service(self.save_map_client, "Save Map")

        # Button Y (Index 3) -> Reset
        if msg.buttons[3] and not self.last_buttons[3]:
            self.call_service(self.reset_client, "Reset")

        # Update last state
        self.last_buttons = msg.buttons

    def call_service(self, client, name):
        if client.wait_for_service(timeout_sec=0.1):
            client.call_async(Empty.Request())
            self.get_logger().info(f'Gamepad Command: {name}')
        else:
            self.get_logger().warn(f'Service for {name} unavailable')

def main(args=None):
    rclpy.init(args=args)
    node = GamepadControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
