#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import Target  # Import your custom Target service
from math import pi, sqrt

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')

        # Publisher to publish random target positions
        
        # Create a client to request the autoNode for status
        self.target_client = self.create_client(Target, '/auto_target')

        # Ensure that the service is available before sending a request


        # Timer to periodically generate and publish targets (every 3 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)  

        self.target_reached = False  # State flag to track if target is reached

    def generate_random_target(self, min_val=0.03, max_val=0.53):
        """Generate a random target within the robot's workspace, ensuring the point remains valid."""
        while True:
            u = np.random.uniform()
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)

            # Generate the radius with proper scaling
            r = (u * (max_val**3 - min_val**3) + min_val**3)**(1/3)

            # Calculate the Cartesian coordinates
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)

            # Apply the z-offset without exceeding the workspace bounds
            z += 0.2

            # Check if the total distance is within workspace limits
            total_distance = sqrt(x**2 + y**2 + z**2)

            if min_val <= total_distance <= max_val:
                # self.get_logger().info(f"Generated target: {x}, {y}, {z}, total distance: {total_distance}")
                return x, y, z
            
    def timer_callback(self):
        if not self.target_reached:  # Only proceed if target was reached
            x, y, z = self.generate_random_target()

            # Create PoseStamped message with the new random target
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "link_0"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            self.send_target_request(msg.pose.position)
        else:

            self.get_logger().info("Waiting for autoNode to reach the previous target...")

    def send_target_request(self, target_point):
        req = Target.Request()
        req.target = target_point 

        self.future = self.target_client.call_async(req)
        self.future.add_done_callback(self.target_response_callback)

    def target_response_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.target_reached = False 
                
            else:
                self.target_reached = False

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
