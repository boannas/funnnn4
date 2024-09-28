#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from fun4_interfaces.srv import Mode 
from std_msgs.msg import String


class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.srv = self.create_service(Mode, '/set_mode', self.handle_mode_request)

        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.create_timer(0.01, self.timer_callback)

        self.current_mode = 0

    def handle_mode_request(self, request, response):
        mode = request.mode.data
        # self.get_logger().info(f"mode ={mode}")

        if mode in [1, 2, 3]:
            self.current_mode = mode
            response.success = True
            if mode == 1:
                self.get_logger().info(f"Mode set to Inverse kinematics")
            elif mode == 2:
                self.get_logger().info(f"Mode set to Teleoperation")
            elif mode == 3:
                self.get_logger().info(f"Mode set to Autonomous")
        else:
            response.success = False
            self.get_logger().warn(f"Invalid mode: {mode}")

        return response

    def publish_mode(self, mode):
        mode_msg = String()
        if mode == 1:
            mode_msg.data = "Inverse Kinematics"
        elif mode == 2:
            mode_msg.data = "Teleoperation"
        elif mode == 3:
            mode_msg.data = "Autonomous"

        self.mode_pub.publish(mode_msg)
        
    def timer_callback(self):
        self.publish_mode(self.current_mode)


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
