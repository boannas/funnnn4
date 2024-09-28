#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import Target  # Import your custom Target service
import roboticstoolbox as rtb
from math import pi, sqrt
from spatialmath import SE3
from std_msgs.msg import String


class autoNode(Node):
    def __init__(self):
        super().__init__('auto_node')
        
        # Define the robot with DH parameters
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
                rtb.RevoluteMDH(a=0.25, d=0.1),
            ],
            name="RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)
        
        # Initial parameters
        self.dt = 0.01
        self.q = [0.0, 0.0, 0.0]  # Current joint states
        self.q_sol = [0.0, 0.0, 0.0]  # Target joint states
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.current_mode = ""
        self.ready = True  

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(self.dt, self.sim_loop)
        self.mode_sub = self.create_subscription(String, '/robot_mode', self.mode_cb, 10)
        self.publisher_target = self.create_publisher(PoseStamped, '/target', 10)

        self.srv = self.create_service(Target, '/auto_target', self.handle_set_target)

    def mode_cb(self, msg: String):
        self.current_mode = msg.data

    def handle_set_target(self, request, response):
        if self.current_mode == "Autonomous":
            if self.ready:  
                target = request.target
                x, y, z = target.x, target.y, target.z
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "link_0"
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = z
                self.publisher_target.publish(msg)
                
                if self.is_in_workspace(x, y, z):
                    T_0e = SE3.Trans(x, y, z)
                    q_sol = self.robot.ikine_LM(T_0e, mask=[1, 1, 1, 0, 0, 0])
                    self.q_sol = q_sol.q  
                    response.success = True
                    self.ready = False  
                    self.get_logger().info(f"Autonomous mode: Moving to target at {x}, {y}, {z}")
                else:
                    response.success = False
                    self.get_logger().error(f"Target out of workspace: {x}, {y}, {z}")
            else:
                response.success = False
        else:
            response.success = False
            # self.get_logger().warn(f"Current mode is: {self.current_mode}, not 'Autonomous'")

        return response

    def is_in_workspace(self, x, y, z):
        min_val = 0.03
        max_val = 0.53
        distance = sqrt(x**2 + y**2 + z**2)
        return min_val <= distance < max_val

    def sim_loop(self):
        if self.current_mode == "Autonomous":
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"

            for i in range(len(self.q)):
                self.q[i] += (self.q_sol[i] - self.q[i]) * (self.dt * 2)
                msg.position.append(self.q[i])
                msg.name.append(self.name[i])

            self.joint_pub.publish(msg)

            if self.has_reached_target():
                self.ready = True  

    def has_reached_target(self):
        tolerance = 0.01  
        for i in range(len(self.q)):
            if abs(self.q[i] - self.q_sol[i]) > tolerance:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = autoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
