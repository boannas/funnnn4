#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
                rtb.RevoluteMDH(a=0.25, d=0.1),
            ],
            name="RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)
        
        self.q = [0.0, 0.0, 0.0]  
        self.name = ["joint_1", "joint_2", "joint_3"]
        
        self.dt = 0.1  # 100ms

    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]

        v_e = np.array([linear_velocity[0], linear_velocity[1], linear_velocity[2], angular_velocity[0], angular_velocity[1], angular_velocity[2]])

        J = self.robot.jacob0(self.q)

        q_dot = np.linalg.pinv(J) @ v_e

        self.q = [self.q[i] + q_dot[i] * self.dt for i in range(len(self.q))]

        self.publish_joint_state(q_dot)

    def publish_joint_state(self, q_dot):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        for i in range(len(self.q)):
            msg.position.append(self.q[i])
            msg.velocity.append(q_dot[i])
            msg.name.append(self.name[i])

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
