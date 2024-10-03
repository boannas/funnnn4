#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
                rtb.RevoluteMDH(a=0.25, d=0.1),
            ],
            name="RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)
        
        self.q = np.array([0.05, 0.2, 0.2]) 
        self.q_vel = np.array([0.0, 0.0, 0.0])  
        self.cmd_vel = np.array([0.0, 0.0, 0.0]) 
        self.name = ["joint_1", "joint_2", "joint_3"]  
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.mode_sub = self.create_subscription(String, '/robot_mode', self.mode_cb, 10)
        
        self.create_timer(0.01, self.timer_callback)

        self.current_mode = ""
        self.manipulability_threshold = 0.00001  
        self.damping_factor = 0.1

    def mode_cb(self, msg: String):
        self.current_mode = msg.data

    def cmd_cb(self, msg: Twist):        
        self.cmd_vel = np.array([msg.linear.x * 0.1, msg.linear.y * 0.1, msg.linear.z * 0.1])

    def timer_callback(self):
        if self.current_mode == "Teleoperation_b":
            J = self.robot.jacob0(self.q) 
        elif self.current_mode == "Teleoperation_e":
            J = self.robot.jacobe(self.q)  
        else:
            return 

        J_reduce = J[:3, :3]  
        manipulability = np.sqrt(np.linalg.det(J_reduce @ J_reduce.T))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        if manipulability > self.manipulability_threshold:
            q_dot = np.linalg.inv(J_reduce) @ self.cmd_vel
        else:
            #SVD-based damping
            u, s, v = np.linalg.svd(J_reduce)
            for i in range(len(s)):
                if s[i] < 0.005:  # Threshold 
                    s[i] = 0
                else:
                    s[i] = 1.0 / float(s[i])  
            J_damped = np.dot(v.T, np.dot(np.diag(s), u.T)) 
            q_dot = J_damped @ self.cmd_vel
        
        self.q += q_dot * 0.1
        for i in range(len(self.q)):
            msg.position.append(self.q[i])
            msg.name.append(self.name[i])

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
