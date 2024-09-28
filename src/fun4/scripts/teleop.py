#!/usr/bin/python3

from example_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from math import pi
import math
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation as R
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
        
        self.q = np.array([0.0, 0.0, 0.0])
        self.q_vel = np.array([0.0, 0.0, 0.0])
        self.cmd_vel = np.array([0.0, 0.0, 0.0])
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(0.01, self.timer_callback)
        self.mode_sub = self.create_subscription(String, '/robot_mode', self.mode_cb, 10)

        self.current_mode = ""

        
    def mode_cb(self, msg:String):
        self.current_mode = msg.data
        # self.get_logger().info(f"{self.current_mode}")

        
    def cmd_cb(self, msg: Twist):        
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        # self.get_logger().info(f"{self.cmd_vel}")
        
    def timer_callback(self):
        if self.current_mode == "Teleoperation":

            J = self.robot.jacob0(self.q)
            J_trans = J[:3,:3]
            
            q_dot = np.linalg.pinv(J_trans).dot(self.cmd_vel)
            
            self.q += q_dot * 0.1
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            
            for i in range(len(self.q)):
                msg.position.append(self.q[i])
                msg.name.append(self.name[i])
                
            self.joint_pub.publish(msg)
            # self.get_logger().info(f"{msg}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
