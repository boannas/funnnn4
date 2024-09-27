#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from math import pi, sqrt
from spatialmath import SE3
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState


class setup_node(Node):
    def __init__(self):
        super().__init__('setup_node')
        self.publisher_target = self.create_publisher(PoseStamped, '/target', 10)
        self.publisher_end = self.create_publisher(PoseStamped, '/end_effector', 10)

        self.subscription = self.create_subscription(JointState, '/joint_states', self.end_effector_cb, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  
        
        self.robot = rtb.DHRobot(
        [
            rtb.RevoluteMDH(d=0.2),
            rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
            rtb.RevoluteMDH(a=0.25, d=0.1),
        ],
        name = "RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)

        # self.get_logger().info(f"{self.robot}")

        
        
    def generate_random_target(self, min_val=0.03, max_val=0.53):
        while True:
            x = np.random.uniform(-max_val, max_val)
            y = np.random.uniform(-max_val, max_val)
            z = np.random.uniform(-max_val, max_val) + 0.2

            r = sqrt(x**2 + y**2 + z**2)
            if r >= min_val and r < max_val:
                self.get_logger().info(f"{x,y,z}")
                self.get_logger().info(f"{r}")
                return x,y,z
            else :
                pass
    
    def end_effector_cb(self, msg: JointState):
        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        # self.get_logger().info(f"{msg}")
        
        q = [q1, q2, q3]

        T_e = self.robot.fkine(q)
        # self.get_logger().info(f"{T_e}")
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0" 
        
        msg.pose.position.x = T_e.t[0]
        msg.pose.position.y = T_e.t[1]
        msg.pose.position.z = T_e.t[2]
        self.publisher_end.publish(msg)
    
    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0" 
        
        x, y, z = self.generate_random_target()
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        self.publisher_target.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = setup_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
