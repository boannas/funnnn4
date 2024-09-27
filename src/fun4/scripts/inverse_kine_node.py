#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from sensor_msgs.msg import JointState




class inverse_kine_node(Node):
    def __init__(self):
        super().__init__('inverse_kine_node')
        self.subscription = self.create_subscription(PoseStamped, '/target', self.fine_ike, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.robot = rtb.DHRobot(
        [
            rtb.RevoluteMDH(d=0.2),
            rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
            rtb.RevoluteMDH(a=0.25, d=0.1),
        ],
        name = "RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)
        
        self.q = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]


        
    def fine_ike(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        T_0e = SE3.Trans(x,y,z)
        self.get_logger().info(f"{x,y,z}")

        q_sol = self.robot.ikine_LM(T_0e, mask=[1,1,1,0,0,0])  
        q_sol_values = q_sol.q 
        T_inverse_0e = self.robot.fkine(q_sol_values)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom" 

        self.q[0] = q_sol_values[0]
        self.q[1] = q_sol_values[1]
        self.q[2] = q_sol_values[2]
        
        for i in range(len(self.q)):
            msg.position.append(self.q[i])
            msg.name.append(self.name[i])
        self.joint_pub.publish(msg)
        # self.get_logger().info(f"{T_inverse_0e.t}")

        



def main(args=None):
    rclpy.init(args=args)
    node = inverse_kine_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
