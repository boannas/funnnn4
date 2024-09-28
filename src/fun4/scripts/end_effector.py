#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi


class EndEffectorNode(Node):
    def __init__(self):
        super().__init__('end_effector_node')

        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(d=0.2),
                rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
                rtb.RevoluteMDH(a=0.25, d=0.1),
            ],
            name="RRR_Robot"
        )
        self.robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)

        self.publisher_end = self.create_publisher(PoseStamped, '/end_effector', 10)

        self.subscription = self.create_subscription(JointState, '/joint_states', self.end_effector_cb, 10)



    def end_effector_cb(self, msg: JointState):
        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q = [q1, q2, q3]

        T_e = self.robot.fkine(q)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "link_0"

        pose_msg.pose.position.x = T_e.t[0]
        pose_msg.pose.position.y = T_e.t[1]
        pose_msg.pose.position.z = T_e.t[2]

        self.publisher_end.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
