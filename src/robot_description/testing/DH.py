import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import matplotlib.pyplot as plt

# Define the robot with MDH parameters
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.2),
        rtb.RevoluteMDH(alpha=-pi/2, d=-0.12),
        rtb.RevoluteMDH(a=0.25, d=0.1),
    ],
    name = "RRR_Robot"
)

robot.tool = SE3.Trans(0.28, 0.0, 0.0) * SE3.RPY(pi/2, 0, pi/2)

print(robot)

q = [0, -pi/2, 0]

T_e_home_confif = robot.fkine(q)
print(T_e_home_confif)
