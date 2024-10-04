# Fun 4 
## Introduction 
Fun4 is a project that integrated in field of ROS2 (Robotic Operate System) and Kinematics to make a robot arm simulation.

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture](#system-archtecture)
- [Installation and setup](#installation-and-setup)
- [Usage](#usage)

## Project Overview

The Fun4 project consists of three main components:

1. **Inverse Kinematics Mode**

   Control robot arm by giving target position (x, y, z) and the robot arm will go to the target if it possible but if are not possible or not in robot workspace it will notify that target is out of workspace. If target able to reach it will return response True and make robot move to target but if target not in workspace it will notify and don't move.

2. **Tele-operation Mode**

    Control robot arm by /cmd_vel from teleop_twist_keyboard and compute for the joint angular velocity and then end-effector will move. In this teleop has 2 mode to select /cmd_vel with respect to end-effector and /cmd_vel with respect to base. This can notify you if robot arm close to singularity and it can't go closer to singularity any more.

3. **Autonomous**

   This node will subscribe random node that will random target position in workspace and use inverse kinematics to compute how to chang degree in each joint and it will go to the target position, if robot reach to target position it should be sent response to acknowledge to random node and wait for new target and iteration.


## System Archtecture

### Nodes

-   **end_effector** : compute robot arm cofiguration and publish end-effector position to rviz
-   **random_node** : random target for autonomous mode to reach target
-   **schedular** : wait for user input mode selection, at begin it should inverse kinematics(mode1) but you can select mode by service call
-   **inverse_kine** : collect target point(x, y, z) by service call and use inverse kinematic to compute how can each joint rotate to reach target point and it will publish joint_state to control robot 
-   **teleop** :  collect cmd_vel input from teleop_twist_keyboard node and compute to publish joint_state to control robot, in this mode it has 2 modes inside cmd_vel reference from base frame or cmd_vel reference from end-effector frame
-   **auto** :  subscribe target point from random node and try to reach the point, if it close to target (< threshold) it will sent noti and recieve next target point and try to go to next target


### Topics
-   **/joint_states** : for control each joint radian
-   **/cmd_vel** : from teleop_twist_keyboard to control velocity of end-effector
-   **/robot_mode** : to select robot mode
-   **/end_effect** : visualize end-effector in rviz
-   **/target** : set target for robot to reach point

### Services
-   **/auto_target** : request target(x, y, z) and response success
-   **/set_target** : request target(x, y, z) and response success
-   **/set_mode** : request mode(1, 2, 3, 4) and response success


## Installation and Setup

### Step 1: Clone the repository

```bash
git clone https://github.com/boannas/funnnn4.git
```

### Step 2: Build the Package
```bash
cd funnnn4 && colcon build
```
### Step 3: Build turtlesim plus
```bash
source dependencies_install.bash && colcon build --packages-select fun4 fun4_interfaces
```
### Step 4: Source the Setup File
```bash
source ~/funnnn4/install/setup.bash
```
### Step 5: (Optional) Add to .bashrc
```bash
echo "source ~/funnnn4/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

### Step 6: Install and setup python package
```bash
pip3 install numpy==1.23.5
pip3 install roboticstoolbox-python
```



## Usage
- ### Run seperately node

```bash
ros2 run fun4 {Node_name}
```

- ### run teleop_twist_keyboard node
  can use only in mode 2(teleop)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

- ### Launch fun4 project 
  Firstly, it begin at inverse-kinematics mode you can change mode by following code

```bash
ros2 launch fun4 fun.launch.py 
```


- ### Set robot mode 
  1: inverse kinematics, 2: teleop reference from base, 
  
  3: teleop reference from end-effector, 4: Autonomous 
```bash
ros2 service call /set_mode fun4_interfaces/srv/Mode "mode: 
  data: {mode}"
```

- ### Set target for inverse kinematics mode
  change x, y, z to your desire target point
```bash
ros2 service call /set_target fun4_interfaces/srv/Target "target:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

- ### Display singularity check from teleop mode
```bash
ros2 topic echo /singularity_check 
```
