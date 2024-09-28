# Fun 4 
## Introduction 
Fun4 is a project that integrated in field of ROS2 (Robotic Operate System) and Kinematics to make a robot arm simulation.

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture](#system-archtecture)
- [Installation and setup](#installation-and-setup)
- [Usage](#usage)
- [Testing and Result](#testing-and-result)

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


### Topics


### Services



## Installation and Setup

### Step 1: Clone the repository

```bash
git clone https://github.com/ongsa12342/FunnyTurtlePlus.git
```

### Step 2: Build the Package
```bash
cd FunnyTurtlePlus && colcon build
```
### Step 3: Build turtlesim plus
```bash
source dependencies_install.bash && colcon build --packages-select turtlesim_plus turtlesim_plus_interfaces
```
### Step 4: Source the Setup File
```bash
source ~/FunnyTurtlePlus/install/setup.bash
```
### Step 5: (Optional) Add to .bashrc
```bash
echo "source ~/FunnyTurtlePlus/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## Usage
- ### Run seperately node

```bash
    ros2 run funnyturtle {Node_name}
```

- ### Launch teleop turtle only
```bash
    ros2 launch funnyturtle teleop.launch.py
```

- ### Launch teleop and copy tutles together
```bash
    ros2 launch funnyturtle funnyturtle.launch.py
```

- ### Launch Extra Melodic turtle (After copy turtle was done!)
```bash
    ros2 launch funnyturtle melodic.launch.py
```

## Testing and Result

#