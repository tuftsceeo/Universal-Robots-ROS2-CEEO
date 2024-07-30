# MyUR ROS2 Python Control

## Overview

This repository contains Python scripts for controlling the Universal Robots UR3e robotic arm using ROS2. The main component is the **MyUR3e** class, which provides functionalities to interface with the robot arm for tasks such as trajectory planning, gripper control, and accessing live joint states and tool wrench data.

## Prerequisites

Before running the scripts in this repository, ensure you have the following installed:

- ROS2 Humble
- Python 3.x (recommended: Python 3.8+)
- UR ROS2 Driver

If you are running code from the CEEO JupyterHub then you can assume that your environment is already configured.

## Installation

Clone this repository to your local machine:

```bash
git clone https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO
```

Install / update all dependencies:

```bash
cd Universal-Robots-ROS2-CEEO
pip install .
```

## Installation

Run the launch.py file:
```bash
python3 launch.py
```

Verify you can interact with the ROS2 topics:
```bash
# Should return a list of ~20 topics, for example /joint_states and /gripper/control
ros2 topic list
```

Open a Python file or Notebook and start playing with the MyUR3e class!
```python
from myur import MyUR3e

def main():
    try:
        # Create MyUR3e instance
        robot = MyUR3e()

        # Get live data from the robot
        print(robot.read_joints_pos()) # Joint Positions
        print(robot.read_global_pos()) # Global End Effector Position
        print(robot.read_force()) # End Effector Force
        print(robot.read_gripper()) # Gripper Position, Speed, Force

        # Open and close the gripper
        robot.move_gripper(0,255,255) # Open
        robot.move_gripper(255,255,255) # Close

        # Create a point or trajectory
        point = [[0.5, 0.5, 0.3, 0.0, 0.0, 0.0]] # [x,y,z,rx,ry,rz]
        trajectory = [[0.5, 0.5, 0.3, 0.0, 0.0, 0.0],
                      [0.4, 0.4, 0.3, 0.0, 0.0, 0.0]]

        # Move the Arm:
        robot.move_global(trajectory)

    finally:
        del robot

if __name__ == '__main__':
    main()
```

## Troubleshooting ##

My code freezes when I try to initialize the MyUR3e class:
  - Make sure that you have launched the UR ROS Driver.<br/>
I am getting the error "Goal Rejected :(":
  - Check the logs of the UR ROS Driver for error messages
  - Make sure the UR Pendant has the "External Control" program running
  - Make sure the scaled_joint_trajectory_controller is running. If its not, run the following in an available terminal:
```bash
ros2 control list_controllers
ros2 control switch_controllers --activate scaled_joint_trajectory_controller
```
