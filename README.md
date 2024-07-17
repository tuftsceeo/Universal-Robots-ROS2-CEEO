# MyUR3e ROS2 Python Control

## Overview

This repository contains Python scripts for controlling the Universal Robots UR3e robotic arm using ROS2. The main component is the **MyUR3e** class, which provides functionalities to interface with the robot arm for tasks such as trajectory planning, gripper control, and accessing live joint states and tool wrench data.

## Prerequisites

Before running the scripts in this repository, ensure you have the following installed:

- ROS2 Humble
- Python 3.x (recommended: Python 3.8+)
- UR ROS2 Driver

* If you are running code from the CEEO JupyterHub then you can assume that your environment is already configured.

## Installation

Clone this repository to your local machine:

```bash
git clone <repository-url>
cd <repository-directory>

## Usage

Run the launch.py file:
```bash
python3 launch.py
```

Verify you can interact with the ROS2 topics by listening to topics:
```bash
ros2 topic list
```

Open a Python file or Notebook and start playing with the MyUR3e class!
```python
import MyUR3e
import rclpy

def main():
    rclpy.init()

    try:
        # Create MyUR3e instance
        robot = MyUR3e.MyUR3e()

        # Get live data from the robot
        print(robot.joint_states.get_joints()) # Joint Positions, Velocities, Efforts
        print(robot.joint_states.get_global()) # Global End Effector Position
        print(robot.tool_wrench.get()) # End Effector Force, Torque
        print(robot.gripper.get()) # Gripper Position, Speed, Force

        # Open and close the gripper
        robot.gripper.move(0,255,255) # Open
        robot.gripper.move(255,255,255) # Close

        # Move the Arm:
        coordinates = [0.5, 0.5, 0.3, 0.0, 0.0, 0.0] # [x,y,z,rx,ry,rz]
        robot.move_global(coordinates)

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
