# MyUR Python Control

## Overview

This repository contains Python scripts for controlling the Universal Robots UR3e robotic arm via ROS2. The main component is the **MyUR3e** class, which provides functionalities to interface with the robot arm for tasks such as trajectory planning, gripper control, and accessing live joint states and force sensor data. Check out [our Notion page](https://fetlab.notion.site/UR3e-Robot-Arm-16afcd0dec6648f0b090a2a0807abf8c?pvs=4) to learn more about this project.

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

Install the myur python package and all dependencies:

```bash
cd Universal-Robots-ROS2-CEEO
pip install -e .
```

Add the install directory to your shell's PATH list:
```bash
export PATH="$HOME/.local/bin:$PATH"
source ~/.bashrc
```

## Setup

Before setting up the robot, check your work area. Make sure that nothing is obstructing the path of the UR arm and that all persons are clear of its reach. It is good practice to alert anybody nearby that you are enabling the robot. To begin the setup process, activate the UR arm using the UR Teach Pendant. Go to [this page](https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO/blob/main/URTeachPendant.md) for help using the UR Teach Pendant. Once the arm is activated, follow these steps to establish external control.

If no one else has done so already, open a new terminal and launch the ROS2 UR Driver: (leave this running)
```bash
launch_ur --ip "130.64.17.5"
```

If no one else has done so already, open a second new terminal and launch the ROS Gripper Node: (leave this running)
```bash
launch_gripper --ip "130.64.17.5"
```

Verify you can interact with the ROS2 topics:
```bash
# Should return a list of ~20 topics, for example /joint_states and /gripper/control
ros2 topic list
```

If you would like to interface a microcontroller or locally run PC program with the Jupyter Hub / ROS, you can do so by creating a MQTT forwarder and using the provided Sub_Node.
```bash
create_mqtt_forwarder -h "130.63.16.222" -p "1884" -t "topic"
```

## Code

Open a Python file or Notebook and start playing with the MyUR3e class!
```python
# Import MyUR3e class
from myur import MyUR3e

# Create MyUR3e instance
robot = MyUR3e()

# Get live data from the robot
print(robot.read_joints_pos()) # Joint Positions
print(robot.read_global_pos()) # Global End Effector Position
print(robot.read_force()) # End Effector Force
print(robot.read_gripper()) # Gripper Position, Speed, Force

# Close the gripper
robot.move_gripper(100) # 100 = Close, 0 = Open

# Create a point
point = [0.5, 0.5, 0.3, 0.0, 0.0, 0.0] # [x,y,z,rx,ry,rz]

# Move the arm to the point:
robot.move_global(point)

# Create a trajectory
trajectory = [[0.5, 0.5, 0.3, 0.0, 0.0, 0.0],
              [0.4, 0.4, 0.3, 0.0, 0.0, 0.0]]

# Move the arm along the trajectory:
robot.move_global(trajectory)
```
Go to the [In_Depth_Tutorial](https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO/blob/6d0b88f86543e63ce5a9f9999cb61271c0f339b7/examples/In_Depth_Tutorial.ipynb) in the examples folder for a full run down of features.

## Troubleshooting

I am getting the error **"Goal Rejected :("**:
  - Make sure the UR Pendant has the "External Control" program running
  - Check the logs of the UR ROS Driver for error messages.

The UR ROS Driver logs say **"Can't accept new action goals. Controller is not running."**.
  - Run the following in a new terminal:
```bash
ros2 control switch_controllers --activate scaled_joint_trajectory_controller
```

My code freezes in a move command:
  - Check the logs of the UR ROS Driver for error messages.

My goal is executing but nothing happens:
  - Check the logs of the UR ROS Driver for error messages.

My driver log says **"state tolerance error"**:
  - Use read_joints_pos() or the Teach Pendant to ensure that no joints are wound over 360 degrees.
  - If they are, freedrive the joint back to a neutral position (closest to zero).

My driver log says **"path tolerance error"**:
  - Try freedriving the robot to a different pose, rebooting your UR driver, or rebooting your hub.

My driver log says **"goal tolerance error"**:
  - Try freedriving the robot to a different pose, rebooting your UR driver, or rebooting your hub.
