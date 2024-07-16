from subprocess import call

# Initilaize the launch files, using python
call(["ros2", "launch", "ur_robot_driver", "ur_control.launch.py", "ur_type:=ur3e", "robot_ip:=130.64.17.5", "launch_rviz:=false"])
call(["ros2", "control", "switch_controllers", "--activate", "scaled_joint_trajectory_controller"])

# Spin up the gripper publisher
call(["python3","Gripper_Publisher.py"])
