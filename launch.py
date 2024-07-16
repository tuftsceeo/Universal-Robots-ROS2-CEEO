import subprocess

process1 = subprocess.Popen(
    [
        "ros2",
        "launch",
        "ur_robot_driver",
        "ur_control.launch.py",
        "ur_type:=ur3e",
        "robot_ip:=130.64.17.5",
        "launch_rviz:=false",
    ]
) 
process2 = subprocess.Popen(
    [
        "ros2",
        "control",
        "switch_controllers",
        "--activate",
        "scaled_joint_trajectory_controller",
    ]
)
process3 = subprocess.Popen(["python3", "Gripper_Publisher.py"])

process1.wait()
process2.wait()
process3.wait()
