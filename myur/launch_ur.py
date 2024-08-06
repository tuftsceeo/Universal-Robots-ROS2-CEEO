# myur/launch_ur_control.py

import subprocess

def main():
    command = [
        "ros2", "launch", "ur_robot_driver", "ur_control.launch.py",
        "ur_type:=ur3e", "robot_ip:=130.64.17.5", "launch_rviz:=false"
    ]
    subprocess.run(command)

if __name__ == "__main__":
    main()
