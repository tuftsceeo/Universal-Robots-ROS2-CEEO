import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import robotiq_gripper
import time

IP = "130.64.17.5"


class Publisher(Node):

    def __init__(self):
        # initialize the topic (name it, create the publisher and publishing rate
        super().__init__("GripperNode")
        print("Initizializing Gripper Node")
        queue_size = 10
        self.publisher_ = self.create_publisher(Int32MultiArray, "/gripper/state", 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            Int32MultiArray, "/gripper/control", self.listener_callback, 10
        )

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(IP, 63352)
        print("Activating Gripper")
        self.gripper.activate(auto_calibrate=False) # should maybe add control option to calibrate

    def timer_callback(self):
        # every interval, create and publish gripper state
        msg = Int32MultiArray()
        msg.data = [
            int(self.gripper.get_current_position()),
            int(self.gripper.get_current_speed()),
            int(self.gripper.get_current_force()),
        ]
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            [POS,SPE,FOR] = msg.data
            self.gripper.move(POS,SPE,FOR) # non blocking (move_and_wait_for_pos() is blocking)
        except Exception as e:
            # is there a better way to return status of command?
            self.get_logger().info(f"Incorrect Gripper Control Format: [pos,spe,for]")


def main():
    rclpy.init()
    chatter = Publisher()
    rclpy.spin(chatter)  # The code stays here forever, publishing strings
    chatter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
