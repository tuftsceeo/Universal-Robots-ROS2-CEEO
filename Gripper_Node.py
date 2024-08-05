import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from myur import robotiq_gripper
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
        self.gripper.activate(
            auto_calibrate=True
        )  # should maybe add control option to calibrate

        self.target = -1

    def timer_callback(self):
        # every interval, create and publish gripper state
        msg = Int32MultiArray()
        msg.data = [
            int(self.gripper.get_current_position()),
            int(self.gripper.get_current_speed()),
            int(self.gripper.get_current_force()),
            int(self.target),
            int(self.gripper._get_var(self.gripper.PRE)),
            int(self.gripper._get_var(self.gripper.OBJ))
        ]
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            print("RECIEVED CONTROL:",msg.data)
            [POS, SPE, FOR] = msg.data
            null, self.target = self.gripper.move(POS, SPE, FOR)
        except Exception as e:
            self.get_logger().info(f"Incorrect Gripper Control Format: [POS,SPE,FOR]")


def main():
    rclpy.init()
    chatter = Publisher()
    rclpy.spin(chatter)  # The code stays here forever, publishing strings
    chatter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
