import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import robotiq_gripper
import time

IP = "130.64.17.5"


class Publisher(Node):

    def __init__(self):
        # initialize the topic (name it, create the publisher and publishing rate
        super().__init__("PublisherNode")
        queue_size = 10
        self.publisher_ = self.create_publisher(Float64MultiArray, "gripper_controller", queue_size)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(IP, 63352)
        self.gripper.activate(auto_calibrate=False)

    def timer_callback(self):
        # every interval, create and publish a string
        msg = Float64MultiArray()
        msg.data = [
            float(self.gripper.get_current_position()),
            float(self.gripper.get_current_speed()),
            float(self.gripper.get_current_force()),
        ]
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    chatter = Publisher()
    rclpy.spin(chatter)  # The code stays here forever, publishing strings
    # destructor & shutdown were removed
    chatter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
