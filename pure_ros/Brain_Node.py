import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_action.action import DrawingAction
import xml.etree.ElementTree as ET
import json


class DrawingActionClient(Node):
    def __init__(self, svg_file):
        super().__init__("the_brain")
        self.get_logger().info(f"Initialized the_brain")
        self._action_client = ActionClient(self, DrawingAction, "drawing_action")
        self.coordinates, (self.max_x, self.max_y) = self.parse_svg(svg_file)
        self.get_logger().info(f"Parsed SVG: max[{self.max_x},{self.max_y}]")
        self.done = True

    def send_goal(self, line):
        self.done = False
        data = json.dumps([[self.max_x, self.max_y], line])

        goal_msg = DrawingAction.Goal()
        goal_msg.coordinates = data

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info(f"Sent Goal")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        # Get the result from the action server
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.status}%")

    def get_result_callback(self, future):
        result = future.result().result
        status = "Good" if result.result else "Bad"
        self.get_logger().info(f"Result: {status}")
        # rclpy.shutdown()
        self.done = True

    def parse_svg(self, svg_file):
        tree = ET.parse(svg_file)
        root = tree.getroot()

        # Namespace handling
        namespaces = {"svg": "http://www.w3.org/2000/svg"}

        polylines = root.findall(".//svg:polyline", namespaces)
        all_coordinates = []

        max_x = float("-inf")
        max_y = float("-inf")

        for polyline in polylines:
            points = polyline.get("points")
            if points:
                # Split the points string into individual coordinate pairs
                points_list = points.strip().split(",")
                # Extract x and y coordinates by pairing adjacent items in the list
                coordinates = [
                    (float(points_list[i]), float(points_list[i + 1]))
                    for i in range(0, len(points_list), 2)
                ]
                all_coordinates.append(coordinates)

                # Update max x and y values
                for x, y in coordinates:
                    if x > max_x:
                        max_x = x
                    if y > max_y:
                        max_y = y

        return all_coordinates, (max_x, max_y)


def main(args=None):
    rclpy.init(args=args)
    action_client = DrawingActionClient("crogers.svg")
    for line in action_client.coordinates:
        action_client.send_goal(line)
        # rclpy.spin(action_client)
        while not action_client.done:
            rclpy.spin_once(action_client)
            time.sleep(0.01)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
