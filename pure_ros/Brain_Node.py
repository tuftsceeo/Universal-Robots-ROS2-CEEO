import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_action.action import DrawingAction
import xml.etree.ElementTree as ET
import json

class DrawingActionClient(Node):
    def __init__(self,svg_file):
        super().__init__("the_brain")
        self.get_logger().info(f"Initialized the_brain")
        self._action_client = ActionClient(self, DrawingAction, "drawing_action")
        self.coordinates, (self.max_x,self.max_y) = self.parse_svg(svg_file)
        self.num_lines = len(self.coordinates)
        self.get_logger().info(f"Parsed SVG")
        self.done = True
        self.counter = 0

    def send_goal(self,line):
        self.done = False
        data = json.dumps([[self.max_x,self.max_y],line])

        goal_msg = DrawingAction.Goal()
        goal_msg.coordinates = data

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info(f"Line #{self.counter}/{self.num_lines}: Sent")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Line #{self.counter}/{self.num_lines} rejected :(")
            return

        self.get_logger().debug(f"Line #{self.counter}/{self.num_lines} accepted :)")
        # Get the result from the action server
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"\rLine #{self.counter}/{self.num_lines}: {100*round(feedback.status,2)}%",end='',flush=True)

    def get_result_callback(self, future):
        print()
        result = future.result().result
        if result.result:
            status = "COMPLETE"
            self.done = True
        else:
            status = "BAD"
        self.get_logger().info(f"Line #{self.counter}/{self.num_lines}: {status}")

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
    from linedraw import linedraw
    linedraw.sketch("IMG_3322.jpg")
    rclpy.init(args=args)
    
    action_client = DrawingActionClient("out.svg")
    
    for line in action_client.coordinates:
        action_client.counter += 1
        action_client.send_goal(line)
        #rclpy.spin(action_client)
        while not action_client.done:
            rclpy.spin_once(action_client)
            time.sleep(0.01)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
