import time
import math
import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class MyUR3e(rclpy.node.Node):
    def __init__(self):
        super().__init__("remote_control")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        controller_name = (
            self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        )
        self.joints = self.get_parameter("joints").value
        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        print(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

    @staticmethod
    def pointdeg2rad(point):
        point2 = []
        # point should be a list of 6 floats
        for i in range(len(point)):
            point2.append(math.radians(point[i]))
        print(point2)
        return point2

    def move_global(self,cords):
        pass

    def move_joints(self, joint_positions):
        goal = self.make_goal(joint_positions)
        self.execute_goal(goal)

        pass

    #@staticmethod # not used because goal.joint_names relies on instance's self.joints
    def make_goal(self, joint_positions, units = 'radians', velocities = [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start = 5):
        goal = JointTrajectory()
        goal.joint_names = self.joints
        point = JointTrajectoryPoint()

        if units == "radians":
            point.positions = joint_positions
        elif units == "degrees":
            point.positions = self.pointdeg2rad(joint_positions)

        point.velocities = velocities
        point.time_from_start = Duration(sec = time_from_start, nanosec = 0)
        goal.points.append(point)
        return goal

    def execute_goal(self, goal):
        this_goal = FollowJointTrajectory.Goal()
        this_goal.trajectory = goal
        self._send_goal_future = self._action_client.send_goal_async(this_goal)
        print("Sent goal")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected :(")
            return
        else:
            print("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f"Done with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            print("Goal finished")

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"
