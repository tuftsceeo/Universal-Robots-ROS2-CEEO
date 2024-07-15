import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.action import ActionClient
from ik_solver.ur_kinematics import URKinematics
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped


class MyException(Exception):
    pass


class MyUR3e(rclpy.node.Node):
    def __init__(self):
        super().__init__("remote_control_client")
        self.get_logger().debug("Initializing MyUR3e...")

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
        self.get_logger().debug(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.ik_solver = URKinematics("ur3e")
        self._send_goal_future = None
        self._get_result_future = None
        self.done = True
        self.id = 0
        self.joint_states = JointStates()
        self.tool_wrench = ToolWrench()

    @staticmethod
    def pointdeg2rad(point):
        point2 = []
        # point should be a list of 6 floats
        for i in range(len(point)):
            point2.append(math.radians(point[i]))
        return point2

    def move_global(self, cords, time=5):
        current_pose = self.joint_states.get()["position"]
        self.get_logger().debug(f"Current Pose: {current_pose}")

        if len(cords) == 6:
            r = R.from_euler("xyz", cords[3:6], degrees=True)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat
        joint_positions = self.ik_solver.inverse(
            cords, False, q_guess=current_pose
        )  # missing q_guess=joint_angles_rad could be essential
        self.get_logger().debug(f"New Pose: {joint_positions}")
        if joint_positions is None:
            raise MyException("IK solution not found")
        else:
            self.move_joints(joint_positions.tolist(), time=time)

    def move_joints(self, joint_positions, time=5):
        self.get_logger().debug(f"Moving to joint angles {joint_positions}")
        goal = self.make_goal(joint_positions, time=time)
        self.execute_goal(goal)
        self.wait(self)

    # @staticmethod # not used because goal.joint_names relies on instance's self.joints
    def make_goal(
        self,
        joint_positions,
        units="radians",
        velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time=5,
    ):
        goal = JointTrajectory()
        goal.joint_names = self.joints
        point = JointTrajectoryPoint()
        self.id += 1

        if units == "radians":
            point.positions = joint_positions
        elif units == "degrees":
            point.positions = self.pointdeg2rad(joint_positions)

        point.velocities = velocities
        sec = int(time - (time % 1))
        nanosec = int(time % 1 * 1000000000)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.points.append(point)
        return goal

    def execute_goal(self, goal):
        self.get_logger().info(f"Goal #{self.id}: Executing")
        self.done = False  # this was .static ??? could be error

        this_goal = FollowJointTrajectory.Goal()
        this_goal.trajectory = goal

        self._action_client.wait_for_server()  # this was missing
        self._send_goal_future = self._action_client.send_goal_async(this_goal)
        self.get_logger().debug(f"Sent goal #{self.id}")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal #{self.id} rejected :(")
            return
        self.get_logger().debug(f"Goal #{self.id} accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().debug(
            f"Done with result from #{self.id}: {self.error_code_to_str(result.error_code)}"
        )
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.done = True
            self.get_logger().info(f"Goal #{self.id}: Completed")

    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)

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


class JointStates(rclpy.node.Node):
    """
    Subscribe to the joint_states topic
    """

    def __init__(self):
        super().__init__("SubscriberNode")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        data = {}
        data["name"] = [msg.name[5]] + msg.name[0:5]
        data["position"] = [msg.position[5]] + msg.position[0:5].tolist()
        data["velocity"] = [msg.velocity[5]] + msg.velocity[0:5].tolist()
        data["effort"] = [msg.effort[5]] + msg.effort[0:5].tolist()

        self.states = data
        self.done = True

    def get(self):
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for joint_states_client")


class ToolWrench(rclpy.node.Node):
    """
    Subscribe to the /force_torque_sensor_broadcaster/wrench topic.
    Data =  {
            header = [sec, nanosec],
            force = [x,y,z],
            torque = [x,y,z]
            }
    """

    def __init__(self):
        super().__init__("wrench_subscriber")
        self.subscription = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.listener_callback,
            10,
        )
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        data = {}
        data["header"] = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        data["force"] = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        data["torque"] = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

        self.states = data
        self.done = True

    def get(self):
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for wrench client")
