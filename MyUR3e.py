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

    @staticmethod
    def pointdeg2rad(point):
        point2 = []
        # point should be a list of 6 floats
        for i in range(len(point)):
            point2.append(math.radians(point[i]))
        return point2

    # might need to add support for list of velocities if it seems useful
    def move_global(
        self,
        cords,
        units="radians",
        velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time=5,
    ):
        # get current pose form /joint_states topic
        current_pose = self.joint_states.get()["position"]
        self.get_logger().debug(f"Current Pose: {current_pose}")

        # check cords for single or multiple points
        if type(cords[0]) == int:
            joint_positions = self.solve_ik(cords, current_pose)
        elif type(cords[0]) == list:
            joint_positions = []
            for i,cord in enumerate(cords): # compute ik for each cord
                try: # ideally just don't add any point for invalid IK
                    if i == 0: # for first cord, use current pose as q_guess
                        joint_positions.append(self.solve_ik(cord, current_pose))
                    else: # for following cords use last pose as q_guess
                        joint_positions.append(
                            self.solve_ik(cord, joint_positions[-1])
                        )
                except Exception as e:
                    self.get_logger().debug(e)
        else:
            raise ValueError("Invalid coordinate format")

        # send off target joint angles
        self.move_joints(
            joint_positions, units=units, velocities=velocities, time=time
        )

    # Potential BUG with invalid IK cords - needs testing
    # TODO add support for more rotation formats
    # ??? what is the most intuitive way of orienting the end effector?
    def solve_ik(self, cords, q_guess):
        if len(cords) == 6:  # convert 3 rotations around cartesian axes to quaternion
            r = R.from_euler("xyz", cords[3:6], degrees=True)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat

        # use class ik solver to find joint positions from x,y,z,quaternion
        joint_positions = self.ik_solver.inverse(
            cords, False, q_guess=q_guess
        )

        if joint_positions is None:
            raise RuntimeError("IK solution not found")

        return joint_positions.tolist()

    def move_joints(
        self,
        joint_positions,
        units="radians",
        velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time=5,
    ):
        self.get_logger().debug(f"Moving to joint angles {joint_positions}")

        # make trajectory and append points
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints

        # make trajectories for single and multiple point instances
        if type(joint_positions[0]) == int:
            trajectory = self.make_point(
                joint_positions,
                trajectory,
                units=units,
                velocities=velocities,
                time=time,
            )
        elif type(joint_positions[0]) == list:
            for pos in joint_positions:
                trajectory = self.make_point(
                    pos, trajectory, units=units, velocities=velocities, time=time
                )

        self.execute_trajectory(trajectory)
        self.wait(self)

    # append point to inputted trajectory object
    def make_point(
        self,
        joint_positions,
        trajectory,
        units="radians",
        velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time=5,
    ):
        point = JointTrajectoryPoint()

        if units == "radians":
            point.positions = joint_positions
        elif units == "degrees":
            point.positions = self.pointdeg2rad(joint_positions)

        point.velocities = velocities
        sec = int(time - (time % 1))
        nanosec = int(time % 1 * 1000000000)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        trajectory.points.append(point)

        return trajectory

    def execute_trajectory(self, trajectory):
        self.id += 1

        self.get_logger().info(f"Trajectory #{self.id}: Executing")
        self.done = False 

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._action_client.wait_for_server()  # this was missing
        self._send_goal_future = self._action_client.send_goal_async(goal)
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
