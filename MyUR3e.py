# MyUR3e.py
# Written by Aengus Kennedy and Liam Campbell
# Center for Engineering Education and Outreach
# Summer of 2024

# External Libraries:
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

# Internal Libraries:
from ik_solver.ur_kinematics import URKinematics
import TrajectoryPlanner

# ROS2:
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

# ROS2 Msg Types:
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int32MultiArray
from action_msgs.srv import CancelGoal # need to implement


class MyUR3e(rclpy.node.Node):
    """
    A class to control the UR3e robot arm using ROS2.

    Public Attributes:
        ik_solver (URKinematics): Inverse kinematics solver for the UR3e robot.
        joint_states (JointStates): Instance for subscribing to joint states.
        tool_wrench (ToolWrench): Instance for subscribing to tool wrench data.
        gripper (Gripper): Instance for subscribing to and controlling the gripper.
    """

    def __init__(self,response_callback=None, result_callback=None, timer_callback=None):
        """
        Initialize the MyUR3e node.
        """
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

        # set timer
        self.timer = self.create_timer(1.0, self.get_timer_callback)

        self.get_logger().debug(f"Waiting for action server on {controller_name}")

        # Private Attributes
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self._action_client.wait_for_server()
        self._send_goal_future = None
        self._get_result_future = None
        self._id = 0
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)

        # Public Attributes
        self.sim = TrajectoryPlanner.TrajectoryPlanner()
        self.ik_solver = URKinematics("ur3e")
        self.joint_states = JointStates()
        self.tool_wrench = ToolWrench()
        self.gripper = Gripper()
        self.done = True

        # Set Functions
        self.response_callback = response_callback
        self.result_callback = result_callback
        self.timer_callback = timer_callback

    ########################################################
    #################### PUBLIC METHODS ####################
    ########################################################

    #################### CLASS ACCESS METHODS ####################

    def set_response_callback(self, user_function):
        self.response_callback = user_function

    def set_result_callback(self, user_function):
        self.result_callback = user_function

    def set_timer_callback(self, user_function, period = None):
        if period:
            self.timer.cancel()
            self.timer = self.create_timer(period, self.get_timer_callback)
        self.timer_callback = user_function

    def clear_sim(self):
        """
        Clear all trajectories in the simulation plot.
        """
        self.sim.clear_plot()

    #################### SERVICE METHODS ####################

    def get_gripper(self):
        """
        Get the current state of the gripper.

        Returns:
            list: [POS (int), SPE (int), FOR (int)]
        """
        return list(self.gripper.get())

    def get_joints(self):
        """
        Get the joint_states data.

        Returns:
            dict: {"name","position","velocity","effort"}
        """
        return self.joint_states.get_joints()

    def get_global(self):
        """
        Get the global position of end effector.

        Returns:
            list: [x,y,z,rx,ry,rz]
        """
        return self.joint_states.get_global()

    def get_force(self):
        """
        Get the force data at the end effector.

        Returns:
            dict: {"force","torque"}
        """
        return self.tool_wrench.get()

    #################### MOVEMENT METHODS ####################

    def solve_ik(self, cords, q_guess=None):
        """
        Solve inverse kinematics for given coordinates.

        Args:
            cords (list): A list of coordinates, either [x, y, z, rx, ry, rz] or [x, y, z, qx, qy, qz, qw].
            q_guess (list): A list of joint angles used to find the closest IK solution.

        Returns:
            list: Joint positions that achieve the given coordinates. [see self.joints]
        """
<<<<<<< Updated upstream
        # Get current pose of robot to use as q_guess if q_guess == None
        if q_guess == None:
=======
        # Get current pose of robot to use as q_guess if q_guess is None
        if q_guess is None:
>>>>>>> Stashed changes
            q_guess = self.joint_states.get_joints()["position"]

        # if coordinates in euler format convert to quaternions
        if len(cords) == 6:
            if cords[3:6] == [0, 0, 0]:
                cords[3:6] = [0.1, 0, 0]
            r = R.from_euler("zyx", cords[3:6], degrees=True)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat

        return self.ik_solver.inverse(cords, False, q_guess=q_guess)

    def move_gripper(self, POS, SPE, FOR):
        """
        Move the gripper to the specified position with given speed and force.

        Args:
            POS (int): Position for the gripper.
            SPE (int): Speed for the gripper.
            FOR (int): Force for the gripper.
        """
        self.gripper.control(POS, SPE, FOR)

    def move_global(self, coordinates, time_step=5, sim=True, wait=True):
        """
        Move the robot to specified global coordinates.

        Args:
            coordinates (list): List of coordinates to move to.
                either [x, y, z, rx, ry, rz] or [x, y, z, qx, qy, qz, qw].
            time_step (int): Time step between each coordinate.
            sim (bool): True if no motion is desired, False if motion is desired.
        """
        joint_positions = []
        for i, cord in enumerate(coordinates):
            if i == 0:
                joint_positions.append(self.solve_ik(cord).tolist())
            else:
                joint_positions.append(self.solve_ik(cord, joint_positions[i - 1]).tolist())

        self.sim.add_trajectory(coordinates, joint_positions)

        if None in joint_positions:
            raise RuntimeError(
                f"IK solution not found for {joint_positions.count(None)}/{len(joint_positions)} points"
            )
        elif sim == False:
<<<<<<< Updated upstream
=======
            joint_positions
>>>>>>> Stashed changes
            self.move_joints(joint_positions, time_step=time_step, sim=sim,wait=wait)

    def move_joints_r(self, joint_deltas, time_step=5, units="radians", sim=True, wait=True):
        """
        Move the robot joints relative to their current or last position.

        Args:
            joint_positions (list): List of relative joint positions.
            time_step (int): Time step between each position.
            units (string): Units of angle ("radians","degrees").
            sim (bool): True if no motion is desired, False if motion is desired.
        """
        sequence = []
        for i, delta in enumerate(joint_deltas):
            if i == 0:
                curr = self.get_joints()["position"]
                sequence.append([sum(x) for x in zip(curr, delta)])
            else:
                sequence.append([sum(x) for x in zip(sequence[i - 1], delta)])
        self.move_joints(sequence, time_step=time_step, units=units, sim=sim,wait=wait)

    def move_joints(self, joint_positions, time_step=5, units="radians", sim=True, wait=True):
        """
        Move the robot joints to the specified angular positions.

        Args:
            joint_positions (list): List of joint positions.
            time_step (int): Time step between each position.
            units (string): Units of angle ("radians","degrees").
            sim (bool): True if no motion is desired, False if motion is desired.
        """
        if not sim:
            self.get_logger().debug(f"Beginning Trajectory")
            if units == "radians":
                trajectory = self.make_trajectory(joint_positions, time_step=time_step)
            elif units == "degrees":
                trajectory = self.make_trajectory(
                    joint_positions, units="degrees", time_step=time_step
                )
            self.execute_trajectory(trajectory)
            if wait:
                self.wait(self)
            else:                
                spinthread = threading.Thread(target = self.spinfunction)
                spinthread.start()

    def stop(self):
<<<<<<< Updated upstream
        curr_joints = 
=======
        curr_joints = None 
>>>>>>> Stashed changes
        stop_trajectory = self.make_trajectory(None, stop=True)
        self.execute_trajectory(stop_trajectory)
        self.wait(self)

    def spinfunction(self):

        self._executor.spin_once()
        while self._send_goal_future is None:
            self._executor.spin_once()
        while not self._send_goal_future.done():
            self._executor.spin_once()
        if self._send_goal_future.result().accepted:
            while self._get_result_future is None:
                self._executor.spin_once()
            while not self._get_result_future.done():
                self._executor.spin_once()
            self._executor.spin_once()  # must spin once more for final result callback
            error_code = self.error_code_to_str(self._get_result_future.result().result.error_code)
        else:
            self.get_logger().info(f"Goal #{self._id} rejected :( (Check driver logs for more info)")
        self.get_logger().debug(f"Exiting Async Thread: {error_code}")

        self._send_goal_future = None
        self._get_result_future = None
        self.done = True

    ########################################################
    #################### PRIVATE METHODS ###################
    ########################################################

    #################### ROS CLIENT METHODS #################

    def make_trajectory(self, joint_positions, units="radians", time_step=5,stop=False):
        """
        Create a trajectory for the robot to follow.

        Args:
            joint_positions (list): List of joint positions.
            units (str): Units for the joint positions, either 'radians' or 'degrees'.
            time_step (int): Time step between each position.

        Returns:
            JointTrajectory: The created trajectory.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        self._id += 1

        if not stop:
            for i, position in enumerate(joint_positions):
                point = JointTrajectoryPoint()

                if units == "radians":
                    point.positions = position
                elif units == "degrees":
                    point.positions = self.pointdeg2rad(position)

                if i == 0 and type(time_step) == tuple:
                    time = time_step[0]
                elif type(time_step) == tuple:
                    time = time_step[0] + (i + 1) * time_step[1]
                else:
                    time = (i + 1) * time_step

                sec = int(time - (time % 1))
                nanosec = int(time % 1 * 1000000000)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                trajectory.points.append(point)
        # else:
        #     #point = JointTrajectoryPoint()
        #     trajectory.points.append(point)
        return trajectory

    def execute_trajectory(self, trajectory):
        """
        Execute the given trajectory.

        Args:
            trajectory (JointTrajectory): The trajectory to execute.
        """
        self.get_logger().info(f"Goal #{self._id}: Executing")
        self.done = False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self.get_logger().debug(f"Sent trajectory #{self._id}")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def wait(self, client):
        """
        Wait for the action to complete.

        Args:
            client (ActionClient): The action client.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)

    #################### CALLBACKS ####################

    def get_timer_callback(self, *args, **kwargs):
        if self.timer_callback:
            self.timer_callback(*args, **kwargs)
        else:
            pass

    def goal_response_callback(self, future, *args, **kwargs):
        """
        Callback for when a goal response is received.

        Args:
            future (Future): The future object containing the goal response.
        """
        goal_handle = future.result()

        # user defined callback
        if self.response_callback:
            self.response_callback(goal_handle.accepted, *args, **kwargs)

        if not goal_handle.accepted:
            self.get_logger().info(
                f"Goal #{self._id} rejected :( (Check driver logs for more info)"
            )
            return
        self.get_logger().debug(f"Goal #{self._id} accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future, *args, **kwargs):
        """
        Callback for when a result is received.

        Args:
            future (Future): The future object containing the result.
        """
        result = future.result().result

        # user defined callback
        if self.result_callback:
            self.result_callback(result.error_code,*args, **kwargs)

        self.get_logger().debug(
            f"Done with result from #{self._id}: {self.error_code_to_str(result.error_code)}"
        )
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.done = True
            self.get_logger().info(f"Goal #{self._id}: Completed")

    ########################################################
    #################### STATIC METHODS ####################
    ########################################################

    @staticmethod
    def pointdeg2rad(point):
        """
        Convert a point from degrees to radians.

        Args:
            point (list): List of points in degrees.

        Returns:
            list: List of points in radians.
        """
        return [math.radians(p) for p in point]

    @staticmethod
    def error_code_to_str(error_code):
        """
        Convert an error code to a string.

        Args:
            error_code (int): The error code.

        Returns:
            str: The error code as a string.
        """
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
    Subscribe to the joint_states topic.
    """

    def __init__(self):
        """
        Initialize the JointStates node.
        """
        super().__init__("SubscriberNode")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )
        self.ik_solver = URKinematics("ur3e")
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when joint states are received.

        Args:
            msg (JointState): The joint state message.
        """
        data = {
            "name": [msg.name[5]] + msg.name[0:5],
            "position": [msg.position[5]] + msg.position[0:5].tolist(),
            "velocity": [msg.velocity[5]] + msg.velocity[0:5].tolist(),
            "effort": [msg.effort[5]] + msg.effort[0:5].tolist(),
        }

        self.states = data
        self.done = True

    def get_joints(self):
        """
        Get the current joint states.

        Returns:
            dict: The current joint states.
        """
        self.wait(self)
        self.done = False
        return self.states

    def get_global(self):
        """
        Get the global position of the end effector.

        Returns:
            list: The global position and orientation in Euler angles.
        """
        self.wait(self)
        self.done = False
        cords_q = self.ik_solver.forward(self.states["position"])
        r = R.from_quat(cords_q[3:7], scalar_first=True)
        euler = r.as_euler("xyz", degrees=True).tolist()
        cords = cords_q[0:3].tolist() + euler
        return cords

    def wait(self, client):
        """
        Wait for the joint states to be updated.

        Args:
            client (Node): The node to wait for.
        """
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
        """
        Initialize the ToolWrench node.
        """
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
        """
        Callback for when wrench data is received.

        Args:
            msg (WrenchStamped): The wrench stamped message.
        """
        data = {
            "header": [msg.header.stamp.sec, msg.header.stamp.nanosec],
            "force": [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            "torque": [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
        }

        self.states = data
        self.done = True

    def get(self):
        """
        Get the current wrench data.

        Returns:
            dict: The current wrench data.
        """
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        """
        Wait for the wrench data to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for wrench client")


class Gripper(rclpy.node.Node):
    """
    Subscribe and publish to Gripper topics.
    """

    def __init__(self):
        """
        Initialize the Gripper node.
        """
        super().__init__("gripper_client")
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/gripper/state",
            self.listener_callback,
            10,
        )

        self.publisher_ = self.create_publisher(Int32MultiArray, "/gripper/control", 10)

        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when gripper state data is received.

        Args:
            msg (Int32MultiArray): The gripper state message.
        """
        self.states = msg.data
        self.done = True

    def get(self):
        """
        Get the current state of the gripper.

        Returns:
            list: The current state of the gripper.
        """
        self.wait(self)
        self.done = False
        return self.states

    def control(self, POS, SPE, FOR):
        """
        Control the gripper with the given position, speed, and force.

        Args:
            POS (int): Position for the gripper.
            SPE (int): Speed for the gripper.
            FOR (int): Force for the gripper.
        """
        msg = Int32MultiArray()
        msg.data = [POS, SPE, FOR]
        self.publisher_.publish(msg)

    def wait(self, client): #class gripper
        """
        Wait for the gripper state to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for gripper client")