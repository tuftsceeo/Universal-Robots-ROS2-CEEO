import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from ik_solver.ur_kinematics import URKinematics
#from example_interfaces.action import Fibonacci
#from my_custom_action_package.action import DrawingAction
from custom_action.action import DrawingAction # source ros2_ws/install/setup.bash
from ast import literal_eval
import math
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Duration, Time
#@todo move all imports in the code body to the top

# PAGE COORDINATES
LEFT_TOP = [-0.0904, 0.4208, 0.22, 0, 0, 1]
LEFT_BOTTOM = [-0.0904, 0.2049, 0.217, 0, 0, 1]
RIGHT_BOTTOM = [0.1890, 0.2049, 0.217, 0, 0, 1]
RIGHT_TOP = [0.1890, 0.4208, 0.22, 0, 0, 1]

# TIME STEPS
TRAVEL_TIME = 0.4
TRAJ_TIME = 0.05

# # PROPORTIONAL CONTROL
# TARGET = -9 # Newtons ?
# PROPORTION = 0.0005 
# FREQUENCY = 0.1 # s / 

# X_LINE = [[0.15,0.15,0.25,0,0,0],
#           [0.25,0.15,0.25,0,0,0],
#           [0.15,0.15,0.25,0,0,0],
#           [0.25,0.15,0.25,0,0,0]]
# Y_LINE = [[0.2,0.1,0.25,0,0,0],
#           [0.2,0.2,0.25,0,0,0],
#           [0.2,0.1,0.25,0,0,0],
#           [0.2,0.2,0.25,0,0,0]]

class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')

        # Action Server Init (receive from brain)
        self._action_server = ActionServer(
            self,
            DrawingAction,
            'drawing_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Action Client Init (send to robot)
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
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)

        # Tool Wrench Subscriber
        self.force_subscription = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.force_callback,
            10,
        )

        # Joint State Subscriber
        self.state_subscription = self.create_subscription(
            JointState, "/joint_states", self.state_callback, 10
        )

        # Timer callback
        ### self.timer = self.create_timer(FREQUENCY, self.timer_callback) # Enable this for proportional press force control

        self.ik_solver = URKinematics("ur3e")
        self._done = True
        self.force = None
        self.state = None
        self._id = 0
        self._send_goal_future = None
        self._get_result_future = None
        #self.current_path = None

        # # TRAJECTORY REPLACEMENT TEST
        # self.testtimer = self.create_timer(2, self.test_timer_callback)
        # self.x_axis = False
        # self.start_time = self.get_time()
    
    
    # def test_timer_callback(self):
    #     from random import random
    #     global X_LINE
    #     global Y_LINE
            
    #     time_stamp = self.get_time()

    #     self.get_logger().info(f'REPLACING TRAJECTORY')

    #     if random() < 0.25:
    #         [current_x,current_y] = self.get_global()[0:2]
    #         if self.x_axis:
    #             for i in range(len(X_LINE)):
    #                 X_LINE[i][1] = current_y
    #             self.get_logger().info(f'X AXIS')
    #             self.move_global(X_LINE,time_step=(TRAVEL_TIME,TRAJ_TIME),sim=False,time_start=time_stamp)
    #         else:
    #             for i in range(len(Y_LINE)):
    #                 Y_LINE[i][0] = current_x
    #             self.get_logger().info(f'Y AXIS')
    #             self.move_global(Y_LINE,time_step=(TRAVEL_TIME,TRAJ_TIME),sim=False,time_start=time_stamp)
        
    #         self.x_axis = not self.x_axis

    def get_global(self):
        """
        Get the global position of the end effector.

        Returns:
            list: The global position and orientation in Euler angles.
        """
        cords_q = self.ik_solver.forward(self.state["position"])
        r = R.from_quat(cords_q[3:7], scalar_first=True)
        euler = r.as_euler("xyz", degrees=True).tolist()
        cords = cords_q[0:3].tolist() + euler
        return cords

    def get_time(self):
        time = self.get_clock().now().to_msg()
        return time.sec + time.nanosec * 1e-9

    # def timer_callback(self):
    #     if self.current_path is not None:
    #         time_stamp,line = self.current_path.values()

    #         # Avoid evaluating force while arm is moving to next traj
    #         if self.get_time() - time_stamp > TRAVEL_TIME:
    #             forceZ = self.force['force'][2]

    #             # calculate delta height
    #             proportion = PROPORTION
    #             targetForce = TARGET
    #             delta = proportion*(targetForce - forceZ)
    #             self.get_logger().info(f'Current Force: {forceZ}, Delta: {delta}')

                
    #             # write new trajectory, avoid overwriting z of first and last points
    #             for i in range(len(line)):
    #                 if (i != 0) and (i != len(line)-1):  
    #                     line[i][2] = line[i][2] + delta
                
    #             # send new trajectory
    #             # NEED TO GIVE TIME STAMP
    #             self.get_logger().info(f'Adjusting Trajectory')
    #             self.move_global(line,time_step=(TRAVEL_TIME,TRAJ_TIME),sim=False,time_start=time_stamp)

    def force_callback(self, msg):
        """
        Callback for when joint states are received.

        Args:
            msg (JointState): The joint state message.
        """
        data = {
            "header": [msg.header.stamp.sec, msg.header.stamp.nanosec],
            "force": [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            "torque": [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
        }

        self.force = data

    def state_callback(self, msg):
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

        self.state = data

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        long_string = goal_handle.request.coordinates  # Using "order" to carry the string
        self.get_logger().info(f'Action server received string: {long_string[:100]}')

        max_vals, data = literal_eval(long_string)
        max_vals = list(max_vals)
        data = list(data)
        #exec('data = ' + long_string) #formerly
        
        # Init Feedback:
        feedback_msg = DrawingAction.Feedback()
        feedback_msg.status = 0.0

        # Parse Line + Send Feedback
        coordinates = []
        for i,place in enumerate(data):
            feedback_msg.status = i/len(data)
            goal_handle.publish_feedback(feedback_msg)

            if i == 0: # start up
                coordinates.append(self.paper_coordinates(place[0],place[1],up=True,max_vals=max_vals))

            coordinates.append(self.paper_coordinates(place[0],place[1],up=False,max_vals=max_vals))

            if i == len(data)-1: # end up
                coordinates.append(self.paper_coordinates(place[0],place[1],up=True,max_vals=max_vals))

        self.current_path = {"stamp":self.get_time(),
                             "path":coordinates}

        self.get_logger().info(f'Starting move')
        self.move_global(coordinates,time_step=(TRAVEL_TIME,TRAJ_TIME),sim=False)
        
        await self._send_goal_future # goal accepted
        await self._get_result_future # goal completed # await removed
        
        result = self._get_result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f'SUCCESS')
            # Update Result:
            goal_handle.succeed()
            result = DrawingAction.Result()
            result.result = True
        elif result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            self.get_logger().info(f'INVALID_GOAL')
            result.result = False
        elif result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            self.get_logger().info(f'INVALID_JOINT')
            result.result = False
        elif result.error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            self.get_logger().info(f'OLD_HEADER_TIMESTAMP')
            result.result = False
        elif result.error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            self.get_logger().info(f'PATH_TOLERANCE')
            result.result = False
        elif result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            self.get_logger().info(f'GOAL_TOLERANCE')
            result.result = False
        else:
            self.get_logger().info(f'OTHER_ERROR: {result.error_code}')
            result.result = False
            
        return result


    ########################################################
    #################### MyUR3e METHODS ####################
    ########################################################
    
    ########################################################
    #################### PUBLIC METHODS ####################
    ########################################################


    # Function to convert page coordinates to global
    def paper_coordinates(self,x,y, max_vals=None, up=True,):
        # Purple marker
        # Upper Right: 0.218
        # Upper Left: 0.218
        # Lower Left: 0.2157
        # Lower Right: 0.2158

        # black standard sharpie
        # 0.224 upper right
        # 0.222 lower right
        # 0.224 upper left
        # 0.222 lower left
        
        z_paper_top = 0.224
        z_paper_bottom = 0.222
        
        # the dimensions of the subset of the paper that we will draw on
        draw_height = 0.18 # meters
        draw_width = 0.25 # meters

        paper_height = 0.2159 # meters
        paper_width = 0.2794 # meters
        svg_max_x = max_vals[0]
        svg_max_y = max_vals[1]

        scale_x = draw_width / svg_max_x
        scale_y = draw_height / svg_max_y
        scale = min(scale_x, scale_y)

        y = 0.2159 - (y * scale + ((0.2159 - draw_height) / 2)) 
        x = x * scale + ((0.2794 - max_vals[0]*scale) / 2)

        cords = []
        if (x > 0.2794) or (y > 0.2159) or (x < 0) or (y < 0):
            raise RuntimeError("Your coordinates are off the paper")
        cords.append(LEFT_BOTTOM[0]+x)
        cords.append(LEFT_BOTTOM[1]+y)
        if up:
            cords.append(z_paper_top + 0.004)
        else:
            cords.append(z_paper_top-((z_paper_top - z_paper_bottom)*((0.2159-y)/0.2159)))
        
        return cords + [0, 0, 0]
    
    def solve_ik(self, cords, q_guess=None):
        """
        Solve inverse kinematics for given coordinates.

        Args:
            cords (list): A list of coordinates, either [x, y, z, rx, ry, rz] or [x, y, z, qx, qy, qz, qw].
            q_guess (list): A list of joint angles used to find the closest IK solution.

        Returns:
            list: Joint positions that achieve the given coordinates. [see self.joints]
        """
        # Get current pose of robot to use as q_guess if q_guess == None
        if q_guess is None:
            q_guess = self.state["position"]

        # if coordinates in euler format convert to quaternions
        if len(cords) == 6:
            if cords[3:6] == [0, 0, 0]:
                cords[3:6] = [0.1, 0, 0]
            r = R.from_euler("zyx", cords[3:6], degrees=True)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat

        result = self.ik_solver.inverse(cords, False, q_guess=q_guess)
        if result is None:
            raise RuntimeError('an IK solution was not found for the point {}'.format(cords))


        result = [float(i) for i in result]

        return result

    def move_global(self, coordinates, time_step=5, time_start=None, sim=True):
        """
        Move the robot to specified global coordinates.

        Args:
            coordinates (list): List of coordinates to move to.
                either [x, y, z, rx, ry, rz] or [x, y, z, qx, qy, qz, qw].
            time_step (int): Time step between each coordinate.
            time_start (float): Time to start moving. (can be in the past?)
            sim (bool): True if no motion is desired, False if motion is desired.
        """
        self.get_logger().info(f'Z HEIGHT: {coordinates[1][2]}')
        joint_positions = []
        for i, cord in enumerate(coordinates):
            if i == 0:
                joint_positions.append(self.solve_ik(cord))
            else:
                joint_positions.append(self.solve_ik(cord, joint_positions[i - 1]))

        '''if None in joint_positions:
            raise RuntimeError(
                f"IK solution not found for {joint_positions.count(None)}/{len(joint_positions)} points"
            )
        elif not sim:'''
        self.move_joints(joint_positions, time_step=time_step, time_start=time_start, sim=sim)
            
    def move_joints(self, joint_positions, time_step=5, time_start = None, units="radians", sim=True):
        """
        Move the robot joints to the specified angular positions.

        Args:
            joint_positions (list): List of joint positions.
            time_step (int): Time step between each position.
            time_start (float): Time to start moving. (can be in the past?)
            units (string): Units of angle ("radians","degrees").
            sim (bool): True if no motion is desired, False if motion is desired.
        """
        if not sim:
            self.get_logger().debug(f"Beginning Trajectory")
            if units == "radians":
                trajectory = self.make_trajectory(joint_positions, time_step=time_step, time_start=time_start)
            elif units == "degrees":
                trajectory = self.make_trajectory(
                    joint_positions, units="degrees", time_step=time_step, time_start=time_start
                )
            self.execute_trajectory(trajectory)
            #self.wait(self)

    ########################################################
    #################### PRIVATE METHODS ###################
    ########################################################


    def make_trajectory(self, joint_positions, units="radians", time_step=5, time_start = None):
        """
        Create a trajectory for the robot to follow.

        Args:
            joint_positions (list): List of joint positions.
            units (str): Units for the joint positions, either 'radians' or 'degrees'.
            time_step (int): Time step between each position.
            time_start (float): Time to start moving. (can be in the past?)
        Returns:
            JointTrajectory: The created trajectory.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        self._id += 1

        if time_start is None: # If time_start was not specified, use the time right now.
            trajectory.header.stamp = self.get_clock().now().to_msg()
        else: # If time_start was specifided, convert it to a builtin_interfaces.msg.Time object
            sec = int(time_start)
            nanosec = int(time_start % 1 * 1000000000)
            trajectory.header.stamp = Time(sec=sec, nanosec=nanosec)

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

            sec = int(time)
            nanosec = int(time % 1 * 1000000000)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(point)

        return trajectory

    def execute_trajectory(self, trajectory):
        """
        Execute the given trajectory.

        Args:
            trajectory (JointTrajectory): The trajectory to execute.
        """
        self.get_logger().info(f"Goal #{self._id}: Executing")
        self._done = False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self.get_logger().debug(f"Sent trajectory #{self._id}")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for when a goal response is received.

        Args:
            future (Future): The future object containing the goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(
                f"Goal #{self._id} rejected :( (Check driver logs for more info)"
            )
            return
        self.get_logger().debug(f"Goal #{self._id} accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for when a result is received.

        Args:
            future (Future): The future object containing the result.
        """
        result = future.result().result
        self.get_logger().debug(
            f"Done with result from #{self._id}: {self.error_code_to_str(result.error_code)}"
        )
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._done = True
            self.get_logger().info(f"Goal #{self._id}: Completed")
        else:
            self.get_logger().info(f"Goal #{self._id}: unsuccessful")

    def wait(self, client):
        """
        Wait for the action to complete.

        Args:
            client (ActionClient): The action client.
        """
        rclpy.spin_once(client)
        while not self._done:
            rclpy.spin_once(client)

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


def main(args=None):
    rclpy.init(args=args)
    action_server = MinimalActionServer()
    rclpy.spin(action_server)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()