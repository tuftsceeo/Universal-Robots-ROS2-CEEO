import numpy as np
from myur import MyUR3e

robot = MyUR3e()
robot.move_gripper(255, 255, 255)
robot.clear_sim()


# LOCATION VARIABLES
HOME = [0.3, 0.3, 0.2, 0, 0, 0]
CAKE_CENTER = [0.1, 0.3, 0.3]
CAKE_RADIUS = 0.045
CAKE_HEIGHT = 0.075
LOWEST_Z = 0.175

# SPATULA CHARACTERISTICS
SPATULA_RADIUS = 0.06
SPATULA_OFFSET = 0.12
EFFECTIVE_RADIUS = CAKE_RADIUS + SPATULA_RADIUS

# FROSTING VARIABLES
FROSTING_HALFWAY_JOINTS = [4.13, -1.88, -1.56, -1.55, 0.67, -2.88]
FROSTING_START_JOINTS = [3.46, -2.02, -2.36, -1.89, -1.21, 3.16]
FROSTING_LENGTH = 0.15
FROSTING_DROP_HEIGHT = 0.006
NUM_SCOOP = 0
FROSTING_SEGMENT = 4
SPATULA_ANGLE = 45


def angle_clean(angle):
    angle = angle % 360
    if angle == 0:
        angle = 0.1
    return angle


def home():
    robot.move_global([HOME], time_step=(3, 0.4), sim=False)
    if robot.read_joints_pos()[5] > 6:
        robot.move_joint_r([[0, 0, 0, 0, 0, -2 * np.pi]])
    elif robot.read_joints_pos()[5] < -6:
        robot.move_joints_r([[0, 0, 0, 0, 0, +2 * np.pi]])


def scoop(length, height):
    global NUM_SCOOP  # count number of scoop for varying depth

    robot.move_joints([FROSTING_HALFWAY_JOINTS, FROSTING_START_JOINTS], time_step=2)

    # Create scoop trajectory
    frosting_scoop = []
    for i in range(100):
        frosting_scoop.append(
            [
                0,
                (length / 100),
                0,
                0,
                0,
                0,
            ]
        )
    robot.move_global_r(
        [[0, 0, -(height * NUM_SCOOP), 0, 0, 0]], time_step=0.1 + NUM_SCOOP * 0.06
    )
    robot.move_global_r(frosting_scoop, time_step=(0.1 + NUM_SCOOP * 0.06, 0.02))
    robot.move_joints([FROSTING_HALFWAY_JOINTS], time_step=2)
    NUM_SCOOP += 1


def arc(divisor, segment=0, steps=100, center=CAKE_CENTER, radius=EFFECTIVE_RADIUS):
    # segment should be in range(divisor)
    # i.e. if divisor = 5, segment should be 0, 1, 2, 3, or 4

    angles = []
    for i in range(steps):
        ith_angle = 360 / divisor * (segment + i / steps)
        angles.append(ith_angle)
    arc_points = []
    for theta in angles:
        x = center[0] + radius * np.cos(np.deg2rad(theta))
        y = center[1] + radius * np.sin(np.deg2rad(theta))
        arc_points.append(
            [x, y, LOWEST_Z, 0, 0.1, angle_clean(theta + 270 + SPATULA_ANGLE)]
        )

    start = list(arc_points[0])
    start[2] = start[2] + 0.1
    end = list(arc_points[-1])
    end[2] = end[2] + 0.1
    robot.move_global([start], time_step=2)
    robot.move_global(arc_points, time_step=(1, 0.05))
    robot.move_global([end], time_step=2)


def main():
    home()
    for i in range(4):
        scoop(FROSTING_LENGTH, FROSTING_DROP_HEIGHT)
        home()
        arc(FROSTING_SEGMENT, segment=i)


main()
