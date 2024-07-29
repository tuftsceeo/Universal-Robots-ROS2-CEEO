import numpy as np
from myur import MyUR3e

robot = MyUR3e()
robot.move_gripper(255, 255, 255)
robot.clear_sim()

HOME = [0.3, 0.3, 0.2, 0, 0, 0]

CAKE_CENTER = [0.1, 0.3, 0.3]
CAKE_RADIUS = 0.045
CAKE_HEIGHT = 0.075

SPATULA_RADIUS = 0.06
SPATULA_OFFSET = 0.12

LOWEST_Z = 0.175

EFFECTIVE_RADIUS = CAKE_RADIUS + SPATULA_RADIUS


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


FROSTING_HALFWAY_JOINTS = [4.13, -1.88, -1.56, -1.55, 0.67, -2.88]
FROSTING_START_JOINTS = [4.02, -1.98, -2.34, -2.01, -0.78, -3.08]
FROSTING_LENGTH = 0.15
FROSTING_DROP_HEIGHT = 0.01
frosting_iter = 0

def scoop():
    global frosting_iter
    robot.move_joints(
        [FROSTING_HALFWAY_JOINTS, FROSTING_START_JOINTS], sim=False, time_step=2
    )

    frosting_scoop = []
    for i in range(100):
        frosting_scoop.append(
            [0.303,0.059 - (i * FROSTING_LENGTH / 100),0.091 - FROSTING_DROP_HEIGHT * frosting_iter,-69.28,84.67,-22.26]
        )

    robot.move_global(frosting_scoop, sim=False, time_step=(0.1, 0.02))
    robot.move_joints([FROSTING_HALFWAY_JOINTS], sim=False, time_step=2)
    frosting_iter += 1


steps = 100
circle_points = []
FROSTING_SEGMENT = 4


def circle():
    for t in range(steps):
        x = CAKE_CENTER[0] + EFFECTIVE_RADIUS * np.cos(t * 2 * np.pi / (steps))
        y = CAKE_CENTER[1] + EFFECTIVE_RADIUS * np.sin(t * 2 * np.pi / (steps))
        circle_points.append(
            [x, y, LOWEST_Z, 0, 0.1, angle_clean(t * 360 / steps + 270)]
        )
    segment = int(len(circle_points) / FROSTING_SEGMENT)
    for i in range(0, FROSTING_SEGMENT):
        start = i * segment
        robot.move_global(
            circle_points[start : start + segment], time_step=(3, 0.2), sim=False
        )
        scoop()

def main():
    home()
    circle()

if __name__ == "__main__":
    main()
