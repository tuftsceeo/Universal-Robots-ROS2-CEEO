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
FROSTING_START_JOINTS = [4.02, -1.98, -2.34, -2.01, -0.78, -3.08]
FROSTING_LENGTH = 0.15
FROSTING_DROP_HEIGHT = 0.01
NUM_SCOOP = 0
FROSTING_SEGMENT = 4


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

def scoop(length,height):
    global NUM_SCOOP # count number of scoop for varying depth

    robot.move_joints(
        [FROSTING_HALFWAY_JOINTS, FROSTING_START_JOINTS], time_step=2
    )

    # Create scoop trajectory
    frosting_scoop = []
    for i in range(100):
        frosting_scoop.append(
            [
                0.303,
                0.059 - (i * length / 100),
                0.091 - height * NUM_SCOOP,
                -69.28,
                84.67,
                -22.26,
            ]
        )

    robot.move_global(frosting_scoop, time_step=(0.1, 0.02))
    robot.move_joints([FROSTING_HALFWAY_JOINTS], time_step=2)
    frosting_iter += 1


def circle():
    steps = 100
    circle_points = []
    
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

def arc(divisor, segment = 0, steps=100, center = CAKE_CENTER, radius = EFFECTIVE_RADIUS):
    # segment should be in range(divisor)
    # i.e. if divisor = 5, segment should be 0, 1, 2, 3, or 4
    
    angles = []
    for i in range(steps):
        ith_angle = 360/divisor * (segment + i/steps)
        angles.append(ith_angle)
    arc_points = []
    for theta in angles:
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        arc_points.append(
            [x, y, LOWEST_Z, 0, 0.1, angle_clean(theta + 270)]
        )
    robot.move_global(arc_points, time_step = (3, 0.1))

def main():
    home()
    for i in range(4):
        scoop(FROSTING_LENGTH,FROSTING_DROP_HEIGHT)
        arc(FROSTING_SEGMENT,i)

if __name__ == "__main__":
    main()
