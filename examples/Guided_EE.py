import numpy as np
import time
from myur import MyUR3e

robot = MyUR3e()

# Parameters
FORCE_SCALE = 0.01
MAX_V = 0.2 # meters / sec
FREQ = 0.05
DECAY = 0.05
THRESHOLD = 0.05

# Variables
velocity_vector = np.array([0,0,0])
speed_scalar = 0
trajectory = []
orientation = [0,0,0]

while True:
    # Read force
    force = np.array(robot.read_force()) # need to subtract gravity
    magnitude = np.linalg.norm(force)
    force_vector = force / magnitude

    # Update vector and scalar according to force
    velocity_vector = (velocity_vector + force_vector) / np.linalg.norm(
        velocity_vector + force_vector
    )
    speed_scalar = speed_scalar + force_vector*FORCE_SCALE

    # Cap speed scalar
    if (speed_scalar / FREQ) > MAX_V: speed_scalar = MAX_V * FREQ
    # Decay speed scalar
    if speed_scalar > 0:
        speed_scalar = speed_scalar*(1-DECAY)
    # Zero speed scalar
    if speed_scalar < THRESHOLD: speed_scalar = 0

    # Update relative trajectory
    for i in range(100):
        jump = speed_scalar*(1-DECAY)**i # should match to speed scalar DECAY per iteration
        if jump < THRESHOLD: jump = 0

        motion = (velocity_vector*speed_scalar).tolist()
        trajectory.append(motion+orientation)

    # Replace trajectory
    robot.move_global_r(trajectory,time=FREQ*100,wait=False)
    time.sleep(FREQ)
