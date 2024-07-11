import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

r = R.from_quat(
    [
        0.10,
        0.08,
        0.021,
        0.99,
    ]
)
angles = r.as_euler('xyz',degrees=True)
print(angles)

# def make_quat(axis, angle, unit="radians"):
#     if unit == "degrees":
#         angle = angle * 180 / np.pi

#     np.linalg.norm(axis)
#     [i,j,k] = axis

#     return [
#         i * np.sin(angle / 2),
#         j * np.sin(angle / 2),
#         k * np.sin(angle / 2),
#         np.cos(angle / 2),
#     ]

# def make_cone(steps,alpha):
#     quaternions = []
#     for i in range(steps):
#         axis = [np.cos(i * 2 * np.pi / steps), np.sin(i * 2 * np.pi / steps),0]
#         quaternions.append(make_quat(axis,alpha))
#     return quaternions

# def conical_motion_quaternions(alpha, steps):
#     quaternions = []
#     for t in range(steps):
#         theta = 2 * np.pi * t / steps
#         # Rotation around the cone's base (z-axis)
#         Rz = R.from_euler("z", theta)
#         # Tilt downwards by alpha
#         Rx = R.from_euler("y", alpha)
#         # Combine rotations: first tilt, then rotate around the z-axis
#         R_cone = Rz * Rx
#         quaternions.append(R_cone.as_quat())
#     return quaternions


# def plot_conical_motion(quaternions):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection="3d")

#     origin = np.array([0, 0, 0])
#     axis_length = 1

#     for quat in quaternions:
#         r = R.from_quat(quat)
#         direction = r.apply([axis_length, 0, 0])
#         ax.quiver(*origin, *direction, length=1, normalize=True)

#     ax.set_xlim([-1.5, 1.5])
#     ax.set_ylim([-1.5, 1.5])
#     ax.set_zlim([-1.5, 1.5])
#     ax.set_xlabel("X axis")
#     ax.set_ylabel("Y axis")
#     ax.set_zlabel("Z axis")

#     plt.show()


# # Parameters
# alpha = np.radians(45)  # 30 degree cone angle
# steps = 10  # Number of steps around the cone

# # quaternions = conical_motion_quaternions(alpha, steps)
# quaternions = make_cone(steps, alpha)
# plot_conical_motion(quaternions)
