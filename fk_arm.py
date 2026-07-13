import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. Define the DH transformation function #
def dh_transform(theta, d, a, alpha):
    """
    Returns a 4x4 homogeneous transformation matrix from DH parameters.
    theta: joint angle (in degrees)
    d: link offset
    a: link length
    alpha: link twist (in degrees)
    """
    theta_rad = np.radians(theta)
    alpha_rad = np.radians(alpha)

    matrix = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad) * np.cos(alpha_rad), np.sin(theta_rad) * np.sin(alpha_rad), a * np.cos(theta_rad)],
        [np.sin(theta_rad),  np.cos(theta_rad) * np.cos(alpha_rad), -np.cos(theta_rad) * np.sin(alpha_rad), a * np.sin(theta_rad)],
        [0,                  np.sin(alpha_rad),                     np.cos(alpha_rad),                     d],
        [0,                  0,                                     0,                                     1]
    ])
    return matrix

# 2. Define the forward kinematics function #
def forward_kinematics(joint_angles, dh_table):
    """
    Calculates the end-effector position given joint angles and a DH table.
    joint_angles: list of angles (in degrees) for each joint.
    dh_table: list of rows, each row is [theta, d, a, alpha].
    """
    T = np.eye(4)

    for i, (theta, d, a, alpha) in enumerate(dh_table):
        T_i = dh_transform(joint_angles[i], d, a, alpha)
        T = np.matmul(T, T_i)

    return T

# 3. 3D visualisation #
def plot_arm_3d(joint_angles, dh_table, title="3D Robotic Arm"):
    """
    Plots the robotic arm in 3D using matplotlib.
    joint_angles: list of angles (in degrees) for each joint.
    dh_table: list of rows, each row is [theta, d, a, alpha].
    """
    T = np.eye(4)
    positions = [(0, 0, 0)]

    for i, (theta, d, a, alpha) in enumerate(dh_table):
        T_i = dh_transform(joint_angles[i], d, a, alpha)
        T = np.matmul(T, T_i)
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        positions.append((x, y, z))

    xs, ys, zs = zip(*positions)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(xs, ys, zs, 'o-', markersize=8, linewidth=3, color='blue', label='Arm')
    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=100, label='End-Effector')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(-3, 3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_title(title)
    ax.legend()
    plt.show()


# 4. Tests


# 2D Planar Arm
dh_table_2d = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 0],
    [0, 0, 0.5, 0]
]
joint_angles_2d = [30, 45, 15]
T = forward_kinematics(joint_angles_2d, dh_table_2d)
print("2D Arm (30°, 45°, 15°):", T[0:3, 3])

# 3D Arm with Twists
dh_table_3d = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 90],
    [0, 0, 0.5, 0]
]
joint_angles_3d = [30, 45, 15]
T = forward_kinematics(joint_angles_3d, dh_table_3d)
print("3D Arm (30°, 45°, 15°):", T[0:3, 3])

# 4D Arm with Twists
dh_table_4d = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 90],
    [0, 0, 0.5, 0],
    [0, 0, 0.3, -90]
]
joint_angles_4d = [0, 0, 0, 0]
T = forward_kinematics(joint_angles_4d, dh_table_4d)
print("4D Arm (0°, 0°, 0°, 0°):", T[0:3, 3])

# 4D Arm with non-zero angles
joint_angles_4dof = [30, 45, 15, -30]
T = forward_kinematics(joint_angles_4dof, dh_table_4d)
print("4D Arm (30°, 45°, 15°, -30°):", T[0:3, 3])

# Visualise 3D Arm
plot_arm_3d(joint_angles_3d, dh_table_3d, title="3D Robotic Arm (30°, 45°, 15°)")
