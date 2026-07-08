import numpy as np

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

# DH table for a 2‑joint planar arm
dh_table = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 0]
]

# Test Config 1: θ₁ = 0°, θ₂ = 0°
joint_angles = [0, 0]
T = forward_kinematics(joint_angles, dh_table)
print("Config 1 (0°, 0°):", T[0:3, 3])

# Test Config 2: θ₁ = 30°, θ₂ = 45°
joint_angles = [30, 45]
T = forward_kinematics(joint_angles, dh_table)
print("Config 2 (30°, 45°):", T[0:3, 3])