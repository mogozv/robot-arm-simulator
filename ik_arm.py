import numpy as np


# FK Functions (from my fk_arm file)

def dh_transform(theta, d, a, alpha):
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
    T = np.eye(4)
    for i, (theta, d, a, alpha) in enumerate(dh_table):
        T_i = dh_transform(joint_angles[i], d, a, alpha)
        T = np.matmul(T, T_i)
    return T


# IK Function

def inverse_kinematics_2d(x, y, L1=1.0, L2=1.0):
    r = np.sqrt(x**2 + y**2)

    if r > L1 + L2 or r < abs(L1 - L2):
        return None

    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)

    theta2_up = np.arccos(cos_theta2)
    theta2_down = -theta2_up

    theta1_up = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2_up), L1 + L2 * np.cos(theta2_up))
    theta1_down = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2_down), L1 + L2 * np.cos(theta2_down))

    return [
        (np.degrees(theta1_up), np.degrees(theta2_up)),
        (np.degrees(theta1_down), np.degrees(theta2_down))
    ]

# Test

theta1_deg = 30
theta2_deg = 45
L1, L2 = 1.0, 1.0

dh_table = [
    [0, 0, L1, 0],
    [0, 0, L2, 0]
]

joint_angles = [theta1_deg, theta2_deg]
T = forward_kinematics(joint_angles, dh_table)
x, y = T[0, 3], T[1, 3]
print(f"Target from FK: ({x:.4f}, {y:.4f})")

solutions = inverse_kinematics_2d(x, y, L1, L2)

if solutions:
    for i, (theta1, theta2) in enumerate(solutions):
        print(f"Solution {i+1}: θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°")
else:
    print("Target is unreachable")
