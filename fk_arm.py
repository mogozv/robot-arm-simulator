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
    T = np.eye(4) #creates a 4x4 identity matrix

    for i, (theta, d, a, alpha) in enumerate(dh_table):##unpacks the values from the dh_table and passes them to the dh_transform function
        T_i = dh_transform(joint_angles[i], d, a, alpha)  #stores the transformation 4x4 matrix for the current joint
        T = np.matmul(T, T_i)

    return T

# DH table for a 2‑joint planar arm
#Extended for a 3-joint planar arm
dh_table = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 0],
    [0, 0, 0.5 ,0],
    [0, 0, 0.3, 0]
]


# Test Config 1: θ₁ = 0°, θ₂ = 0°
joint_angles = [0, 0, 0,0]
T = forward_kinematics(joint_angles, dh_table)
print("Config 1 (0°, 0°, 0°, 0°):", T[0:3, 3])

# Test Config 2: θ₁ = 30°, θ₂ = 45°
joint_angles = [30, 45, 15, -30]
T = forward_kinematics(joint_angles, dh_table)
print("Config 2 (30°, 45°, 15°, -30°):", T[0:3, 3])

import matplotlib.pyplot as plt
def plot_arm(joint_angles, dh_table):
    """"
    Plots the robotic arm in 2D using matplotlib.
    joint_angles: list of angles (in degrees) for each joint.
    dh_table: list of rows, each row is [theta, d, a, alpha].
    """

    T=np.eye(4)
    positions = [(0,0)]  # Starting at the origin)]

    for i, (theta, d, a, alpha) in enumerate(dh_table):
        T_i = dh_transform(joint_angles[i], d, a, alpha)
        T = np.matmul(T, T_i) #multiplies the current transformation matrix with the new one to get the cumulative transformation
        x, y = T[0, 3], T[1, 3]
        positions.append((x, y))

        xs,ys = zip(*positions)

        plt.figure(figsize=(6, 6)) #creates a new figure with a size of 6x6 inches
        plt.plot(xs, ys, 'o-', markersize=8, linewidth=3, color='blue')# draws the arm by plotting the positions of the joints and connecting them with lines
        plt.xlim(-3, 3)#set the plot range for the x-axis from -3 to 3 meters
        plt.ylim(-3, 3)#set the plot range for the y-axis from -3 to 3 meters
        plt.grid(True)#sets the grid to be visible on the plot
        plt.axis('equal')
        plt.title(f"Robotic Arm Configuration: {joint_angles}")
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.show()#shows the plot

        dh_table_4dof = [
            [0, 0, 1.0, 0],
            [0, 0, 1.0, 0],
            [0, 0, 0.5, 0],
            [0, 0, 0.3, 0]
            ]


        joint_angles_4dof = [30, 45, 15, -30]
        T = forward_kinematics(joint_angles_4dof, dh_table_4dof)
        print("Config 2 4-DOF Arm (30°, 45°, 15°, -30°):", T[0:3, 3])

        plot_arm(joint_angles_4dof, dh_table_4dof)

