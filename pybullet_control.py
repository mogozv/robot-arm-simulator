import pybullet as p
import pybullet_data
import time
import numpy as np

# Analytical IK for a 2‑joint planar arm
def inverse_kinematics_2d_analytical(target_pos, L1=1.0, L2=1.0):
    x, y, _ = target_pos
    r = np.sqrt(x**2 + y**2)

    # Check reachability
    if r > L1 + L2 or r < abs(L1 - L2):
        return None

    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    theta2 = np.arccos(cos_theta2)

    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    return [np.degrees(theta1), np.degrees(theta2)]

def move_arm_to_position(target_pos):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    # Find revolute joints
    joint_indices = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:
            joint_indices.append(i)
    print("Joint indices:", joint_indices)

    # Solve IK analytically
    solution = inverse_kinematics_2d_analytical(target_pos, L1=1.0, L2=1.0)
    if solution is None:
        print("Target unreachable")
        p.disconnect()
        return

    target_angles = [np.radians(angle) for angle in solution]
    print("IK solution (deg):", solution)

    for i, joint_index in enumerate(joint_indices):
        p.setJointMotorControl2(
            robot_id,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=target_angles[i],
            force=200.0
        )

    for _ in range(2000):
        p.stepSimulation()
        time.sleep(1/240)

    # Get end‑effector position
    link_state = p.getLinkState(robot_id, len(joint_indices))
    if link_state is not None:
        position = link_state[0]
        print(f"Target: {target_pos}")
        print(f"Reached: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
    else:
        print("Failed to get link state")

    input("Press Enter to close...")
    p.disconnect()

target = [0.5, 0.3, 0.0]
move_arm_to_position(target)
