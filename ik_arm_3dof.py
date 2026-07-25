import numpy as np
#[DH TRANSFORMATION FUNCTION]
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

#[FORWARD KINEMATICS FUNCTION]
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

#[REACHABILITY CHECK FUNCTION]

def is_reachable(target_pos, dh_table):
    """
    Checks if the target position (x, y, z) is reachable by the robotic arm.
    target_pos:(x, y, z) target position
    dh_table: list of rows, each row is [theta, d, a, alpha].
    Returns True if reachable, False otherwise
    
    """
    max_reach =sum([row[2] for row in dh_table])
    distance = np.linalg.norm(target_pos)
    return distance <= max_reach

# [CCD INVERSE KINEMATICS SOLVER]

def inverse_kinematics_ccd(target_pos, dh_table, initial_angles=None, max_iterations=1000, tolerance=1e-3):
    """
    Inverse Kinematics using Cyclic Coordinate Descent (CCD) method.
    target_pos: (x, y, z) target position
    dh_table: list of rows, each row is [theta, d, a, alpha].
    initial_angles: optional initial joint angles (in degrees)
    max_iterations: maximum number of iterations
    tolerance: acceptable error in end-effector position
    Returns the joint angles that achieve the target position or None if not reachable.
    """

    if not is_reachable(target_pos, dh_table):
        print("Warning Target position is not reachable.")
        return None


    num_joints = len(dh_table)
    
    if initial_angles is None:
        joint_angles = np.zeros(num_joints)
    else:
        joint_angles = np.array(initial_angles)


    for iteration in range(max_iterations):
        T = forward_kinematics(joint_angles, dh_table)
        current_pos = np.array([T[0, 3], T[1, 3], T[2, 3]])
        
        error = target_pos - current_pos
        distnace = np.linalg.norm(error)
        if distnace < tolerance:
            print(f"Converged in {iteration} iterations.")
            return joint_angles

    for i in range(num_joints - 1, -1, -1):
        T_joint = forward_kinematics(joint_angles[:i+1], dh_table[:i+1])
        joint_pos = np.array([T_joint[0, 3], T_joint[1, 3], T_joint[2, 3]])
        
        vec_to_effector = current_pos - joint_pos
        vec_to_target = target_pos - joint_pos

        if np.linalg.norm(vec_to_effector) < 1e-6 or np.linalg.norm(vec_to_target) < 1e-6:

            continue

        vec_eff_xy = vec_to_effector[:2]
        vec_tgt_xy = vec_to_target[:2]
        
        cross = np.cross(vec_eff_xy, vec_tgt_xy)
        dot = np.dot(vec_eff_xy, vec_tgt_xy)
        angle = np.arctan2(cross, dot)

        angles[i] += np.degrees(angle)
        T = forward_kinematics(joint_angles, dh_table)
        current_pos = np.array([T[0, 3], T[1, 3], T[2, 3]])

        error = target_pos - current_pos
        distnace = np.linalg.norm(error)
        if distnace < tolerance:
            print(f"Converged after {iteration} iterations. (joint{i})")
            return angles

    print("Warning: Max iterations reached without convergence.")
    return None
 #[VERIFICATION FUNCTION]

def verify_solution(target_pos, joint_angles, dh_table, tolerance=1e-3):
    """
    Verifies if the computed joint angles achieve the target position within a specified tolerance.
    target_pos: (x, y, z) target position
    joint_angles: computed joint angles (in degrees)
    dh_table: list of rows, each row is [theta, d, a, alpha].
    tolerance: acceptable error in end-effector position
    Returns True if the solution is valid, False otherwise.
    """
    if solution is None:
        print("No solution to verify.")
        return False

    T = forward_kinematics(solution, dh_table)
    pos = [[T[0, 3], T[1, 3], T[2, 3]]]
    error = np.linalg.norm(np.array(target_pos) - np.array(pos))
     
    return error < 1e-3, pos, error

#[TEST]
def run_test(dh_table, test_angles):
    T = forward_kinematics(test_angles, dh_table)
    target_pos = [T[0, 3], T[1, 3], T[2, 3]]

    print(f"Test angles: {test_angles}")
    print(f"Target position: ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")

    solution = inverse_kinematics_ccd(target_pos, dh_table, initial_angles=test_angles)

    if solution is None:
        print("IK failed to converge")
        print("=" * 50)
        return

    print(f"IK solution: {[round(a, 2) for a in solution]}")

    T_solution = forward_kinematics(solution, dh_table)
    pos = [T_solution[0, 3], T_solution[1, 3], T_solution[2, 3]]
    error = np.linalg.norm(np.array(target_pos) - np.array(pos))

    print(f"Position from IK: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"Error: {error:.6f}")
    print(f"Verification: {'Passed' if error < 1e-3 else 'Failed'}")
    print("=" * 50)

dh_table_3dof = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 0],
    [0, 0, 0.5, 0]
]

dh_table_4dof = [
    [0, 0, 1.0, 0],
    [0, 0, 1.0, 0],
    [0, 0, 0.5, 0],
    [0, 0, 0.3, 0]
]

# RUN TESTS 

print("Tests for 3-DOF robotic arm")
print("=" * 50)
run_test(dh_table_3dof, [30, 45, 15])

print("Tests for 4-DOF robotic arm")
print("=" * 50)
run_test(dh_table_4dof, [30, 45, 15, -30])

print("Test: Unreachable target position")
print("=" * 50)
unreachable_target = [100, 100, 0]
solution = inverse_kinematics_ccd(unreachable_target, dh_table_3dof)
print(f"Unreachable target: {unreachable_target}")
print(f"Solution: {solution}")
print("Status: Passed" if solution is None else "Failed")
print("=" * 50)


