import numpy as np
import casadi as ca

# -------------------------------
# Rotation matrices functions
# -------------------------------
def R_y(angle):
    return np.array([[np.cos(angle), 0, -np.sin(angle)],
                     [0, 1, 0],
                     [np.sin(angle), 0, np.cos(angle)]])

def R_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

# -------------------------------
# Forward kinematics (with hip-roll consideration)
# -------------------------------
def forward_kinematics_relative(q, L1, L2, L3, fix_hip=True):
    """
    L1: offset from base to hip (depends on the side)
    L2: length of thigh (usually constant)
    L3: length of calf (usually constant)

    v1 = vector from base to hip (after hip-joint rotation)
    v2 = vector from hip to thigh (after thigh-joint rotation)
    v3 = vector from thigh to calf(foot) (after calf-joint rotation)
    """
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]

    if fix_hip:
        v1 = np.array([0, L1, 0])
        v2 = R_y(theta2) @ np.array([L2, 0, 0])
        v3 = R_y(theta2 + theta3) @ np.array([L3, 0, 0])
        return v1 + v2 + v3
    
    else:
        # hip-roll angle
        v1 = R_x(theta1) @ np.array([0, L1, 0])
        v2 = R_x(theta1) @ R_y(theta2) @ np.array([L2, 0, 0])
        v3 = R_x(theta1) @ R_y(theta2 + theta3) @ np.array([L3, 0, 0])
        return v1 + v2 + v3

# -------------------------------
# Absolute kinematics: p_abs = effective_hip_offset + p_rel
# -------------------------------
def forward_kinematics(q, robot, effective_hip_offset, fix_hip=True):
    """
    effective_hip_offset is the position of the hip joint in the world frame
    """
    p_rel = forward_kinematics_relative(q, L1=robot.L1, L2=robot.L2, L3=robot.L3, fix_hip=fix_hip)
    return effective_hip_offset + p_rel

# -------------------------------
# Function to compute leg positions for visualization
# -------------------------------
def compute_leg_positions(q, effective_offset, robot, fix_hip=True):
    hip = effective_offset.copy()
    if fix_hip:
        knee = hip.copy()
        thigh = R_y(q[1]) @ np.array([robot.L2, 0, 0])
        ankle = knee + thigh
        calf = R_y(q[1] + q[2]) @ np.array([robot.L3, 0, 0])
        foot = ankle + calf
    else:
        q0, q1, q2 = q
        thigh_local = R_y(q1) @ np.array([robot.L2, 0, 0])
        thigh = R_x(q0) @ thigh_local
        knee = hip.copy()
        ankle = knee + thigh
        calf_local = R_y(q1 + q2) @ np.array([robot.L3, 0, 0])
        calf = R_x(q0) @ calf_local
        foot = ankle + calf

    # this is the real position in 3D, not the joint angle
    return hip, knee, ankle, foot

# -------------------------------
# Inverse kinematics for 3DoF leg (analytically through numpy)
# -------------------------------
def inverse_kinematics(desired, robot):
    print("the length L1, L2, L3:", robot.L1, robot.L2, robot.L3)
    
    # desired: 3-dimensional vector (d_x, d_y, d_z) – end effector position relative to hip (effective)
    d_x, d_y, d_z = desired
    # First, find hip-roll angle q0:
    R_val = np.hypot(d_y, d_z)
    # If R_val is very small, choose q0 = 0 (to avoid division by zero)
    if R_val < 1e-6:
        theta1 = 0.0
    else:
        theta1 = np.arctan2(d_y, -d_z) - np.arccos(robot.L1 / R_val)
    # fake and real length of thigh + calf (fake when we look from the front side of the robot)
    L23_fake = np.sqrt((d_y - robot.L1 * np.sin(theta1)) ** 2 + (d_z - robot.L1 * np.cos(theta1)) ** 2)
    L23_real = np.sqrt(d_x ** 2 + L23_fake ** 2)

    # calculate theta3 (ankle angle); -1 as solution for current configuration
    theta3 = -1 * np.arccos((L23_real ** 2 - robot.L2 ** 2 - robot.L3 ** 2) / (2 * robot.L2 * robot.L3))

    # calculate theta2 (knee angle)
    a2 = np.arcsin(robot.L3 * np.sin(theta3) / L23_real)
    theta2 = -np.arccos(L23_fake / L23_real) - a2

    return np.array([theta1, theta2, theta3])

# -------------------------------
# Functions for calculating robot CoM
# -------------------------------
def compute_leg_com(positions, leg_prefix, link_masses):
    # positions: (hip, knee, ankle, foot)
    hip, knee, ankle, foot = positions
    mass_hip = link_masses.get(f"{leg_prefix}_hip", 0)
    mass_thigh = link_masses.get(f"{leg_prefix}_thigh", 0)
    mass_calf = link_masses.get(f"{leg_prefix}_calf", 0)
    mass_foot = link_masses.get(f"{leg_prefix}_foot", 0)
    com_leg = mass_hip * hip + mass_thigh * ((hip + ankle) / 2) + mass_calf * ((ankle + foot) / 2) + mass_foot * foot
    total_mass_leg = mass_hip + mass_thigh + mass_calf + mass_foot
    return com_leg, total_mass_leg

def compute_robot_com(trunk_center, leg_positions, link_masses):
    mass_trunk = link_masses.get("trunk", 0)
    com_sum = trunk_center * mass_trunk
    total_mass = mass_trunk
    for leg in ['FR', 'FL', 'RR', 'RL']:
        com_leg, mass_leg = compute_leg_com(leg_positions[leg], leg, link_masses)
        com_sum = com_sum + com_leg
        total_mass += mass_leg
    return com_sum / total_mass if total_mass > 0 else trunk_center