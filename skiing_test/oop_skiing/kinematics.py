import numpy as np
import casadi as ca
from robot_model import RobotModel
from config_skate_board import config_skate_board   

# -------------------------------
# Rotation matrices functions (notice the rotation along y have different sign)
# -------------------------------
def R_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

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
        v2 = R_y(theta2) @ np.array([0, 0, -L2])
        v3 = R_y(theta2 + theta3) @ np.array([0, 0, -L3])
        return v1 + v2 + v3
    
    else:
        # hip-roll angle
        v1 = R_x(theta1) @ np.array([0, L1, 0])
        v2 = R_x(theta1) @ R_y(theta2) @ np.array([0, 0, -L2])
        v3 = R_x(theta1) @ R_y(theta2 + theta3) @ np.array([0, 0, -L3])
        return v1 + v2 + v3

# -------------------------------
# Absolute kinematics: p_abs = global_hip_offset + p_rel
# -------------------------------
def forward_kinematics(q, robot:RobotModel=None, side: str = "FR", fix_hip=True):
    """
    global_hip_offset is the position of the hip joint in the world frame

    return the position of the foot in the world frame (global frame)
    """
    # get attributes from robot model based on the side
    L1 = getattr(robot, f"hip_fixed_offset_{side}")[1]
    L2 = robot.L2
    L3 = robot.L3
    global_hip_offset = getattr(robot, f"global_hip_offset_{side}")
    p_rel = forward_kinematics_relative(q, L1=L1, L2=L2, L3=L3, fix_hip=fix_hip)
    return global_hip_offset + p_rel

# -------------------------------
# Function to compute leg positions for visualization
# -------------------------------
def compute_leg_positions(q, robot:RobotModel=None, side:str="FR", fix_hip=True):
    """
    v0 = vector from global base to hip (global_hip_offset)
    v1 = vector from base to hip (after hip-joint rotation)
    v2 = vector from hip to thigh (after thigh-joint rotation)
    v3 = vector from thigh to calf(foot) (after calf-joint rotation)

    """
    LL1 = getattr(robot, f"hip_fixed_offset_{side}").copy()
    LL2 = np.array([0, 0, -robot.L2])
    LL3 = np.array([0, 0, -robot.L3])

    v0 = getattr(robot, f"global_hip_offset_{side}").copy()
    if fix_hip:
        v1 = LL1
        v2 = R_y(q[1]) @ LL2
        v3 = R_y(q[1] + q[2]) @ LL3
    else:
        v1 = R_x(q[0]) @ LL1
        v2 = R_x(q[0]) @ R_y(q[1]) @ LL2
        v3 = R_x(q[0]) @ R_y(q[1] + q[2]) @ LL3
    # this is the real position in 3D, not the joint angle
    hip = v0.copy()
    knee = (hip + v1).copy()
    ankle = (knee + v2).copy()
    foot = (ankle + v3).copy()
    return hip, knee, ankle, foot

# -------------------------------
# Inverse kinematics for 3DoF leg (analytically through numpy)
# -------------------------------
def inverse_kinematics(desired, robot:RobotModel=None, side: str = "FR"):

    """
    desired: 3-dimensional vector (d_x, d_y, d_z) – end effector position in global frame    

    return the joint angles (hip-roll, knee, ankle) in radians
    """

    # get attributes from robot model based on the side
    LL1 = getattr(robot, f"hip_fixed_offset_{side}")[1]
    LL2 = robot.L2
    LL3 = robot.L3
    global_hip_offset = getattr(robot, f"global_hip_offset_{side}")

    # desired: 3-dimensional vector (d_x, d_y, d_z) – end effector position relative to hip (effective- or origin-hip)
    d_x, d_y, d_z = desired - global_hip_offset
    # First, find hip-roll angle q0:
    R_val = np.hypot(d_y, d_z)
    # If R_val is very small, choose q0 = 0 (to avoid division by zero)
    if R_val < 1e-6:
        theta1 = 0.0
    else:
        theta1 = np.arccos(LL1 / R_val) - 0.5 * np.pi + np.arctan2(d_y, -d_z)
    # fake and real length of thigh + calf (fake when we look from the front side of the robot)
    L23_fake = np.sqrt((d_y - LL1 * np.cos(theta1)) ** 2 + (d_z - LL1 * np.sin(theta1)) ** 2)
    L23_real = np.sqrt(d_x ** 2 + L23_fake ** 2)
    
    # calculate theta3 (ankle angle); -1 as solution for current configuration
    theta3 = -1 * np.arccos((L23_real ** 2 - LL2 ** 2 - LL3 ** 2) / (2 * LL2 * LL3))
    if np.isnan(theta3):
        # log all the values
        print(f"LL2: {LL2}, LL3: {LL3}, L23_real: {L23_real}")
        print(f"d_x: {d_x}, d_y: {d_y}, d_z: {d_z}")
        print(f"theta1: {theta1}, L23_fake: {L23_fake}")
        print(f"theta3: {theta3}")
        print("the value inside acos is out of range: ", (L23_real ** 2 - LL2 ** 2 - LL3 ** 2) / (2 * LL2 * LL3))
        raise ValueError("theta3 calculation resulted in NaN. Check the input values.")
        

    # calculate theta2 (knee angle)
    a2 = np.arcsin(LL3 * np.sin(theta3) / L23_real)
    theta2 = -np.arccos(L23_fake / L23_real) - a2
    if np.isnan(theta2):
        # log all the values
        print(f"LL2: {LL2}, LL3: {LL3}, L23_real: {L23_real}")
        print(f"d_x: {d_x}, d_y: {d_y}, d_z: {d_z}")
        print(f"theta1: {theta1}, L23_fake: {L23_fake}")
        print(f"theta3: {theta3}, a2: {a2}")
        print(f"theta2: {theta2}")
        raise ValueError("theta2 calculation resulted in NaN. Check the input values.")

    return np.array([theta1, theta2, theta3])

# -------------------------------
# Functions for calculating robot CoM
# -------------------------------
# TODO: check these 2 functions 
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

def moving_body(robot:RobotModel=None, final_trunk_center=None, q_initial=None, N:int=100, fix_hip=True):
    """
    we have the intial configuration of the legs, then find it's foot position of these legs
    try to move the body to the new position but don't change the foot position

    final_trunk_center is the new position of the trunk center in global frame
    final_trunk_center = [x, y, z]
    q_initial is the initial configuration for 4 legs of the robot
    q_initial = [q_FL, q_FR, q_RL, q_RR]
    N: is the number of steps for moving the body


    return the trajectory of the joint angles for 4 legs that move the body
    """

    # calculate the initial configuration of the foot
    foot_FL = forward_kinematics(q_initial[0, :].squeeze(), robot, side="FL", fix_hip=fix_hip)
    foot_FR = forward_kinematics(q_initial[1, :].squeeze(), robot, side="FR", fix_hip=fix_hip)
    foot_RL = forward_kinematics(q_initial[2, :].squeeze(), robot, side="RL", fix_hip=fix_hip)
    foot_RR = forward_kinematics(q_initial[3, :].squeeze(), robot, side="RR", fix_hip=fix_hip)

    # change the trunk position
    robot.set_trunk_center(final_trunk_center)

    # calculate the new configuration of the foot
    q_final = np.zeros_like(q_initial)
    q_final[0,:] = inverse_kinematics(foot_FL, robot, side="FL")
    q_final[1,:] = inverse_kinematics(foot_FR, robot, side="FR")
    q_final[2,:] = inverse_kinematics(foot_RL, robot, side="RL")
    q_final[3,:] = inverse_kinematics(foot_RR, robot, side="RR")

    # interpolate the joint angles
    interp = np.linspace(0, 1, num=N)
    q_initial = q_initial.reshape(-1)
    q_final = q_final.reshape(-1)

    q_trajectory = np.array([q_initial + (q_final - q_initial) * t for t in interp])

    return q_trajectory
