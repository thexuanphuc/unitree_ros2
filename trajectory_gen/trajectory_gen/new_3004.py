import casadi as ca
import numpy as np
import xml.etree.ElementTree as ET
import scipy.optimize as opt
import math
import sys
import json


# -------------------------------
# Configuration
# -------------------------------
default_config = {
    "urdf_file": "/home/phuc/working/a1_sim/unitree_ros2/skiing_test/a1.urdf",
    "standing_height": 0.35,      # approximate foot height
    "step_length": 0.2,
    "swing_height": 0.07,
    "trajectory_type": "two_phase",
    "N": 100,
    "dt": 0.01,
    "dq_max": 28.6,
    "fix_hip": False,             # in MPC mode fix hip; for IK - don't fix q[0]
    "q0": [0.0, 2.5, -1],
    "w_pos": 10000.0,
    "w_dq": 0.1,
    "w_smooth": 0.1,
    "w_y": 1000.0,
    "joint_limits": {
         "q_min": [-0.80, -1.05, -2.70],
         "q_max": [ 0.80,  4.19, -0.92]
    },
    # Parameter for shifting the center of mass (trunk) in com_transfering mode
    "com_shifting": 0.08  
}

hip_target = np.deg2rad(0)  # initial hip-roll value (in radians)

# -------------------------------
# Function to read offset from URDF by joint name
# -------------------------------
def read_offset(urdf_file, joint_name):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            origin = joint.find("origin")
            if origin is not None and "xyz" in origin.attrib:
                return np.array([float(v) for v in origin.attrib["xyz"].split()])
    return None

# -------------------------------
# Robot model class (reading URDF and kinematics)
# -------------------------------
class RobotModel:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.L1, self.L2, self.L3 = self.read_link_lengths_from_urdf()
        self.trunk_dims = self.read_trunk_dimensions_from_urdf()
        self.hip_offset_FR = read_offset(urdf_file, "FR_hip_joint")
        self.hip_fixed_offset_FR = read_offset(urdf_file, "FR_hip_fixed")
        self.hip_offset_FL = read_offset(urdf_file, "FL_hip_joint")
        self.hip_fixed_offset_FL = read_offset(urdf_file, "FL_hip_fixed")
        self.hip_offset_RR = read_offset(urdf_file, "RR_hip_joint")
        self.hip_fixed_offset_RR = read_offset(urdf_file, "RR_hip_fixed")
        self.hip_offset_RL = read_offset(urdf_file, "RL_hip_joint")
        self.hip_fixed_offset_RL = read_offset(urdf_file, "RL_hip_fixed")
        self.link_masses = self.read_link_masses_from_urdf()

    def read_link_lengths_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        L1, L2, L3 = None, None, None
        for joint in root.findall("joint"):
            if joint.get("name") == "FR_hip_fixed":
                origin = joint.find("origin")
                if origin is not None and "xyz" in origin.attrib:
                    values = [float(v) for v in origin.attrib["xyz"].split()]
                    L1 = np.linalg.norm(values)
                break
        for link in root.findall("link"):
            if link.get("name") == "FR_thigh":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            sizes = [float(v) for v in box.attrib["size"].split()]
                            L2 = max(sizes)
                break
        for link in root.findall("link"):
            if link.get("name") == "FR_calf":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            sizes = [float(v) for v in box.attrib["size"].split()]
                            L3 = max(sizes)
                break
        if L1 is None or L2 is None or L3 is None:
            print("Error reading link lengths from URDF - using default values.")
            L1, L2, L3 = 0.085, 0.20, 0.20
        print("Link lengths: L1 = {:.3f}, L2 = {:.3f}, L3 = {:.3f}".format(L1, L2, L3))
        return L1, L2, L3

    def read_trunk_dimensions_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        trunk_dims = None
        for link in root.findall("link"):
            if link.get("name") == "trunk":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            trunk_dims = [float(v) for v in box.attrib["size"].split()]
                break
        if trunk_dims is None:
            trunk_dims = [0.267, 0.194, 0.114]
        print("Trunk dimensions (x,y,z):", trunk_dims)
        return trunk_dims

    def read_link_masses_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        masses = {}
        for link in root.findall("link"):
            mass = None
            inertial = link.find("inertial")
            if inertial is not None:
                mass_elem = inertial.find("mass")
                if mass_elem is not None and "value" in mass_elem.attrib:
                    mass = float(mass_elem.attrib["value"])
            if mass is not None:
                masses[link.get("name")] = mass
        print("Link masses from URDF:", masses)
        return masses

# -------------------------------
# Rotation matrices functions
# -------------------------------
def R_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def R_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle),  np.cos(angle)]])

# -------------------------------
# Forward kinematics (with hip-roll consideration)
# -------------------------------
def forward_kinematics_relative(q, robot, fix_hip=True):
    if fix_hip:
        q1 = q[1]
        q2 = q[2]
        thigh_vector = R_y(q1) @ np.array([robot.L2, 0, 0])
        calf_vector  = R_y(q1+q2) @ np.array([robot.L3, 0, 0])
        return ca.vertcat(thigh_vector[0] + calf_vector[0],
                          0,
                          thigh_vector[2] + calf_vector[2])
    else:
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        thigh_vector = R_y(q1) @ np.array([robot.L2, 0, 0])
        calf_vector  = R_y(q1+q2) @ np.array([robot.L3, 0, 0])
        p_local = thigh_vector + calf_vector
        p_rel = R_x(q0) @ p_local
        return ca.vertcat(p_rel[0], p_rel[1], p_rel[2])

# -------------------------------
# Absolute kinematics: p_abs = effective_hip_offset + p_rel
# -------------------------------
def forward_kinematics(q, robot, effective_hip_offset, fix_hip=True):
    p_rel = forward_kinematics_relative(q, robot, fix_hip)
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
        calf  = R_y(q[1]+q[2]) @ np.array([robot.L3, 0, 0])
        foot  = ankle + calf
    else:
        q0, q1, q2 = q
        thigh_local = R_y(q1) @ np.array([robot.L2, 0, 0])
        thigh = R_x(q0) @ thigh_local
        knee = hip.copy()
        ankle = knee + thigh
        calf_local = R_y(q1+q2) @ np.array([robot.L3, 0, 0])
        calf = R_x(q0) @ calf_local
        foot = ankle + calf
    return hip, knee, ankle, foot

# -------------------------------
# Inverse kinematics for 3DoF leg (analytically through numpy)
# -------------------------------
def inverse_kinematics(desired, robot):
    # desired: 3-dimensional vector (d_x, d_y, d_z) – end effector position relative to hip (effective)
    d_x, d_y, d_z = desired
    # First, find hip-roll angle q0:
    R_val = np.hypot(d_y, d_z)
    # If R_val is very small, choose q0 = 0 (to avoid division by zero)
    if R_val < 1e-6:
        q0 = 0.0
    else:
        q0 = np.arctan2(d_y, -d_z)
    # Define scalar R for 2R manipulator:
    R_des = R_val
    A = robot.L2
    B = robot.L3
    # Standard equation for 2 links:
    D = (d_x**2 + R_des**2 - A**2 - B**2) / (2 * A * B)
    D = np.clip(D, -1.0, 1.0)
    # Choose "elbow down" solution (q2 negative)
    q2 = -np.arccos(D)
    # Calculate q1:
    q1 = np.arctan2(R_des, d_x) - np.arctan2(B * np.sin(q2), A + B * np.cos(q2))
    return np.array([q0, q1, q2])

# -------------------------------
# Generate trajectory for FR leg (relative to hip)
# -------------------------------
def generate_trajectory(N, step_length, swing_height,  effective_hip_offset, p0_abs, desired_y):
    N1 = N // 2
    N2 = N - N1
    p_ref_list = []
    # Dragging phase: end effector moves on the ground (z = 0)
    for k in range(N1+1):
        alpha = k / N1
        x_val = -step_length * alpha
        z_val = 0.0
        p_ref_list.append([x_val, desired_y, z_val])
    # Lifting phase: end effector lifts up and down
    for k in range(1, N2+1):
        alpha = k / N2
        x_val = -step_length + step_length * alpha
        z_val = swing_height * np.sin(np.pi * alpha)
        p_ref_list.append([x_val, desired_y, z_val])
    p_ref_arr = np.array(p_ref_list)
    ref_offset = p0_abs - effective_hip_offset
    p_ref_arr = p_ref_arr + effective_hip_offset + ref_offset - np.array([0, 0, p0_abs[2]])
    return p_ref_arr

# -------------------------------
# MPC controller for FR leg
# -------------------------------
class MPCController:
    def __init__(self, robot, config, q0):
        self.robot = robot
        self.N = config["N"]
        self.dt = config["dt"]
        self.fix_hip = config["fix_hip"]
        self.q0 = q0
        self.dq_max = config["dq_max"]
        self.w_pos = config["w_pos"]
        self.w_dq = config["w_dq"]
        self.w_smooth = config["w_smooth"]
        self.w_y = config["w_y"]
        self.q_min = np.array(config["joint_limits"]["q_min"])
        self.q_max = np.array(config["joint_limits"]["q_max"])
        self.opti = ca.Opti()
        self.q_vars  = [self.opti.variable(3) for _ in range(self.N+1)]
        self.dq_vars = [self.opti.variable(3) for _ in range(self.N)]
    
    def setup_problem(self, p_ref_arr, effective_hip_offset, desired_y):
        self.opti.subject_to(self.q_vars[0] == self.q0)
        for k in range(self.N):
            self.opti.subject_to(self.q_vars[k+1] == self.q_vars[k] + self.dq_vars[k]*self.dt)
            self.opti.subject_to(self.q_vars[k] >= self.q_min)
            self.opti.subject_to(self.q_vars[k] <= self.q_max)
            self.opti.subject_to(self.dq_vars[k] >= -self.dq_max)
            self.opti.subject_to(self.dq_vars[k] <= self.dq_max)
        self.opti.subject_to(self.q_vars[self.N] >= self.q_min)
        self.opti.subject_to(self.q_vars[self.N] <= self.q_max)

        cost = 0
        for k in range(self.N+1):
            p_foot = forward_kinematics(self.q_vars[k], self.robot, effective_hip_offset, self.fix_hip)
            p_ref_k = ca.DM(p_ref_arr[k])
            cost += self.w_pos * ca.sumsqr(p_foot - p_ref_k)
            cost += self.w_y * (p_foot[1] - desired_y)**2
        for k in range(self.N):
            cost += self.w_dq * ca.sumsqr(self.dq_vars[k])
        for k in range(1, self.N):
            cost += self.w_smooth * ca.sumsqr(self.dq_vars[k] - self.dq_vars[k-1])
        self.opti.minimize(cost)
    
    def solve(self):
        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N):
            self.opti.set_initial(self.dq_vars[k], 0.0)
        sol = self.opti.solve()
        q_sol = np.array([sol.value(self.q_vars[k]) for k in range(self.N+1)])
        return q_sol





# -------------------------------
# Functions for calculating robot CoM
# -------------------------------
def compute_leg_com(positions, leg_prefix, link_masses):
    # positions: (hip, knee, ankle, foot)
    hip, knee, ankle, foot = positions
    mass_hip   = link_masses.get(f"{leg_prefix}_hip", 0)
    mass_thigh = link_masses.get(f"{leg_prefix}_thigh", 0)
    mass_calf  = link_masses.get(f"{leg_prefix}_calf", 0)
    mass_foot  = link_masses.get(f"{leg_prefix}_foot", 0)
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


# -------------------------------
# MPC mode (mpc_pushing)
# -------------------------------
def run_mpc_pushing():
    config = default_config.copy()
    config["fix_hip"] = False  # FR leg with free hip, but for IK MPC we set q0 separately
    robot = RobotModel(config["urdf_file"])
    distance_between_skate_and_robot = 0.21
    board_top_z = 0.058
    standing_height = board_top_z + distance_between_skate_and_robot
    trunk_center = np.array([0.0, 0.0, standing_height])
    config["standing_height"] = standing_height

    q0_all = np.array(config["q0"])
    effective_hip_offset_FR = trunk_center + (robot.hip_offset_FR + robot.hip_fixed_offset_FR)
    effective_hip_offset_FL = trunk_center + (robot.hip_offset_FL + robot.hip_fixed_offset_FL)
    effective_hip_offset_RR = trunk_center + (robot.hip_offset_RR + robot.hip_fixed_offset_RR)
    effective_hip_offset_RL = trunk_center + (robot.hip_offset_RL + robot.hip_fixed_offset_RL)
    
    desired_y = effective_hip_offset_FR[1]
    
    # Configuration for static legs (FL, RR, RL) - calculated as in previous code
    q_static = q0_all.copy()
    def f_static(q2):
        p_foot_z = effective_hip_offset_FL[2] - (
            robot.L2 * np.sin(q_static[1]) +
            robot.L3 * np.sin(q_static[1] + q2)
        )
        return p_foot_z - board_top_z
    q_static[2] = opt.brentq(f_static, -2.7, -0.92)
    def fk_numeric(q):
        p = forward_kinematics(q, robot, effective_hip_offset_FR, config["fix_hip"])
        return np.array(ca.DM(p)).flatten()
    p0_abs_static = fk_numeric(q_static)
    
    # For FR leg, solve inverse kinematics analytically
    d_x = p0_abs_static[0] - effective_hip_offset_FR[0]
    d_y = 0  # desired - foot should be directly under hip
    d_z = board_top_z - effective_hip_offset_FR[2]
    q0_FR = inverse_kinematics(np.array([d_x, d_y, d_z]), robot)
    print("Initial FR configuration (for MPC):", q0_FR)
    p0_abs_FR = fk_numeric(q0_FR)
    
    p_ref_arr = generate_trajectory(config["N"], config["step_length"], config["swing_height"],
                                    effective_hip_offset_FR, p0_abs_FR, desired_y)
    mpc = MPCController(robot, config, q0_FR)
    mpc.setup_problem(p_ref_arr, effective_hip_offset_FR, desired_y)
    q_sol_FR = mpc.solve()

    joint_positions_FR = []
    foot_traj_FR = []
    for q in q_sol_FR:
        pos = compute_leg_positions(q, effective_hip_offset_FR, robot, config["fix_hip"])
        joint_positions_FR.append(pos)
        foot_traj_FR.append(pos[3])
    joint_positions_FR = np.array(joint_positions_FR)
    foot_traj_FR = np.array(foot_traj_FR)

    print("Joint trajectory for FR leg:", foot_traj_FR.shape)

    static_positions_FL = compute_leg_positions(q_static, effective_hip_offset_FL, robot, fix_hip=True)
    static_positions_RR = compute_leg_positions(q_static, effective_hip_offset_RR, robot, fix_hip=True)
    static_positions_RL = compute_leg_positions(q_static, effective_hip_offset_RL, robot, fix_hip=True)

    print("Link lengths:")
    for leg, pos in zip(["FR", "FL", "RR", "RL"], [joint_positions_FR[0], static_positions_FL, static_positions_RR, static_positions_RL]):
        lk = np.linalg.norm(pos[1] - pos[0])
        la = np.linalg.norm(pos[2] - pos[1])
        lf = np.linalg.norm(pos[3] - pos[2])
        print(f"{leg}: Hip-Knee: {lk:.3f}, Knee-Ankle: {la:.3f}, Ankle-Foot: {lf:.3f}")
    return foot_traj_FR

# -------------------------------
# com_transfering mode (trunk movement with fixed feet)
# -------------------------------
def run_com_transfering():
    config = default_config.copy()
    robot = RobotModel(config["urdf_file"])

    distance_between_skate_and_robot = 0.12
    board_top_z = 0.058
    standing_height = board_top_z + distance_between_skate_and_robot
    trunk_center = np.array([0.0, 0.0, standing_height])
    config["standing_height"] = standing_height
    com_shift = config["com_shifting"]

    # Effective offsets
    effective_offsets = {
        "FR": trunk_center + (robot.hip_offset_FR + robot.hip_fixed_offset_FR),
        "FL": trunk_center + (robot.hip_offset_FL + robot.hip_fixed_offset_FL),
        "RR": trunk_center + (robot.hip_offset_RR + robot.hip_fixed_offset_RR),
        "RL": trunk_center + (robot.hip_offset_RL + robot.hip_fixed_offset_RL)
    }
    # IK for static leg pose (each foot on the board)
    q_static = {}
    foot_static = {}
    for leg in effective_offsets:
        d0 = np.array([0.0, 0.0, board_top_z - effective_offsets[leg][2]])
        q_static[leg] = inverse_kinematics(d0, robot)
        foot_static[leg] = effective_offsets[leg] + np.array(
            forward_kinematics_relative(q_static[leg], robot, fix_hip=False)
        ).flatten()

    # Create CYCLE: 0 -> +com_shift -> 0 -> -com_shift -> 0
    cycle_points = [0.0, +com_shift, 0.0, -com_shift, 0.0]
    num_frames = config["N"]
    # Evenly distribute frames across these segments
    segment_len = num_frames // (len(cycle_points) - 1)

    trunk_ys = []
    for i in range(len(cycle_points)-1):
        startY = trunk_center[1] + cycle_points[i]
        endY   = trunk_center[1] + cycle_points[i+1]
        for j in range(segment_len):
            alpha = j / (segment_len - 1)
            y_val = (1-alpha)*startY + alpha*endY
            trunk_ys.append(y_val)

    # If frames are slightly fewer/more, we can truncate/extend
    trunk_traj = []
    for y in trunk_ys:
        trunk_traj.append([trunk_center[0], y, trunk_center[2]])
    trunk_traj = np.array(trunk_traj)

    # Collect leg and CoM trajectories
    leg_trajs = {leg: [] for leg in effective_offsets}
    com_traj = []
    for i in range(len(trunk_traj)):
        new_trunk = trunk_traj[i]
        current_leg_positions = {}
        for leg in effective_offsets:
            # "dynamic" offset
            if leg == "FR":
                hip_off = robot.hip_offset_FR + robot.hip_fixed_offset_FR
            elif leg == "FL":
                hip_off = robot.hip_offset_FL + robot.hip_fixed_offset_FL
            elif leg == "RR":
                hip_off = robot.hip_offset_RR + robot.hip_fixed_offset_RR
            else:
                hip_off = robot.hip_offset_RL + robot.hip_fixed_offset_RL
            new_eff = new_trunk + hip_off

            # foot should stay in place at foot_static[leg]
            desired_foot_world = foot_static[leg]
            d_new = desired_foot_world - new_eff
            q_new = inverse_kinematics(d_new, robot)
            pos = compute_leg_positions(q_new, new_eff, robot, fix_hip=False)
            current_leg_positions[leg] = pos
        for leg in current_leg_positions:
            leg_trajs[leg].append(current_leg_positions[leg])

        c = compute_robot_com(new_trunk, current_leg_positions, robot.link_masses)
        com_traj.append(c)

    for leg in leg_trajs:
        leg_trajs[leg] = np.array(leg_trajs[leg])
    com_traj = np.array(com_traj)
    # print("Leg trajectories in transfering COM:", leg_trajs.shape)
    # print("the com trajectory in transfering mode is:", com_traj)

    return leg_trajs

# -------------------------------
# Main program
# -------------------------------
def run_mpc():
    mode = "mpc_pushing"
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    if mode == "com_transfering":
        joint_trajectory = run_com_transfering()
    else:
        joint_trajectory = run_mpc_pushing()
    print("Saving the joint trajectory to joint_trajectory.csv")
    np.savetxt("joint_trajectory.csv", joint_trajectory, delimiter=",")
    return joint_trajectory
if __name__ == "__main__":
    run_mpc()