import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET
import scipy.optimize as opt
import math
import sys
import json

# -------------------------------
# Configuration
# -------------------------------
default_config = {
    "urdf_file": "a1.urdf",
    "standing_height": 0.35,      # approximate foot height
    "step_length": 0.2,
    "swing_height": 0.07,
    "trajectory_type": "two_phase",
    "N": 500,
    "N_lift": 200,  # number of steps for lifting one foot from skateboard to the floor
    "dt": 0.01,
    "dq_max": 28.6,
    "fix_hip": False,             # in MPC mode fix hip; for IK - don't fix q[0]
    "q0": [0.0, 0.0, -0.9],
    "w_pos": 10000.0,
    "w_dq": 0.001,
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
    return np.array([[np.cos(angle), 0, -np.sin(angle)],
                     [0, 1, 0],
                     [np.sin(angle), 0, np.cos(angle)]])

def R_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle),  np.cos(angle)]])

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
        v3 = R_y(theta2+theta3) @ np.array([L3, 0, 0])
        return v1 + v2 + v3
    
    else:
        # hip-roll angle
        v1 = R_x(theta1) @ np.array([0, L1, 0])
        v2 = R_x(theta1) @ R_y(theta2) @ np.array([L2, 0, 0])
        v3 = R_x(theta1) @ R_y(theta2+theta3) @ np.array([L3, 0, 0])
        return v1 + v2 + v3
        

# -------------------------------
# Absolute kinematics: p_abs = effective_hip_offset + p_rel
# -------------------------------
def forward_kinematics(q, robot, side: str = "FR", effective_hip_offset=None, fix_hip=True):
    """
    Computes the forward kinematics for a given leg of the robot.
    
    Args:
        q: Joint angles.
        robot: Robot model containing hip offsets.
        side (str): One of "FR", "FL", "RR", or "RL".
        effective_hip_offset: Position of the hip joint in the world frame.
        fix_hip (bool): Whether the hip is fixed or not.
        
    Returns:
        The position of the foot in the world frame.
    """
    L1 = getattr(robot, f"hip_fixed_offset_{side}")[1]
    p_rel = forward_kinematics_relative(q, L1=L1, fix_hip=fix_hip)
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
    L23_fake = np.sqrt((d_y - robot.L1 * np.sin(theta1))**2 + (d_z - robot.L1 * np.cos(theta1))**2)
    L23_real = np.sqrt(d_x**2 + L23_fake**2)

    # calculate theta3 (ankle angle); -1 as solution for current configuration
    theta3 = -1 * np.arccos(L23_real**2 - robot.L2**2 - robot.L3**2) / (2 * robot.L2 * robot.L3)

    # calculate theta2 (knee angle)
    a2 = np.arcsin(robot.L3 * np.sin(theta3) / L23_real)
    theta2 = -np.arccos(L23_fake / L23_real) - a2

    return np.array([theta1, theta2, theta3])


# -------------------------------
# MPC controller for FR leg
# -------------------------------
class MPCController:
    def __init__(self, robot, config, q0_FR_on_board, q0_FR_on_floor):
        self.robot = robot
        self.N_pushing = config["N"]
        self.N_floor = int(0.4 * self.N_pushing) # number of steps that the foot lie on the floor
        self.N_lift = config["N_lift"]
        self.dt = config["dt"]
        self.fix_hip = config["fix_hip"]
        self.q0_FR_on_board = q0_FR_on_board  # initial position for lifting part (from the skateboard)
        self.q0_FR_on_floor = q0_FR_on_floor  # initial position for pushing part (on the floor)
        self.dq_max = config["dq_max"]
        self.w_pos = config["w_pos"]
        self.w_dq = config["w_dq"]
        self.w_smooth = config["w_smooth"]
        self.w_y = config["w_y"]
        # TODO move to config
        self.w_board_penalty = 100000.0
        self.q_min = np.array(config["joint_limits"]["q_min"])
        self.q_max = np.array(config["joint_limits"]["q_max"])

        # for pushing MPC
        self.opti_pushing = ca.Opti()
        self.q_vars_pushing  = [self.opti_pushing.variable(3) for _ in range(self.N_pushing+1)]
        self.dq_vars_pushing = [self.opti_pushing.variable(3) for _ in range(self.N_pushing)]


        # for lifting MPC
        self.opti_lifting = ca.Opti()
        self.q_vars_lifting  = [self.opti_lifting.variable(3) for _ in range(self.N_lift+1)]
        self.dq_vars_lifting = [self.opti_lifting.variable(3) for _ in range(self.N_lift)]

    def setup_problem_pushing(self, effective_hip_offset, distance_from_path_to_center):
        """
        this mpc is for pushing the foot on the floor
        self.q_vars_pushing is the joint angle

        TODO: 
            + add constraints for end position
        """

        print("the intial position of the foot on the skateboard:", self.q0_FR_on_floor)    
        print("the angle limits for the joints:", self.q_min, self.q_max)

        # Initial position constraint
        self.opti_pushing.subject_to(self.q_vars_pushing[0] == self.q0_FR_on_floor) 

        # Final position constraint to make it move in loop
        self.opti_pushing.subject_to(self.q_vars_pushing[self.N_pushing] == self.q0_FR_on_floor)

        # Dynamics and bounds for each step
        for k in range(self.N_pushing):
            # kinematics constraints
            self.opti_pushing.subject_to(self.q_vars_pushing[k+1] == self.q_vars_pushing[k] + self.dq_vars_pushing[k]*self.dt)

            # limits for joint angles and velocities
            self.opti_pushing.subject_to(self.opti_pushing.bounded(self.q_min, self.q_vars_pushing[k], self.q_max))
            self.opti_pushing.subject_to(self.opti_pushing.bounded(-self.dq_max, self.dq_vars_pushing[k], self.dq_max))

        # Final position bounds
        self.opti_pushing.subject_to(self.opti_pushing.bounded(self.q_min, self.q_vars_pushing[self.N_pushing], self.q_max))

        cost = 0

        # constraint for trajectory of the foot on the floor
        for k in range(self.N_pushing+1):
            p_foot = forward_kinematics(self.q_vars_pushing[k], self.robot, effective_hip_offset, self.fix_hip)
            
            # constraint the length of the foot trajectory
            if(k == 0):
                self.x_foot_start = p_foot[0]
            if(k == self.N_floor):
                min_push_length = 0.15
                self.opti_pushing.subject_to(self.x_foot_start - p_foot[0] >= min_push_length)


            min_lift_height = 0.025  # 5 cm above the ground
        
            if (k <= self.N_floor):
                # constraint to keep the path on floor straight
                cost += self.w_y * (p_foot[1] - distance_from_path_to_center)**2
                # Ground contact constraint
                self.opti_pushing.subject_to(p_foot[2] == 0)

            elif (k > self.N_floor + 20 and k < self.N_pushing - 20):
                # the part of trajecotry that is not on the floor
                self.opti_pushing.subject_to(p_foot[2] >= min_lift_height)


        for k in range(self.N_pushing):
            #  this cost to minimize the velocity
            #  TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
            cost += self.w_dq * ca.sumsqr(self.dq_vars_pushing[k] - 0)
            
            if k > 0:
                # this cost to smooth the velocity
                # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
                # cost += self.w_smooth * ca.sumsqr(self.dq_vars_pushing[k] - self.dq_vars_pushing[k-1])
                penalty = 1 * self.w_smooth * (self.dq_vars_pushing[k][0] - self.dq_vars_pushing[k-1][0])**2 \
                            + 0.5 * self.w_smooth * (self.dq_vars_pushing[k][1] - self.dq_vars_pushing[k-1][1])**2 \
                            + self.w_smooth * (self.dq_vars_pushing[k][2] - self.dq_vars_pushing[k-1][2])**2  # different weights for hip, knee, ankle
                cost += penalty

        self.opti_pushing.minimize(cost)

    def solve_pushing(self):
        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti_pushing.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N_pushing):
            self.opti_pushing.set_initial(self.q_vars_pushing[k], self.q0_FR_on_floor)
            self.opti_pushing.set_initial(self.dq_vars_pushing[k], 0.0)
        try:
            sol = self.opti_pushing.solve()
        except RuntimeError as e:
            print("Solver failed!")
            print("q0 initial:", self.opti_pushing.debug.value(self.q_vars_pushing[0]))
            print("qN guess:", self.opti_pushing.debug.value(self.q_vars_pushing[-1]))
            raise e        
        
        # get the solution for joint angles
        q_sol = np.array([sol.value(self.q_vars_pushing[k]) for k in range(self.N_pushing+1)])

        # get the solution for joint velocities
        dq_sol = np.array([sol.value(self.dq_vars_pushing[k]) for k in range(self.N_pushing)])
        return q_sol, dq_sol


    def setup_problem_lifting(self, effective_hip_offset):
        """
        this mpc is for lifting the foot from skateboard to the floor
        """
        # Initial position constraint
        # TODO what is the position of the foot on the skateboard
        self.opti_lifting.subject_to(self.q_vars_lifting[0] == self.q0_FR_on_board)

        # Final position of lifting phase should the same as the initial position of pushing phase
        self.opti_lifting.subject_to(self.q_vars_lifting[self.N_lift] == self.q0_FR_on_floor)

        # Dynamics and bounds for each step
        for k in range(self.N_lift):
            # kinematics constraints
            self.opti_lifting.subject_to(self.q_vars_lifting[k+1] == self.q_vars_lifting[k] + self.dq_vars_lifting[k]*self.dt)

            # limits for joint angles and velocities
            self.opti_lifting.subject_to(self.opti_lifting.bounded(self.q_min, self.q_vars_lifting[k], self.q_max))
            self.opti_lifting.subject_to(self.opti_lifting.bounded(-self.dq_max, self.dq_vars_lifting[k], self.dq_max))

        cost = 0
        # constraint to unsure the foot does not go through the skateboard
        for k in range(self.N_lift+1):
            p_foot = forward_kinematics(self.q_vars_lifting[k], self.robot, effective_hip_offset, self.fix_hip)
            
            # if the foot[1] < 0.24 (skate board size + 0.04), then foot[2] should be > 0.058 (skateboard height)
            alpha = -1000  # steepness of the sigmoid
            threshold = - 0.21
            min_height = 0.06

            # Smooth condition: 0 when p_foot[1] < - 0.21, 1 when p_foot[1] > -0.21
            condition = 1 / (1 + ca.exp(alpha * (p_foot[1] - threshold)))  # sigmoid

            # Enforce that if condition is "on", p_foot[2] >= min_height
            penalty = self.w_board_penalty * (ca.fmin(0, p_foot[2] - min_height * condition))**2
            cost += penalty

        for k in range(self.N_lift):
            #  this cost to minimize the velocity
            #  TODO put into threshold function, that velocity should be smaller than some value
            #  but do we really need this one???
            cost += self.w_dq * ca.sumsqr(self.dq_vars_lifting[k] - 0)
            if k > 0:
                # this cost to smooth the velocity
                # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
                penalty = 10 * self.w_smooth * (self.dq_vars_lifting[k][0] - self.dq_vars_lifting[k-1][0])**2\
                            + 2 * self.w_smooth * (self.dq_vars_lifting[k][1] - self.dq_vars_lifting[k-1][1])**2\
                            + self.w_smooth * (self.dq_vars_lifting[k][2] - self.dq_vars_lifting[k-1][2])**2  # different weights for hip, knee, ankle
        
                cost += penalty
        self.opti_lifting.minimize(cost)
    
    def solve_lifting(self):
        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti_lifting.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N_lift):
            self.opti_lifting.set_initial(self.dq_vars_lifting[k], 0.0)
            self.opti_lifting.set_initial(self.q_vars_lifting[k], self.q0_FR_on_board)
        self.opti_lifting.set_initial(self.q_vars_lifting[self.N_lift], self.q0_FR_on_board)
        
        sol = self.opti_lifting.solve()
        # get the solution for joint angles
        q_sol = np.array([sol.value(self.q_vars_lifting[k]) for k in range(self.N_lift+1)])
        # get the solution for joint velocities
        dq_sol = np.array([sol.value(self.dq_vars_lifting[k]) for k in range(self.N_lift)])
        return q_sol, dq_sol

# -------------------------------
# Function for drawing ground surface
# -------------------------------
def draw_ground(ax, allX, allY, ground_z=0, color='lightgreen', alpha=0.3):
    x_min, x_max = np.min(allX), np.max(allX)
    y_min, y_max = np.min(allY), np.max(allY)
    vertices = np.array([
        [x_min, y_min, ground_z],
        [x_max, y_min, ground_z],
        [x_max, y_max, ground_z],
        [x_min, y_max, ground_z]
    ])
    ground = Poly3DCollection([vertices], color=color, alpha=alpha)
    ax.add_collection3d(ground)

# -------------------------------
# Functions for drawing 3D objects (trunk, skateboard)
# -------------------------------
def draw_box(ax, lower_corner, width, depth, height, edgecolor='green', facecolor=(0,0,0,0), alpha=0.25):
    x, y, z = lower_corner
    vertices = np.array([
        [x, y, z],
        [x+width, y, z],
        [x+width, y+depth, z],
        [x, y+depth, z],
        [x, y, z+height],
        [x+width, y, z+height],
        [x+width, y+depth, z+height],
        [x, y+depth, z+height]
    ])
    faces = [
        [vertices[j] for j in [0,1,2,3]],
        [vertices[j] for j in [4,5,6,7]],
        [vertices[j] for j in [0,1,5,4]],
        [vertices[j] for j in [1,2,6,5]],
        [vertices[j] for j in [2,3,7,6]],
        [vertices[j] for j in [3,0,4,7]]
    ]
    box = Poly3DCollection(faces, linewidths=1, edgecolors=edgecolor, alpha=alpha)
    box.set_facecolor(facecolor)
    ax.add_collection3d(box)

def draw_skateboard(ax,
                    board_length=0.7,
                    board_width=0.4,
                    board_thickness=0.02,
                    wheel_diameter=0.056,
                    wheel_width=0.04,
                    wheel_gap=0.01):
    wheel_radius = wheel_diameter/2
    z_bottom_board = wheel_radius + wheel_gap
    lower_corner = np.array([-board_length/2, -board_width/2, z_bottom_board])
    # Skateboard is transparent (alpha=0.3)
    draw_box(ax, lower_corner, board_length, board_width, board_thickness,
             edgecolor='black', facecolor=(0.5,0.3,0.1,0.3), alpha=0.3)
    x_wheels = [0.30, 0.30, -0.30, -0.30]
    y_wheels = [0.08, -0.08, 0.08, -0.08]
    for (xw, yw) in zip(x_wheels, y_wheels):
        center = (xw, yw, wheel_radius)
        theta = np.linspace(0, 2*np.pi, 24)
        xs = center[0] + wheel_radius*np.cos(theta)
        zs = center[2] + wheel_radius*np.sin(theta)
        ys = np.full_like(theta, center[1])
        ax.plot(xs, ys, zs, color='gray')

# -------------------------------
# Function for setting equal axes scales
# -------------------------------
def set_axes_equal_3d(ax, X, Y, Z):
    x_min, x_max = np.min(X), np.max(X)
    y_min, y_max = np.min(Y), np.max(Y)
    z_min, z_max = np.min(Z), np.max(Z)
    if z_min < 0:
        z_min_new = 0
    else:
        z_min_new = z_min
    range_x = x_max - x_min
    range_y = y_max - y_min
    range_z = z_max - z_min_new
    max_range = max(range_x, range_y, range_z)
    mid_x = 0.5*(x_min + x_max)
    mid_y = 0.5*(y_min + y_max)
    ax.set_xlim3d(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim3d(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim3d(z_min_new, z_min_new + max_range)

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
# Animation for MPC mode (mpc_pushing)
# -------------------------------
def animate_mpc(joint_positions_FR, foot_traj_FR, trunk_dims, trunk_center,
                static_positions_FL, static_positions_RR, static_positions_RL, board_top_z, link_masses):
    fig = plt.figure(figsize=(12,10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("3D Animation: MPC Leg Control and CoM Dynamics")
    
    draw_skateboard(ax)
    tw, td, th = trunk_dims
    lower_box = trunk_center - np.array([tw/2, td/2, th/2])
    draw_box(ax, lower_box, tw, td, th, edgecolor='green', facecolor=(0,1,0,0.3))
    
    colors_FR = ['skyblue', 'blue', 'navy']
    lines_FR = [ax.plot([], [], [], color=c, lw=3)[0] for c in colors_FR]
    colors_FL = ['lightcoral', 'red', 'darkred']
    lines_FL = [ax.plot([], [], [], color=c, lw=3)[0] for c in colors_FL]
    colors_RR = ['lightgreen', 'green', 'darkgreen']
    lines_RR = [ax.plot([], [], [], color=c, lw=3)[0] for c in colors_RR]
    colors_RL = ['plum', 'purple', 'indigo']
    lines_RL = [ax.plot([], [], [], color=c, lw=3)[0] for c in colors_RL]
    
    line_FR_traj, = ax.plot([], [], [], 'k--', lw=1, label='FR Foot Trajectory')
    com_marker, = ax.plot([], [], [], 'o', color='magenta', markersize=8, label='CoM Projection')
    
    # Stability triangle for three static legs: FL, RR, RL
    tri_vertices = np.array([
        [static_positions_FL[3][0], static_positions_FL[3][1], board_top_z],
        [static_positions_RR[3][0], static_positions_RR[3][1], board_top_z],
        [static_positions_RL[3][0], static_positions_RL[3][1], board_top_z]
    ])
    stability_triangle = Poly3DCollection([tri_vertices], facecolor=(0, 0.8, 0, 0.3), edgecolor='k', lw=2, label='Stability Triangle')
    ax.add_collection3d(stability_triangle)
    
    handles = []
    labels = []
    for i, c in enumerate(colors_FR):
        h, = ax.plot([], [], [], color=c, lw=3)
        handles.append(h)
        if i == 0:
            labels.append("FR Hip-Knee")
        elif i == 1:
            labels.append("FR Knee-Ankle")
        elif i == 2:
            labels.append("FR Ankle-Foot")
    for leg_name, color_set in zip(["FL", "RR", "RL"], [colors_FL, colors_RR, colors_RL]):
        for i, c in enumerate(color_set):
            h, = ax.plot([], [], [], color=c, lw=3)
            handles.append(h)
            if i == 0:
                labels.append(f"{leg_name} Hip-Knee")
            elif i == 1:
                labels.append(f"{leg_name} Knee-Ankle")
            elif i == 2:
                labels.append(f"{leg_name} Ankle-Foot")
    handles.append(line_FR_traj)
    labels.append("FR Foot Trajectory")
    handles.append(com_marker)
    labels.append("CoM Projection")
    # Create a dummy line for the stability triangle legend
    dummy_tri = ax.plot([], [], [], color='k', lw=2, alpha=0.3)[0]
    handles.append(dummy_tri)
    labels.append("Stability Triangle")
    ax.legend(handles=handles, labels=labels, loc='upper right', fontsize=9)
    
    allX, allY, allZ = [], [], []
    for pos in joint_positions_FR:
        for pt in pos:
            allX.append(pt[0])
            allY.append(pt[1])
            allZ.append(pt[2])
    for pos in [static_positions_FL, static_positions_RR, static_positions_RL]:
        for pt in pos:
            allX.append(pt[0])
            allY.append(pt[1])
            allZ.append(pt[2])
    allX.extend([lower_box[0], lower_box[0]+tw, -0.35, 0.35])
    allY.extend([lower_box[1], lower_box[1]+td, -0.2, 0.2])
    allZ.extend([lower_box[2], lower_box[2]+th, 0.0, 0.15])
    set_axes_equal_3d(ax, np.array(allX), np.array(allY), np.array(allZ))
    
    zmin, zmax = ax.get_zlim3d()
    if zmin < 0:
        ax.set_zlim3d(0, zmax)
    
    def update(frame):
        hip, knee, ankle, foot = joint_positions_FR[frame]
        seg_FR = [(hip, knee), (knee, ankle), (ankle, foot)]
        for i, (p_start, p_end) in enumerate(seg_FR):
            xs = [p_start[0], p_end[0]]
            ys = [p_start[1], p_end[1]]
            zs = [p_start[2], p_end[2]]
            lines_FR[i].set_data(xs, ys)
            lines_FR[i].set_3d_properties(zs)
        xs_traj = foot_traj_FR[:frame+1, 0]
        ys_traj = foot_traj_FR[:frame+1, 1]
        zs_traj = foot_traj_FR[:frame+1, 2]
        line_FR_traj.set_data(xs_traj, ys_traj)
        line_FR_traj.set_3d_properties(zs_traj)
        def update_static(lines, pos):
            hip_s, knee_s, ankle_s, foot_s = pos
            seg = [(hip_s, knee_s), (knee_s, ankle_s), (ankle_s, foot_s)]
            for i, (p_start, p_end) in enumerate(seg):
                xs = [p_start[0], p_end[0]]
                ys = [p_start[1], p_end[1]]
                zs = [p_start[2], p_end[2]]
                lines[i].set_data(xs, ys)
                lines[i].set_3d_properties(zs)
        update_static(lines_FL, static_positions_FL)
        update_static(lines_RR, static_positions_RR)
        update_static(lines_RL, static_positions_RL)
        
        # Calculate CoM dynamics
        leg_positions = {
            "FR": joint_positions_FR[frame],
            "FL": static_positions_FL,
            "RR": static_positions_RR,
            "RL": static_positions_RL
        }
        com = compute_robot_com(trunk_center, leg_positions, link_masses)
        com_proj = np.array([com[0], com[1], board_top_z])
        com_marker.set_data([com_proj[0]], [com_proj[1]])
        com_marker.set_3d_properties([com_proj[2]])
        return lines_FR + lines_FL + lines_RR + lines_RL + [line_FR_traj, com_marker]
    # Disable blit for compatibility with Poly3DCollection
    anim = FuncAnimation(fig, update, frames=len(joint_positions_FR), interval=5, blit=False)  
    plt.show()

# -------------------------------
# Animation for com_transfering mode (trunk shifting with fixed feet)
# -------------------------------

def animate_com_transfering(trunk_traj, leg_trajs, com_traj, board_top_z, trunk_dims):
    # leg_trajs - dictionary with keys "FR", "FL", "RR", "RL": for each leg an array of positions (hip,knee,ankle,foot) by frames
    # trunk_traj - array of trunk positions by frames
    fig = plt.figure(figsize=(12,10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("3D Animation: Trunk Movement (CoM Shifting) with Fixed Feet")
    
    draw_skateboard(ax)
    tw, td, th = trunk_dims
    # Initial trunk position
    lower_box = trunk_traj[0] - np.array([tw/2, td/2, th/2])
    trunk_box = Poly3DCollection([], linewidths=1, edgecolors='green', alpha=0.3)
    trunk_box.set_facecolor((0,1,0,0.3))
    ax.add_collection3d(trunk_box)
    
    # Lines for legs for each color set (same for all legs, but drawn separately)
    colors = {
        "FR": ['skyblue', 'blue', 'navy'],
        "FL": ['lightcoral', 'red', 'darkred'],
        "RR": ['lightgreen', 'green', 'darkgreen'],
        "RL": ['plum', 'purple', 'indigo']
    }
    lines = {leg: [ax.plot([], [], [], color=c, lw=3)[0] for c in colors[leg]] for leg in colors}
    
    # CoM Projection
    com_marker, = ax.plot([], [], [], 'o', color='magenta', markersize=8, label='CoM Projection')
    # Stability polygon (e.g., for legs FL, RR, RL)
    stability_polygon = Poly3DCollection([], facecolor=(0, 0.8, 0, 0.3),
                                         edgecolor='k', lw=2)
    ax.add_collection3d(stability_polygon)
    
    # Legend
    handles = []
    labels = []
    for leg in ["FR", "FL", "RR", "RL"]:
        for i, c in enumerate(colors[leg]):
            h, = ax.plot([], [], [], color=c, lw=3)
            handles.append(h)
            if i == 0:
                labels.append(f"{leg} Hip-Knee")
            elif i == 1:
                labels.append(f"{leg} Knee-Ankle")
            elif i == 2:
                labels.append(f"{leg} Ankle-Foot")
    handles.append(com_marker)
    labels.append("CoM Projection")
    # Create a dummy line for the stability polygon legend
    dummy_poly = ax.plot([], [], [], color='k', lw=2, alpha=0.3)[0]
    handles.append(dummy_poly)
    labels.append("Stability Polygon")
    ax.legend(handles=handles, labels=labels, loc='upper right', fontsize=9)
    
    allX, allY, allZ = [], [], []

    allX.extend([lower_box[0], lower_box[0]+tw, -0.35, 0.35])
    allY.extend([lower_box[1], lower_box[1]+td, -0.2, 0.2])
    allZ.extend([lower_box[2], lower_box[2]+th, 0.0, 0.15])
    set_axes_equal_3d(ax, np.array(allX), np.array(allY), np.array(allZ))
    
    zmin, zmax = ax.get_zlim3d()
    if zmin < 0:
        ax.set_zlim3d(0, zmax)
    
    def update(frame):
        # Trunk
        trunk_pos = trunk_traj[frame]
        lower_box = trunk_pos - np.array([tw/2, td/2, th/2])
        vertices = np.array([
            [lower_box[0],       lower_box[1],       lower_box[2]],
            [lower_box[0]+tw,    lower_box[1],       lower_box[2]],
            [lower_box[0]+tw,    lower_box[1]+td,    lower_box[2]],
            [lower_box[0],       lower_box[1]+td,    lower_box[2]],
            [lower_box[0],       lower_box[1],       lower_box[2]+th],
            [lower_box[0]+tw,    lower_box[1],       lower_box[2]+th],
            [lower_box[0]+tw,    lower_box[1]+td,    lower_box[2]+th],
            [lower_box[0],       lower_box[1]+td,    lower_box[2]+th]
        ])
        faces = [
            [vertices[j] for j in [0,1,2,3]],
            [vertices[j] for j in [4,5,6,7]],
            [vertices[j] for j in [0,1,5,4]],
            [vertices[j] for j in [1,2,6,5]],
            [vertices[j] for j in [2,3,7,6]],
            [vertices[j] for j in [3,0,4,7]]
        ]
        trunk_box.set_verts(faces)
        
        # Legs
        for leg in leg_trajs:
            hip, knee, ankle, foot = leg_trajs[leg][frame]
            seg = [(hip,knee), (knee,ankle), (ankle,foot)]
            for i,(p1,p2) in enumerate(seg):
                lines[leg][i].set_data([p1[0],p2[0]],[p1[1],p2[1]])
                lines[leg][i].set_3d_properties([p1[2], p2[2]])
        
        # Polygon (four feet)
        FR_foot = leg_trajs["FR"][frame][3]
        FL_foot = leg_trajs["FL"][frame][3]
        RL_foot = leg_trajs["RL"][frame][3]
        RR_foot = leg_trajs["RR"][frame][3]
        quad_vertices = np.array([
            [FR_foot[0], FR_foot[1], board_top_z],
            [FL_foot[0], FL_foot[1], board_top_z],
            [RL_foot[0], RL_foot[1], board_top_z],
            [RR_foot[0], RR_foot[1], board_top_z],
        ])
        stability_polygon.set_verts([quad_vertices])
        
        # CoM
        com = com_traj[frame]
        com_proj = np.array([com[0], com[1], board_top_z])
        com_marker.set_data([com_proj[0]], [com_proj[1]])
        com_marker.set_3d_properties([com_proj[2]])
        return sum(list(lines.values()), []) + [trunk_box, com_marker, stability_polygon]
    
    # Disable blit for compatibility with Poly3DCollection
    anim = FuncAnimation(fig, update, frames=len(trunk_traj), interval=50, blit=False)
    plt.show()

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

    # the hip_offset_FL is offset from global(center now) to the hip joint
    # the hip_fixed_offset_FL is offset from the hip joint to the thigh joint (on y axis)

    effective_hip_offset_FR = trunk_center + (robot.hip_offset_FR + robot.hip_fixed_offset_FR)
    effective_hip_offset_FL = trunk_center + (robot.hip_offset_FL + robot.hip_fixed_offset_FL)
    # effective_hip_offset_RR = trunk_center + (robot.hip_offset_RR + robot.hip_fixed_offset_RR)
    print(robot.hip_fixed_offset_RR.shape)
    effective_hip_offset_RR = trunk_center + (robot.hip_offset_RR + np.zeros(3,))
    effective_hip_offset_RL = trunk_center + (robot.hip_offset_RL + robot.hip_fixed_offset_RL)
    print("-----------------------the robot.hip_offset_FL is ---------------",robot.hip_offset_FL)
    print("-----------------------the robot.hip_fixed_offset_FL is ---------------",robot.hip_fixed_offset_FL)
    # -----------------------the robot.hip_offset_FL is --------------- [0.183 0.047 0.   ]
    # -----------------------the robot.hip_fixed_offset_FL is --------------- [0.    0.081 0.   ]

    # Configuration for static legs (FL, RR, RL) - calculated as in previous code
    q_static = q0_all.copy()
    def f_static(q2):
        p_foot_z = effective_hip_offset_FL[2] - (
            robot.L2 * np.sin(q_static[1]) +
            robot.L3 * np.sin(q_static[1] + q2)
        )
        return p_foot_z - board_top_z
    
    # recalculate the joint of leg to make the foot on the board
    q_static[2] = opt.brentq(f_static, -2.7, -0.92)
    def fk_numeric(q):
        p = forward_kinematics(q, robot, side="FR", effective_hip_offset_FR, config["fix_hip"])
        return np.array(ca.DM(p)).flatten()
    p0_abs_static = fk_numeric(q_static)
    
    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the board
    d_x = p0_abs_static[0] - effective_hip_offset_FR[0]
    d_y = 0  # desired - foot should be directly under hip
    d_z = board_top_z - effective_hip_offset_FR[2]
    q0_FR_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot)
    print("Initial FR configuration (for MPC):", q0_FR_on_board)

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the floor
    distance_from_path_to_center = - 0.14

    # this is relative position of the foot to the tip of the leg (at hip joint)
    d_x = p0_abs_static[0] - effective_hip_offset_FR[0]
    d_y = distance_from_path_to_center
    d_z = 0.0 - effective_hip_offset_FR[2]
    q0_FR_on_floor = inverse_kinematics(np.array([d_x, d_y, d_z]), robot)
    print("Initial FR configuration (on floor):", q0_FR_on_floor)
    
    mpc = MPCController(robot, config, q0_FR_on_board, q0_FR_on_floor)

    # # for lifting phase
    mpc.setup_problem_lifting(effective_hip_offset_FR)
    q_sol_FR_lifing, q_sol_FR_lifing_vel = mpc.solve_lifting()


    # for pushing phase
    mpc.setup_problem_pushing(effective_hip_offset_FR, distance_from_path_to_center + effective_hip_offset_FR[1])
    q_sol_FR_pushing, q_sol_FR_pushing_vel = mpc.solve_pushing()
    
    # trajectory consists of 1 lifting and 4 pushing 
    q_sol_FR = np.concatenate((q_sol_FR_lifing, q_sol_FR_pushing, q_sol_FR_pushing, q_sol_FR_pushing), axis=0)

    # Convert numpy array to list
    # for position
    pushing_list = q_sol_FR_pushing.tolist()
    lifting_list = q_sol_FR_lifing.tolist()
    # for velocity
    pushing_vel_list = q_sol_FR_pushing_vel.tolist()
    lifting_vel_list = q_sol_FR_lifing_vel.tolist()



    # Save to JSON file
    with open("pushing.json", "w") as json_file:
        json.dump(pushing_list, json_file)
    with open("lifting.json", "w") as json_file:
        json.dump(lifting_list, json_file)
    with open("pushing_vel.json", "w") as json_file:
        json.dump(pushing_vel_list, json_file)
    with open("lifting_vel.json", "w") as json_file:
        json.dump(lifting_vel_list, json_file)
        
    print("Pushing and lifting trajectories saved to JSON files.")

    joint_positions_FR = []
    foot_traj_FR = []
    for q in q_sol_FR:
        # real position in 3D coordinates, not angle = [hip(of 3 legs), knee, angle, foot]
        pos = compute_leg_positions(q, effective_hip_offset_FR, robot, config["fix_hip"])
        # print("coordinate (x,y,z) of  hip, knee, ankle, foot  ---------------->", pos)
        joint_positions_FR.append(pos)
        foot_traj_FR.append(pos[3])
    joint_positions_FR = np.array(joint_positions_FR)
    foot_traj_FR = np.array(foot_traj_FR)
    
    print("Joint positions for FR leg:", joint_positions_FR.shape)
    static_positions_FL = compute_leg_positions(q_static, effective_hip_offset_FL, robot, fix_hip=True)
    static_positions_RR = compute_leg_positions(q_static, effective_hip_offset_RR, robot, fix_hip=True)
    static_positions_RL = compute_leg_positions(q_static, effective_hip_offset_RL, robot, fix_hip=True)

    print("Link lengths:")
    for leg, pos in zip(["FR", "FL", "RR", "RL"], [joint_positions_FR[0], static_positions_FL, static_positions_RR, static_positions_RL]):
        lk = np.linalg.norm(pos[1] - pos[0])
        la = np.linalg.norm(pos[2] - pos[1])
        lf = np.linalg.norm(pos[3] - pos[2])
        print(f"{leg}: Hip-Knee: {lk:.3f}, Knee-Ankle: {la:.3f}, Ankle-Foot: {lf:.3f}")

    animate_mpc(joint_positions_FR, foot_traj_FR, robot.trunk_dims, trunk_center,
                static_positions_FL, static_positions_RR, static_positions_RL, board_top_z, robot.link_masses)

    return q_sol_FR
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
            print("the shape of pos: ------------------------------------------------", pos.shape)
            current_leg_positions[leg] = pos
        for leg in current_leg_positions:
            leg_trajs[leg].append(current_leg_positions[leg])

        c = compute_robot_com(new_trunk, current_leg_positions, robot.link_masses)
        com_traj.append(c)

    for leg in leg_trajs:
        leg_trajs[leg] = np.array(leg_trajs[leg])
    com_traj = np.array(com_traj)

    animate_com_transfering(trunk_traj, leg_trajs, com_traj, board_top_z, robot.trunk_dims)

# -------------------------------
# Main program
# -------------------------------
def main():
    # If a command line argument is specified, choose mode:
    # "mpc_pushing" - original MPC functionality,
    # "com_transfering" - trunk movement (COM transferring) with fixed feet.
    mode = "mpc_pushing"
    if len(sys.argv) > 1:
        mode = sys.argv[1]
    if mode == "com_transfering":
        run_com_transfering()
    else:
        joint_angle = run_mpc_pushing()
        # print("the joint_angle is -------------------------", joint_angle.shape)


if __name__ == "__main__":
    main()