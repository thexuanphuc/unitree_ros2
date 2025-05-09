import sys
import numpy as np
import json
import casadi as ca
import scipy.optimize as opt
from robot_model import RobotModel, default_config, hip_target
from kinematics import forward_kinematics, inverse_kinematics, compute_leg_positions, compute_robot_com
from mpc_controller import MPCController
from animation import animate_mpc, animate_com_transfering

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
        p = forward_kinematics(q, robot, effective_hip_offset_FR, config["fix_hip"])
        return np.array(ca.DM(p)).flatten()
    p0_abs_static = fk_numeric(q_static)
    
    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the board
    d_x = p0_abs_static[0] - effective_hip_offset_FR[0]
    d_y = 0  # desired - foot should be directly under hip
    d_z = board_top_z - effective_hip_offset_FR[2]
    q0_FR_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot)
    print("Initial FR configuration (for MPC):", q0_FR_on_board)

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the floor
    distance_from_path_to_center = -0.14

    # this is relative position of the foot to the tip of the leg (at hip joint)
    d_x = p0_abs_static[0] - effective_hip_offset_FR[0]
    d_y = distance_from_path_to_center
    d_z = 0.0 - effective_hip_offset_FR[2]
    q0_FR_on_floor = inverse_kinematics(np.array([d_x, d_y, d_z]), robot)
    print("Initial FR configuration (on floor):", q0_FR_on_floor)
    
    mpc = MPCController(robot, config, q0_FR_on_board, q0_FR_on_floor)

    # for lifting phase
    mpc.setup_problem_lifting(effective_hip_offset_FR)
    q_sol_FR_lifting, q_sol_FR_lifting_vel = mpc.solve_lifting()

    # for pushing phase
    mpc.setup_problem_pushing(effective_hip_offset_FR, distance_from_path_to_center + effective_hip_offset_FR[1])
    q_sol_FR_pushing, q_sol_FR_pushing_vel = mpc.solve_pushing()
    
    # trajectory consists of 1 lifting and 4 pushing 
    q_sol_FR = np.concatenate((q_sol_FR_lifting, q_sol_FR_pushing, q_sol_FR_pushing, q_sol_FR_pushing), axis=0)

    # Convert numpy array to list
    # for position
    pushing_list = q_sol_FR_pushing.tolist()
    lifting_list = q_sol_FR_lifting.tolist()
    # for velocity
    pushing_vel_list = q_sol_FR_pushing_vel.tolist()
    lifting_vel_list = q_sol_FR_lifting_vel.tolist()

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
            forward_kinematics_relative(q_static[leg], robot.L1, robot.L2, robot.L3, fix_hip=False)
        ).flatten()

    # Create CYCLE: 0 -> +com_shift -> 0 -> -com_shift -> 0
    cycle_points = [0.0, +com_shift, 0.0, -com_shift, 0.0]
    num_frames = config["N"]
    # Evenly distribute frames across these segments
    segment_len = num_frames // (len(cycle_points) - 1)

    trunk_ys = []
    for i in range(len(cycle_points) - 1):
        startY = trunk_center[1] + cycle_points[i]
        endY = trunk_center[1] + cycle_points[i + 1]
        for j in range(segment_len):
            alpha = j / (segment_len - 1)
            y_val = (1 - alpha) * startY + alpha * endY
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
            print("the shape of pos: ------------------------------------------------", np.array(pos).shape)
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