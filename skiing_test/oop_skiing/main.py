import sys
import os
import json
import numpy as np
import casadi as ca
import scipy.optimize as opt
from robot_model import RobotModel, default_config, hip_target
from kinematics import forward_kinematics, inverse_kinematics, compute_leg_positions, compute_robot_com,  forward_kinematics_relative
from mpc_controller import MPCController
from animation import animate_mpc, animate_com_transfering
from config_skate_board import config_skate_board   


# -------------------------------
# MPC mode (mpc_pushing)
# -------------------------------
def run_mpc_pushing():
    # Get the target directory path
    current_dir = os.path.dirname(os.path.abspath(__file__))  # Get current script's directory
    target_dir = os.path.join(current_dir, '../../unitree_controller/config')
    # Create directory if it doesn't exist
    os.makedirs(target_dir, exist_ok=True)

    config = default_config.copy()
    config["fix_hip"] = False  # FR leg with free hip, but for IK MPC we set q0 separately
    distance_between_skate_and_robot = 0.21
    board_top_z = config_skate_board["high"]
    standing_height = board_top_z + distance_between_skate_and_robot
    trunk_center = np.array([0.0, 0.0, standing_height])
    
    # initialize the robot model and its global position
    robot = RobotModel(config)
    robot.set_trunk_center(trunk_center)
    config["standing_height"] = standing_height

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the board
    d_x = (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[0]
    d_y = - config_skate_board["width"] / 2 + 0.015
    d_z = board_top_z
    q0_FR_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="FR")
    print("Initial FR configuration (for MPC):", q0_FR_on_board)

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the floor
    
    distance_from_path_to_center = -config_skate_board["distance_from_edge"] +  (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[1]

    # this is relative position of the foot to the tip of the leg (at hip joint)
    d_x = (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[0]
    d_y = distance_from_path_to_center
    d_z = 0.0
    q0_FR_on_floor = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="FR")
    print("Initial FR configuration (on floor):", q0_FR_on_floor)
    
    # MPC on kinematics model to find the foot trajectory
    mpc = MPCController(robot, config, q0_FR_on_board, q0_FR_on_floor)

    # for lifting phase
    mpc.setup_problem_lifting(side="FR")
    q_sol_FR_lifting, q_sol_FR_lifting_vel = mpc.solve_lifting()

    # for pushing phase
    mpc.setup_problem_pushing(side="FR", path_distance=distance_from_path_to_center)
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
    
    joint_positions_FR = []
    foot_traj_FR = []
    for q in q_sol_FR:
        # real position in 3D coordinates, not angle = [hip(of 3 legs), knee, angle, foot]
        pos = compute_leg_positions(q, robot, side="FR", fix_hip=config["fix_hip"])
        # print("coordinate (x,y,z) of  hip, knee, ankle, foot  ---------------->", pos)
        joint_positions_FR.append(pos)
        foot_traj_FR.append(pos[3])
    joint_positions_FR = np.array(joint_positions_FR)
    foot_traj_FR = np.array(foot_traj_FR)

    # find the position of others legs
    # for RR leg, we need to keep the foot more narrow
    d_x = (robot.global_hip_offset_RR + robot.hip_fixed_offset_RR)[0]
    d_y = - config_skate_board["width"] / 2 + 0.03
    d_z = board_top_z
    q0_RR_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="RR")

    # for FL leg
    d_x = (robot.global_hip_offset_FL + robot.hip_fixed_offset_FL)[0]
    d_y = config_skate_board["width"] / 2 - 0.015
    d_z = board_top_z
    q0_FL_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="FL")
    # for RL leg, we need to keep the foot more narrow
    d_x = (robot.global_hip_offset_RL + robot.hip_fixed_offset_RL)[0]
    d_y = config_skate_board["width"] / 2 - 0.03
    d_z = board_top_z
    q0_RL_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="RL")

    onboard_legs = np.concatenate((q0_FL_on_board, q0_FR_on_board,q0_RL_on_board, q0_RR_on_board), axis=0)

    # Save files to the target directory
    file_paths = {
        "pushing.json": pushing_list,
        "lifting.json": lifting_list,
        "pushing_vel.json": pushing_vel_list,
        "lifting_vel.json": lifting_vel_list,
        "onboard_legs.json": onboard_legs.tolist()
    }

    for filename, data in file_paths.items():
        full_path = os.path.join(target_dir, filename)
        with open(full_path, 'w') as json_file:
            json.dump(data, json_file)

    print("Trajectories saved to:", target_dir)

    static_positions_FL = compute_leg_positions(q0_FL_on_board, robot, side="FL", fix_hip=config["fix_hip"])
    static_positions_RR = compute_leg_positions(q0_RR_on_board, robot, side="RR", fix_hip=config["fix_hip"])
    static_positions_RL = compute_leg_positions(q0_RL_on_board, robot, side="RL", fix_hip=config["fix_hip"])

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
        pass
        # run_com_transfering()
    else:
        joint_angle = run_mpc_pushing()
        # print("the joint_angle is -------------------------", joint_angle.shape)

if __name__ == "__main__":
    main()