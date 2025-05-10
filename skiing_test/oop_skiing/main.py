import sys
import numpy as np
import json
import casadi as ca
import scipy.optimize as opt
from robot_model import RobotModel, default_config, hip_target
from kinematics import forward_kinematics, inverse_kinematics, compute_leg_positions, compute_robot_com,  forward_kinematics_relative
from mpc_controller import MPCController
from animation import animate_mpc, animate_com_transfering

# -------------------------------
# MPC mode (mpc_pushing)
# -------------------------------
def run_mpc_pushing():
    config = default_config.copy()
    config["fix_hip"] = False  # FR leg with free hip, but for IK MPC we set q0 separately
    distance_between_skate_and_robot = 0.21
    board_top_z = 0.058
    standing_height = board_top_z + distance_between_skate_and_robot
    trunk_center = np.array([0.0, 0.0, standing_height])
    
    # initialize the robot model and its global position
    robot = RobotModel(config)
    robot.set_trunk_center(trunk_center)
    config["standing_height"] = standing_height

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the board
    d_x = (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[0]
    d_y = (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[1]
    d_z = board_top_z
    q0_FR_on_board = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="FR")
    print("Initial FR configuration (for MPC):", q0_FR_on_board)

    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the floor
    
    distance_from_path_to_center = -0.1 +  (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[1]

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

    # Save to JSON file
    with open("pushing.json", "w") as json_file:
        json.dump(pushing_list, json_file)
    with open("lifting.json", "w") as json_file:
        json.dump(lifting_list, json_file)
    with open("pushing_vel.json", "w") as json_file:
        json.dump(pushing_vel_list, json_file)
    with open("lifting_vel.json", "w") as json_file:
        json.dump(lifting_vel_list, json_file)
    with open("q0_FR_on_board.json", "w") as json_file:
        json.dump(q0_FR_on_board.tolist(), json_file)
    print("Pushing and lifting trajectories saved to JSON files.")

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

    print("Joint positions for FR leg:", joint_positions_FR.shape)
    # because the configuration for 4 legs on the board is the same
    
    static_positions_FL = compute_leg_positions(q0_FR_on_board, robot, side="FL", fix_hip=config["fix_hip"])
    static_positions_RR = compute_leg_positions(q0_FR_on_board, robot, side="RR", fix_hip=config["fix_hip"])
    static_positions_RL = compute_leg_positions(q0_FR_on_board, robot, side="RL", fix_hip=config["fix_hip"])

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