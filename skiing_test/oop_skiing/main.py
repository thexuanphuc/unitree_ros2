import sys
import os
import json
import numpy as np
import casadi as ca
import scipy.optimize as opt
from robot_model import RobotModel, default_config, hip_target
from kinematics import forward_kinematics, inverse_kinematics, compute_leg_positions, compute_robot_com, moving_body
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

    ####################### status 0: normal sitting on the board##########################
    # For FR leg, solve inverse kinematics analytically to find the initial configuration that the left foot is on the board
    leg_positions = {
        "FL": (config_skate_board["width"] / 2 - 0.015, robot.global_hip_offset_FL, robot.hip_fixed_offset_FL, board_top_z),
        "FR": (-config_skate_board["width"] / 2 + 0.015, robot.global_hip_offset_FR, robot.hip_fixed_offset_FR, board_top_z),
        "RL": (config_skate_board["width_rear"] / 2 - 0.015, robot.global_hip_offset_RL, robot.hip_fixed_offset_RL, board_top_z),
        "RR": (-config_skate_board["width_rear"] / 2 + 0.015, robot.global_hip_offset_RR, robot.hip_fixed_offset_RR, board_top_z),
    }

    onboard_sitting = []
    for side, (dy, global_offset, fixed_offset, dz) in leg_positions.items():
        d_x = (global_offset + fixed_offset)[0]
        d_y = dy
        d_z = dz
        q0 = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side=side)
        onboard_sitting.append(q0)

    onboard_sitting = np.array(onboard_sitting)

    ####################### status 1: move the body to the right ##########################
    new_trunk_center = robot.trunk_center.copy()
    new_trunk_center[1] -= config["com_shifting"]
    print("the  trunk center is before moving_body  ------------------->", robot.trunk_center)
    trajectory_movebody_1 = moving_body(robot=robot, final_trunk_center=new_trunk_center, q_initial=onboard_sitting, N=100, fix_hip=config["fix_hip"])
    print("the  trunk center is after moving_body  ------------------->", robot.trunk_center)
    ####################### status 2: move one front-left leg into center ##########################
    
    # MPC on kinematics model to find the foot trajectory
    mpc = MPCController(config)

    # TODO this foot position should be the center of the robot, maybe problem that the trucnk was moved outside, but not moved inside when
    # center_foot = np.array([(robot.global_hip_offset_FL+ robot.hip_fixed_offset_FL)[0], robot.trunk_center[1], board_top_z])
    # lower the foot position a little bit to avoid the collision with the board
    center_foot = np.array([(robot.global_hip_offset_FL+ robot.hip_fixed_offset_FL)[0], 0.018, board_top_z])

    q0_center_foot = inverse_kinematics(center_foot, robot, side="FL")
    q_current_FL = trajectory_movebody_1[-1,:3].copy()

    # debug
    print("the initial position of the foot is ------------------->", forward_kinematics(q_current_FL, robot, side="FL", fix_hip=config["fix_hip"]))
    print("the target position of the foot is ------------------->", center_foot)
    trajectory_moveleg, trajectory_moveleg_vel = mpc.setup_problem_simple_moving_leg(cur_robot=robot, side="FL", initial_pose=q_current_FL, final_pose=q0_center_foot, N_steps=100, min_height=board_top_z+0.04)
    print("the center of trunk is ------------------->", robot.trunk_center)

    ####################### status 3: move the body back to center ##########################

    # the front left leg configuration was changed, because it was moved to the center
    q_initial_new = trajectory_movebody_1[-1,:].copy().reshape(4, 3)
    q_initial_new[0,:] = trajectory_moveleg[-1,:].copy().reshape(1, 3)
    
    new_trunk_center = robot.trunk_center.copy()
    new_trunk_center[1] += config["com_shifting"]

    trajectory_movebody_2 = moving_body(robot=robot, final_trunk_center=new_trunk_center, q_initial=q_initial_new, N=100, fix_hip=config["fix_hip"])

    ####################### status 4: move the right leg from board to the floor ##########################
    distance_from_path_to_center = -config_skate_board["distance_from_edge"] +  (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[1]
    print("the distance from path to center is ------------------->", distance_from_path_to_center)
    # find the foot position when FR leg is on the floor in flobal frame
    d_x = (robot.global_hip_offset_FR + robot.hip_fixed_offset_FR)[0]
    d_y = distance_from_path_to_center
    d_z = 0.0
    q0_FR_on_floor = inverse_kinematics(np.array([d_x, d_y, d_z]), robot, side="FR")
    print("Initial FR configuration (on floor):", q0_FR_on_floor)

    #current configuration of the FR leg
    q_current_FR = trajectory_movebody_2[-1,3:6].copy()
    # for lifting phase
    trajectory_FR_down, trajectory_FR_down_vel = mpc.setup_problem_lifting(cur_robot = robot, side="FR", q0_FR_on_board=q_current_FR, q0_FR_on_floor=q0_FR_on_floor)
    
    ####################### status 5: push the front right leg on the floor ##########################

    # for pushing phase
    trajectory_FR_pushing, trajectory_FR_pushing_vel = mpc.setup_problem_pushing(cur_robot = robot, side="FR", path_distance=distance_from_path_to_center, q0_FR_on_floor=q0_FR_on_floor)

    ####################### status 6: move the front right leg back to the skateboard (because the velocity is opposite) ##########################
    # trajectory_FR_up, trajectory_FR_up_vel = mpc.setup_problem_lifting(cur_robot = robot, side="FR", q0_FR_on_board=q0_FR_on_floor, q0_FR_on_floor=q_current_FR)

    trajectory_FR_up = trajectory_FR_down[::-1]
    trajectory_FR_up_vel = -1 * trajectory_FR_down_vel[::-1]



    # Save trajectories to the target directory
    trajectories = {
        "onboard_sitting.json": onboard_sitting.reshape(-1),
        "trajectory_movebody_1.json": trajectory_movebody_1,
        "trajectory_moveleg.json": trajectory_moveleg,
        "trajectory_moveleg_vel.json": trajectory_moveleg_vel,
        "trajectory_movebody_2.json": trajectory_movebody_2,
        "lifting_down.json": trajectory_FR_down,
        "lifting_down_vel.json": trajectory_FR_down_vel,
        "pushing.json": trajectory_FR_pushing,
        "pushing_vel.json": trajectory_FR_pushing_vel,
        "lifting_up.json": trajectory_FR_up,
        "lifting_up_vel.json": trajectory_FR_up_vel,
    }

    for filename, data in trajectories.items():
        with open(os.path.join(target_dir, filename), 'w') as json_file:
            json.dump(data.squeeze().tolist(), json_file)

    print("Trajectories saved to:", target_dir)

    
    ###################### for visualization ##########################
    # trajectory consists of 1 down and 1 pushing and 1 lifting
    q_sol_FR = np.concatenate((trajectory_FR_down, trajectory_FR_pushing, trajectory_FR_up), axis=0)

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


    left_foot = []
    for q in trajectory_moveleg:
        # real position in 3D coordinates, not angle = [hip(of 3 legs), knee, angle, foot]
        pos = compute_leg_positions(q, robot, side="FL", fix_hip=config["fix_hip"])
        left_foot.append(pos[3])
    left_foot = np.array(left_foot)

    static_positions_FL = compute_leg_positions(trajectory_moveleg[-1,:], robot, side="FL", fix_hip=config["fix_hip"])
    static_positions_RL = compute_leg_positions(onboard_sitting[2,:], robot, side="RL", fix_hip=config["fix_hip"])
    static_positions_RR = compute_leg_positions(onboard_sitting[3,:], robot, side="RR", fix_hip=config["fix_hip"])

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