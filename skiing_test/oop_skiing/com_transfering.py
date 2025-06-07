
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
        q_static[leg] = inverse_kinematics(d0, robot, side=leg)
        foot_static[leg] = forward_kinematics(q_static[leg], robot, side=leg, fix_hip=True).flatten()
        

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
            q_new = inverse_kinematics(d_new, robot, side=leg)

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

