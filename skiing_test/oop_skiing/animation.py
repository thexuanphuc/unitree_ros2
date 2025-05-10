import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from kinematics import compute_robot_com
import numpy as np

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
def draw_box(ax, lower_corner, width, depth, height, edgecolor='green', facecolor=(0, 0, 0, 0), alpha=0.25):
    x, y, z = lower_corner
    vertices = np.array([
        [x, y, z],
        [x + width, y, z],
        [x + width, y + depth, z],
        [x, y + depth, z],
        [x, y, z + height],
        [x + width, y, z + height],
        [x + width, y + depth, z + height],
        [x, y + depth, z + height]
    ])
    faces = [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]],
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [2, 3, 7, 6]],
        [vertices[j] for j in [3, 0, 4, 7]]
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
    wheel_radius = wheel_diameter / 2
    z_bottom_board = wheel_radius + wheel_gap
    lower_corner = np.array([-board_length / 2, -board_width / 2, z_bottom_board])
    # Skateboard is transparent (alpha=0.3)
    draw_box(ax, lower_corner, board_length, board_width, board_thickness,
             edgecolor='black', facecolor=(0.5, 0.3, 0.1, 0.3), alpha=0.3)
    x_wheels = [0.30, 0.30, -0.30, -0.30]
    y_wheels = [0.08, -0.08, 0.08, -0.08]
    for (xw, yw) in zip(x_wheels, y_wheels):
        center = (xw, yw, wheel_radius)
        theta = np.linspace(0, 2 * np.pi, 24)
        xs = center[0] + wheel_radius * np.cos(theta)
        zs = center[2] + wheel_radius * np.sin(theta)
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
    mid_x = 0.5 * (x_min + x_max)
    mid_y = 0.5 * (y_min + y_max)
    ax.set_xlim3d(mid_x - max_range / 2, mid_x + max_range / 2)
    ax.set_ylim3d(mid_y - max_range / 2, mid_y + max_range / 2)
    ax.set_zlim3d(z_min_new, z_min_new + max_range)

# -------------------------------
# Animation for MPC mode (mpc_pushing)
# -------------------------------
def animate_mpc(joint_positions_FR, foot_traj_FR, trunk_dims, trunk_center,
                static_positions_FL, static_positions_RR, static_positions_RL, board_top_z, link_masses):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("3D Animation: MPC Leg Control and CoM Dynamics")
    
    draw_skateboard(ax)
    tw, td, th = trunk_dims
    lower_box = trunk_center - np.array([tw / 2, td / 2, th / 2])
    draw_box(ax, lower_box, tw, td, th, edgecolor='green', facecolor=(0, 1, 0, 0.3))
    
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
    allX.extend([lower_box[0], lower_box[0] + tw, -0.35, 0.35])
    allY.extend([lower_box[1], lower_box[1] + td, -0.2, 0.2])
    allZ.extend([lower_box[2], lower_box[2] + th, 0.0, 0.15])
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
        xs_traj = foot_traj_FR[:frame + 1, 0]
        ys_traj = foot_traj_FR[:frame + 1, 1]
        zs_traj = foot_traj_FR[:frame + 1, 2]
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
    anim = FuncAnimation(fig, update, frames=len(joint_positions_FR), interval=1, blit=False)
    plt.show()

# -------------------------------
# Animation for com_transfering mode (trunk shifting with fixed feet)
# -------------------------------
def animate_com_transfering(trunk_traj, leg_trajs, com_traj, board_top_z, trunk_dims):
    # leg_trajs - dictionary with keys "FR", "FL", "RR", "RL": for each leg an array of positions (hip,knee,ankle,foot) by frames
    # trunk_traj - array of trunk positions by frames
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("3D Animation: Trunk Movement (CoM Shifting) with Fixed Feet")
    
    draw_skateboard(ax)
    tw, td, th = trunk_dims
    # Initial trunk position
    lower_box = trunk_traj[0] - np.array([tw / 2, td / 2, th / 2])
    trunk_box = Poly3DCollection([], linewidths=1, edgecolors='green', alpha=0.3)
    trunk_box.set_facecolor((0, 1, 0, 0.3))
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

    allX.extend([lower_box[0], lower_box[0] + tw, -0.35, 0.35])
    allY.extend([lower_box[1], lower_box[1] + td, -0.2, 0.2])
    allZ.extend([lower_box[2], lower_box[2] + th, 0.0, 0.15])
    set_axes_equal_3d(ax, np.array(allX), np.array(allY), np.array(allZ))
    
    zmin, zmax = ax.get_zlim3d()
    if zmin < 0:
        ax.set_zlim3d(0, zmax)
    
    def update(frame):
        # Trunk
        trunk_pos = trunk_traj[frame]
        lower_box = trunk_pos - np.array([tw / 2, td / 2, th / 2])
        vertices = np.array([
            [lower_box[0], lower_box[1], lower_box[2]],
            [lower_box[0] + tw, lower_box[1], lower_box[2]],
            [lower_box[0] + tw, lower_box[1] + td, lower_box[2]],
            [lower_box[0], lower_box[1] + td, lower_box[2]],
            [lower_box[0], lower_box[1], lower_box[2] + th],
            [lower_box[0] + tw, lower_box[1], lower_box[2] + th],
            [lower_box[0] + tw, lower_box[1] + td, lower_box[2] + th],
            [lower_box[0], lower_box[1] + td, lower_box[2] + th]
        ])
        faces = [
            [vertices[j] for j in [0, 1, 2, 3]],
            [vertices[j] for j in [4, 5, 6, 7]],
            [vertices[j] for j in [0, 1, 5, 4]],
            [vertices[j] for j in [1, 2, 6, 5]],
            [vertices[j] for j in [2, 3, 7, 6]],
            [vertices[j] for j in [3, 0, 4, 7]]
        ]
        trunk_box.set_verts(faces)
        
        # Legs
        for leg in leg_trajs:
            hip, knee, ankle, foot = leg_trajs[leg][frame]
            seg = [(hip, knee), (knee, ankle), (ankle, foot)]
            for i, (p1, p2) in enumerate(seg):
                lines[leg][i].set_data([p1[0], p2[0]], [p1[1], p2[1]])
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