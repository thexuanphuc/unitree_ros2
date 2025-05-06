#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import sys
import os
from ament_index_python.packages import get_package_share_directory

class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__('trajectory_publisher_node')

        # Declare parameters
        self.declare_parameter('trajectory_file', 'joint_trajectory.npy')
        self.declare_parameter('control_rate', 400.0)
        self.declare_parameter('mode', 'mpc_pushing')
        self.declare_parameter('joint_names', [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint'
        ])
        self.declare_parameter('dt', 0.01)  # Time step from script

        # Get parameters
        trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Resolve trajectory file path
        if os.path.isabs(trajectory_file):
            self.trajectory_file_path = trajectory_file
        else:
            pkg_share = get_package_share_directory('unitree_controller')
            self.trajectory_file_path = os.path.join(pkg_share, 'config', trajectory_file)

        # Load trajectory
        self.get_logger().info(f'Loading trajectory from {self.trajectory_file_path}')
        try:
            self.joint_trajectory = np.load(self.trajectory_file_path, allow_pickle=True)
            print(self.joint_trajectory.shape)
            print(self.joint_trajectory.dtype)

        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory: {str(e)}')
            sys.exit(1)

        # Validate trajectory
        if self.mode == 'mpc_pushing':
            # Expect shape (N+1, 3) for FR leg joint angles
            if not isinstance(self.joint_trajectory, np.ndarray) or self.joint_trajectory.shape[1] != 3:
                self.get_logger().error('Invalid trajectory shape for mpc_pushing, expected (N+1, 3)')
                sys.exit(1)
            self.get_logger().info(f'Loaded mpc_pushing trajectory with shape {self.joint_trajectory.shape}')
        else:  # com_transfering
            # Expect dictionary with leg trajectories
            if not isinstance(self.joint_trajectory, dict):
                self.get_logger().error('Invalid trajectory format for com_transfering, expected dict')
                sys.exit(1)
            self.joint_names = [
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
            ]
            self.get_logger().info(f'Loaded com_transfering trajectory for legs: {list(self.joint_trajectory.keys())}')

        # Initialize publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Start publishing
        self.trajectory_index = 0
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.publish_trajectory
        )
        self.get_logger().info('Trajectory Publisher Node started')

    def publish_trajectory(self):
        if self.mode == 'mpc_pushing':
            self.publish_mpc_pushing()
        else:
            self.publish_com_transfering()

    def publish_mpc_pushing(self):
        if self.trajectory_index >= self.joint_trajectory.shape[0]:
            self.trajectory_index = 0  # Loop trajectory

        # Create JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.joint_trajectory[self.trajectory_index].tolist()
        point.time_from_start = Duration(
            sec=int(self.trajectory_index * self.dt),
            nanosec=int((self.trajectory_index * self.dt % 1) * 1e9)
        )

        traj_msg.points = [point]
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().debug(f'Published mpc_pushing trajectory point {self.trajectory_index}')

        self.trajectory_index += 1

    def publish_com_transfering(self):
        if self.trajectory_index >= len(self.joint_trajectory['FR']):
            self.trajectory_index = 0  # Loop trajectory

        # Create JointTrajectory message for all legs
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names

        # Collect joint angles for all legs
        point = JointTrajectoryPoint()
        positions = []
        for leg in ['FR', 'FL', 'RR', 'RL']:
            # Each leg trajectory contains joint angles (hip, thigh, calf)
            # leg_trajs[leg][i] is (4, 3) for (hip, knee, ankle, foot)
            # We need joint angles, which are computed in run_com_transfering
            leg_traj = self.joint_trajectory[leg][self.trajectory_index]
            # Assuming joint angles are stored (not foot positions)
            positions.extend(leg_traj[3].tolist())  # Adjust if joint angles are elsewhere
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.trajectory_index * self.dt),
            nanosec=int((self.trajectory_index * self.dt % 1) * 1e9)
        )

        traj_msg.points = [point]
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().debug(f'Published com_transfering trajectory point {self.trajectory_index}')

        self.trajectory_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()