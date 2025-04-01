# # import os

# # from ament_index_python.packages import get_package_share_directory
# # from launch import LaunchDescription
# # from launch_ros.actions import Node
# # from launch.substitutions import Command
# # from launch.actions import IncludeLaunchDescription
# # from launch.launch_description_sources import PythonLaunchDescriptionSource


# # def generate_launch_description():
# #     a1_description_path = os.path.join(
# #         get_package_share_directory('a1_description'))
# #     xacro_file = os.path.join(a1_description_path, 'xacro', 'robot.xacro')
# #     params = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true DEBUG:=false']), 'use_sim_time': True}

# #     # params = {'robot_description': robot_desc}

# #     # this is for publishing the robot state ton ros2
# #     rsp = Node(package='robot_state_publisher',
# #                executable='robot_state_publisher',
# #                output='both',
# #                parameters=[params])


# #     # this is for sliders to control the joints
# #     jsp = Node(
# #         package='joint_state_publisher_gui',
# #         executable='joint_state_publisher_gui',
# #         output='screen',
# #     )

# #     # # this is for rviz launching
# #     # rviz_config_file = get_package_share_directory(
# #     #     'a1_description') + '/launch/a1.rviz'
# #     # rviz_node = Node(package='rviz2',
# #     #                  executable='rviz2',
# #     #                  name='rviz2',
# #     #                  output='log',
# #     #                  arguments=['-d', rviz_config_file])

# #     #run gazebo
# #     gazebo = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros') + '/launch/gazebo.launch.py']),
# #         # launch_arguments={'world': get_package_share_directory('a1_gazebo') + '/worlds/a1_empty.world'}.items()
# #     )

# #     # spawn the robot in gazebo
# #     spawn_entity = Node(
# #         package='gazebo_ros',
# #         executable='spawn_entity.py',
# #         arguments=['-entity', 'a1_phuc', '-topic', 'robot_description'],
# #         output='screen',
# #     )

# #     # load controllers
# #     load_controllers = [
# #         ExecuteProcess(
# #             cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
# #             output='screen'
# #         )
# #         for controller in [
# #             'joint_state_broadcaster',
# #             'a1_joint_trajectory_controller'
# #         ]
# #     ]


# #     # return LaunchDescription([rsp, jsp, rviz_node])
# #     return LaunchDescription([rsp, jsp, gazebo, spawn_entity] + load_controllers)

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import Command
# from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     a1_description_path = os.path.join(
#         get_package_share_directory('a1_description'))
#     xacro_file = os.path.join(a1_description_path, 'xacro', 'robot.xacro')
#     params = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true DEBUG:=false']), 'use_sim_time': True}

#     # this is for publishing the robot state to ROS 2
#     rsp = Node(package='robot_state_publisher',
#                executable='robot_state_publisher',
#                output='both',
#                parameters=[params])

#     # this is for sliders to control the joints
#     jsp = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         output='screen',
#     )

#     # run gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros') + '/launch/gazebo.launch.py']),
#     )

#     # spawn the robot in gazebo
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'a1_phuc', '-topic', 'robot_description'],
#         output='screen',
#     )

#     # controller manager
#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[params, os.path.join(
#             get_package_share_directory('a1_description'),
#             'config',
#             'robot_control.yaml'
#         )],
#         output='screen'
#     )

#     # log info
#     log_info = LogInfo(msg="Launching controller manager and loading controllers...")

#     # load controllers with a delay to ensure controller_manager is ready
#     load_controllers = [
#         TimerAction(
#             period=5.0,  # delay in seconds
#             actions=[
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
#                     output='screen'
#                 )
#             ]
#         )
#         for controller in [
#             'joint_state_controller',
#             'FL_hip_controller',
#             'FL_thigh_controller',
#             'FL_calf_controller',
#             'FR_hip_controller',
#             'FR_thigh_controller',
#             'FR_calf_controller',
#             'RL_hip_controller',
#             'RL_thigh_controller',
#             'RL_calf_controller',
#             'RR_hip_controller',
#             'RR_thigh_controller',
#             'RR_calf_controller'
#         ]
#     ]

#     return LaunchDescription([rsp, jsp, gazebo, spawn_entity, controller_manager, log_info] + load_controllers)