# filepath: /home/phuc/working/a1_sim/unitree_ros2/unitree_hardware/launch/gazebo_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros2')
    gazebo_launch_file_dir = os.path.join(gazebo_ros2, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file_dir, '/gazebo.launch.py']),
        ),
        Node(
            package='unitree_gazebo',
            executable='unitree_gazebo_node',
            name='unitree_gazebo_node',
            output='screen'
        )
    ])