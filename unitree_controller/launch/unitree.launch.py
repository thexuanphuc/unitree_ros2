from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    #################################### CONFIG ##################################

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_package",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_config_file",
            description="Relative path from the config package to the YAML file with the controllers configuration .",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            description="Relative path from the description package to the URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="unitree_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            description="Package with the Rviz2\'s configuration folder. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            description="Relative PATH from the rviz config package to the config file of the Rviz2\'"
        )
    )

    # Initialize Arguments
    controllers_config_package = LaunchConfiguration("controllers_config_package")
    controllers_config_file = LaunchConfiguration("controllers_config_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_file]
            ),
            " ",
            "use_gazebo:=",
            'false',
            " ",
            "DEBUG:=",
            'true',
            " ",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    #  fix xacro file
    # a1_description_path = os.path.join(
    #     get_package_share_directory('a1_description'))
    # xacro_file = os.path.join(a1_description_path, 'xacro', 'robot.xacro')
    # robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_gazebo:=true DEBUG:=false'])}


    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(controllers_config_package), controllers_config_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package), rviz_config_file]
    )


    #################################### DEFINE NODE ##################################

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[robot_controller, "-c", "/controller_manager"],
    # )
    #  load the state broadcaster, the name is controller but it was broadcaster only
    
    load_joint_state_broadcaster_aida = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster_aida'],
        output='screen'
    )

    # load the controller for joint
    load_unitree_controller_aida = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'unitree_controller_aida'],
        output='screen'
    )

    #################################### HANDLE TIMING WHILE SPAWNING ##################################

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster_aida,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_joint_state_broadcaster_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster_aida,
            on_exit=[load_unitree_controller_aida],
        )
    )

    #################################### COMBINE AND EXECUTE ##################################

    nodes = [
        control_node,
        robot_state_pub_node,
        load_joint_state_broadcaster_aida,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)