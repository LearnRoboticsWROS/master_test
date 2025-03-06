from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config_package = "cobot_camera_moveit_config"
    robot_description_package = "six_arm"

    robot_description_file = os.path.join(
        get_package_share_directory(robot_description_package), "urdf", "cobot_camera.urdf.xacro"
    )
    ros2_controllers_file = os.path.join(
        get_package_share_directory(robot_description_package), "config", "trajectory_controllers.yaml"
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(robot_description_package), "config", "config.rviz"
    )

    # Parametro robot_description
    robot_description = Command(["xacro ", robot_description_file])

    # Nodo controller_manager per ROS2 Control
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, ros2_controllers_file],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "custom_robot.srdf",
            )},
            {"use_sim_time": False},
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Spawner Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner Arm Trajectory Controller
    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner Gripper Controller
    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Event Handlers per l'ordine di lancio dei controller
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_trajectory_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[gripper_action_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[rviz_node],
        )
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_publisher_node,
        move_group_node,
        delay_joint_state_broadcaster,
        delay_arm_trajectory_controller,
        delay_gripper_controller,
        delay_rviz_node,
    ])
