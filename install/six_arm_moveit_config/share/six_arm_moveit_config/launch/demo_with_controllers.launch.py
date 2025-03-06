import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    # moveit_config = MoveItConfigsBuilder(
    #     "custom_robot", package_name="six_arm_moveit_config"
    # ).to_moveit_configs()


    # Command-line arguments
    # tutorial_arg = DeclareLaunchArgument(
    #     "rviz_tutorial", default_value="False", description="Tutorial flag"
    # )

    # db_arg = DeclareLaunchArgument(
    #     "db", default_value="False", description="Database flag"
    # )

    robot_description_file = os.path.join(
        get_package_share_directory('six_arm_moveit_config'), 'urdf', 'cobot_camera.urdf.xacro'
    )
    robot_description = Command(['xacro ', robot_description_file])

    joint_controllers_file = os.path.join(
        get_package_share_directory('six_arm_moveit_config'), 'config', 'ros2_controllers.yaml'
    )


    # ros2_control_hardware_type = DeclareLaunchArgument(
    #     "ros2_control_hardware_type",
    #     default_value="mock_components",
    #     description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    # )


    moveit_config = (
        MoveItConfigsBuilder("six_arm")
        .robot_description(
            file_path="config/custom_robot.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": LaunchConfiguration(
            #         "ros2_control_hardware_type"
            #     )
            # },
        )
        .robot_description_semantic(file_path="config/custom_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            #pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )



    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    # tutorial_mode = LaunchConfiguration("rviz_tutorial")
    # rviz_base = os.path.join(
    #     get_package_share_directory("six_arm_moveit_config"), "launch"
    # )
    # rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    # rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")
    # rviz_node_tutorial = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_empty_config],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.planning_pipelines,
    #         moveit_config.robot_description_kinematics,
    #     ],
    #     condition=IfCondition(tutorial_mode),
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        # condition=UnlessCondition(tutorial_mode),
    )


    # Static TF
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )


    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("six_arm_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="screen",
    # )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, joint_controllers_file],
        output='screen',
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )




    # Event handler per attivare gli spawner dopo controller_manager
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
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
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    ld.add_action(ros2_control_node)
    ld.add_action(move_group_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz_node)


  
    return ld
