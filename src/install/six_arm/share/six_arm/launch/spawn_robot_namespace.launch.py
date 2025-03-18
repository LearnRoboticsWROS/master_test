from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    robot_description_file = os.path.join(
        get_package_share_directory('six_arm'), 'urdf', 'cobot.urdf.xacro'
    )
    joint_controllers_file = os.path.join(
        get_package_share_directory('six_arm'), 'config', 'controllers.yaml'
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    # Parametro robot_description
    robot_description = Command(['xacro ', robot_description_file])

    # Dichiarazioni degli argomenti
    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'paused': 'true',
        }.items()
    )

    # Node per spawnare il robot
    spawn_the_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='cobot',  # Imposta il namespace
        arguments=[
            '-entity', 'cobot',
            '-topic', '/cobot/robot_description',  # Percorso con namespace
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )

    # Node per controller_manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='cobot',
        parameters=[
            {'robot_description': robot_description},  # Descrizione del robot
            joint_controllers_file,  # File dei controller
        ],
        output='screen',
        remappings=[
            ("~/robot_description", "/cobot/robot_description"),  # Aggiorna il remapping con il namespace
        ],
    )

    # Node per robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='cobot',  # Imposta il namespace
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Node per spawner del joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='cobot',  # Imposta il namespace
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/cobot/controller_manager", 
        ],
        output="screen",
    )

    # Node per spawner del controller per le braccia
    arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='cobot',  # Imposta il namespace
        arguments=[
            "arm_position_controller",
            "--controller-manager", "/cobot/controller_manager", 
        ],
        output="screen",
    )

    # Esegui il controllo di avvio dei processi
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_position_controller_spawner],
        )
    )

    # Aggiungi azioni al launch
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gazebo)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_the_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)

    # Aggiungi il comando per ottenere il parametro robot_description
    # Questo comando può essere utile per verificare che il parametro sia effettivamente impostato.
    # ld.add_action(ExecuteProcess(
    #     cmd=['ros2', 'param', 'get', '/cobot/controller_manager', 'robot_description'],
    #     output='screen'
    # ))

    return ld

