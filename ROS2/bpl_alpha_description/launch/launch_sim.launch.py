from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('bpl_alpha_description'),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    # )

    pkg_path = get_package_share_directory('bpl_alpha_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'alpha_5.urdf.xacro')

    # Process URDF with sim_mode linked to use_sim_time
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' sim_mode:=', use_sim_time
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_config},
            {'use_sim_time': use_sim_time}
        ]
    )

    default_world = os.path.join(
        get_package_share_directory('bpl_alpha_description'),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    ignition = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 --physics-engine gz-physics-bullet-featherstone-plugin ', world], 
                                      'on_exit_shutdown': 'true',
                                      'use_sim_time': 'true',
                                      }.items()
            )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # spawn the robot
    spawn_the_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'alpha_5',
                                   '-z', '0.5'],
                        output='screen')
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory("bpl_alpha_description"),'config','alpha_5_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )   

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )   

    bridge_params = os.path.join(get_package_share_directory('bpl_alpha_description'),'config','bridge_params.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    delayed_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delayed_arm_cont_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_controller_spawner],
        )
    )

    delayed_gripper_cont_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[gripper_controller_spawner],
        )
    )

    # Launch Description
    #ld.add_action(rsp)
    ld.add_action(declare_use_sim_time)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(world_arg)
    ld.add_action(ignition)
    ld.add_action(spawn_the_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_broadcaster_spawner)
    ld.add_action(delayed_arm_cont_spawner)
    ld.add_action(delayed_gripper_cont_spawner)

    return ld