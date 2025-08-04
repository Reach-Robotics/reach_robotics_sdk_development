from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro
import yaml
from launch.event_handlers import OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()
    moveit_config = (
        MoveItConfigsBuilder("alpha_5", package_name="bpl_alpha_description_moveit")
        .robot_description(file_path="config/alpha_5.urdf.xacro")
        .robot_description_semantic(file_path="config/alpha_5.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
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
    rviz_config_path = os.path.join(
        get_package_share_directory("bpl_alpha_description_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # spawn the robot
    spawn_the_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'alpha_5_example',
                                   '-z', '0.5'],
                        output='screen')
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )
    
    bridge_params = os.path.join(get_package_share_directory('bpl_alpha_description'),'config','bridge_params.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        # arguments=[
        #     '--ros-args',
        #     '-p',
        #     f'config_file:={bridge_params}',
        # ]
        parameters=[
            { "config_file": bridge_params },
        ],
        output="screen",
    )

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"use_sim_time": True})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.trajectory_execution,
                    {"use_sim_time": True},
                    {"start_state_max_bounds_error": 0.1},
                    ],   
        arguments=["--ros-args", "--log-level", "info"],
    )






    # Launch Description
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)
    ld.add_action(world_arg)
    ld.add_action(ignition)
    ld.add_action(spawn_the_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(gripper_controller_spawner)

    return ld