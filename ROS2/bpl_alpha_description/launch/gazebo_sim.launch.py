import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue
from launch_ros.descriptions import ParameterValue

from launch_ros.actions import Node


def generate_launch_description():

    package_name='bpl_alpha_description'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = get_package_share_directory('bpl_alpha_description')
    urdf_path = os.path.join(pkg_share, 'urdf/alpha_5_example.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz/rviz.rviz')

    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_path)]), value_type=str),
        }]
    )

    # Node: RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='RVIZ',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
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
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )
    

    # Node: spawn entity into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-name', 'alpha_5',
                    'z', '0.1',],
        output='screen'
    )

   # Node: joint broadcaster controller
    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broadcaster'],
        output='screen'
    )

    # Node: joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    # Timer to delay spawn_entity to wait for Gazebo
    delayed_spawn_entity = TimerAction(
        period=1.0,
        actions=[spawn_entity]
    )

    # Timer to delay controller spawning
    delayed_controller_joint_broadcaster = TimerAction(
        period=4.0,  # seconds
        actions=[
            joint_broadcaster_spawner,
        ]
    )

    delayed_controller_joint_trajectory = TimerAction(
        period=6.0,  # seconds
        actions=[
            joint_trajectory_controller_spawner,
        ]
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'bridge_params.yaml')
    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[bridge_params],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        robot_state_publisher_node,
        rviz_node,
        world_arg,
        gazebo,
        ros_gz_bridge,
        delayed_spawn_entity,
        delayed_controller_joint_broadcaster,
        delayed_controller_joint_trajectory,
    ])
