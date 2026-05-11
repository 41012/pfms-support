#!/usr/bin/env python3
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context):
    # Package directories
    pfms_dir = get_package_share_directory('pfms')
    husky_gazebo_dir = get_package_share_directory('husky_gazebo')
    sjtu_drone_description_dir = get_package_share_directory('sjtu_drone_description')
    sjtu_drone_bringup_dir = get_package_share_directory('sjtu_drone_bringup')

    # -------------------------------------------------------
    # Gazebo Simulation (Ignition Fortress)
    # -------------------------------------------------------
    world_file = os.path.join(pfms_dir, 'worlds', 'terrain_1.world')

    start_paused_str = LaunchConfiguration('start_paused').perform(context)
    gui_str = LaunchConfiguration('gui').perform(context)
    
    # Build Gazebo arguments: add -s flag for headless mode when gui=false
    gz_args = world_file
    if gui_str.lower() != 'true':
        gz_args += ' -s'  # Server only (headless)
    if start_paused_str.lower() != 'true':
        gz_args += ' -r'  # Run on start

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'),
                         'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': gz_args}.items()
    )

    # -------------------------------------------------------
    # Clock Bridge (Global - single clock publisher)
    # -------------------------------------------------------
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        parameters=[{
            'config_file': os.path.join(pfms_dir, 'config', 'clock_bridge.yaml'),
        }],
        output='screen'
    )

    # -------------------------------------------------------
    # Husky
    # -------------------------------------------------------
    # Process xacro to get robot description string
    husky_xacro_file = os.path.join(husky_gazebo_dir, 'urdf', 'husky.urdf.xacro')
    husky_robot_desc = subprocess.check_output(
        ['xacro', husky_xacro_file]).decode('utf-8')

    # Spawn husky into Gazebo from the /husky/robot_description topic
    spawn_husky = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_husky',
        output='screen',
        arguments=['-topic', '/husky/robot_description',
                   '-name', 'husky',
                   '-x', '0', '-y', '0', '-z', '0.4']
    )

    # Bridge topics between Gazebo and ROS2 for the husky
    husky_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='husky_gz_bridge',
        parameters=[{
            'config_file': os.path.join(husky_gazebo_dir, 'config', 'ros_gz_bridge_husky.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ]
    )

    husky_group = GroupAction([
        PushRosNamespace('husky'),
        # RSP publishes to /husky/robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': husky_robot_desc},
                {'frame_prefix': 'husky/'},
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
        Node(
            package='pfms',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[
                {'use_sim_time': True},
                {'frame_prefix': 'husky/'},
                {'parent_frame': 'world'},
                {'child_frame': 'base_link'},
                {'input_mode': 'odom'},
                {'publish_rate_hz': 20.0},
            ],
            remappings=[
                ('odom', '/husky/odom'),
            ],
            output='screen'
        ),
        Node(
            package='pfms',
            executable='reach',
            name='reach',
            parameters=[{'use_sim_time': True}],
            output='screen',
            remappings=[
                ('/odom', '/husky/odom'),
                ('/register_goals', '/husky/register_goals'),
                ('/check_goals', '/husky/check_goals'),
            ]
        ),
        spawn_husky,
        husky_bridge,
    ])

    # -------------------------------------------------------
    # SJTU Drone
    # -------------------------------------------------------
    # Process xacro to get robot description string
    sjtu_drone_xacro_file = os.path.join(sjtu_drone_description_dir, 'urdf', 'sjtu_drone.urdf.xacro')
    sjtu_drone_robot_desc = subprocess.check_output(
        ['xacro', sjtu_drone_xacro_file]).decode('utf-8')

    # Spawn sjtu_drone into Gazebo from the /drone/robot_description topic
    spawn_sjtu_drone = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_sjtu_drone',
        output='screen',
        arguments=['-topic', '/sjtu_drone/robot_description',
                   '-name', 'sjtu_drone',
                   '-x', '0', '-y', '0', '-z', '0.5']
    )

    # Bridge topics between Gazebo and ROS2 for the sjtu_drone
    sjtu_drone_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='sjtu_drone_gz_bridge',
        parameters=[{
            'config_file': os.path.join(sjtu_drone_bringup_dir, 'config', 'ros_ign_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ]
    )

    # SJTU Drone group with namespace
    sjtu_drone_group = GroupAction([
        PushRosNamespace('sjtu_drone'),
        # RSP publishes to /drone/robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': sjtu_drone_robot_desc},
                {'frame_prefix': 'sjtu_drone/'},
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
        Node(
            package='pfms',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[
                {'use_sim_time': True},
                {'frame_prefix': 'sjtu_drone/'},
                {'parent_frame': 'world'},
                {'child_frame': 'base_link'},
                {'input_mode': 'odom'},
                {'input_topic': 'odom'},
                {'publish_rate_hz': 20.0},
            ],
            output='screen'
        ),
        # Node(
        #     package='pfms',
        #     executable='reach',
        #     name='reach',
        #     parameters=[{'use_sim_time': True}],
        #     output='screen',
        #     remappings=[
        #         ('/odom', '/drone/odom'),
        #         ('/register_goals', '/drone/register_goals'),
        #         ('/check_goals', '/drone/check_goals'),
        #     ]
        # ),
        spawn_sjtu_drone,
        sjtu_drone_bridge,
    ])

    # -------------------------------------------------------
    # RViz2
    # -------------------------------------------------------
    # Note: RViz may show TF time jump warnings when launched with the simulation.
    # For best results, launch RViz separately after the simulation starts:
    #   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix pfms)/share/pfms/rviz/a3_terrain.rviz
    rviz_config = os.path.join(pfms_dir, 'rviz', 'a3_terrain.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return [
        gz_sim,
        clock_bridge,
        husky_group,
        # sjtu_drone_group,
        rviz_node,
    ]


def generate_launch_description():
    # Set IGN_GAZEBO_RESOURCE_PATH for Ignition Gazebo to find local models
    pfms_dir = get_package_share_directory('pfms')
    models_path = os.path.join(pfms_dir, 'models')
    
    # Get existing IGN_GAZEBO_RESOURCE_PATH and append our models
    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if ign_resource_path:
        models_path = models_path + ':' + ign_resource_path
    
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=models_path),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Launch Gazebo with GUI (true) or headless mode (false)'),
        DeclareLaunchArgument(
            'start_paused',
            default_value='false',
            description='Start the simulation in a paused state'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz (true) or not (false). Note: For best results, launch RViz separately to avoid TF time jump warnings'),
        OpaqueFunction(function=launch_setup)
    ])
