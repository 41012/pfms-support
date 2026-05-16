#!/usr/bin/env python3
import os
import subprocess

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    # Package directories
    pfms_dir = get_package_share_directory('pfms')
    sjtu_drone_description_dir = get_package_share_directory('sjtu_drone_description')
    sjtu_drone_bringup_dir = get_package_share_directory('sjtu_drone_bringup')

    # -------------------------------------------------------
    # Gazebo Simulation (Ignition Fortress)
    # -------------------------------------------------------
    world_file = os.path.join(pfms_dir, 'worlds', 'tower_search_and_rescue.world')

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
    # SJTU Drone
    # -------------------------------------------------------
    # Process xacro to get robot description string
    sjtu_drone_xacro_file = os.path.join(sjtu_drone_description_dir, 'urdf', 'sjtu_drone.urdf.xacro')
    sjtu_drone_robot_desc = subprocess.check_output(
        ['xacro', sjtu_drone_xacro_file]).decode('utf-8')

    # Spawn sjtu_drone into Gazebo from the /sjtu_drone/robot_description topic
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

    sjtu_drone_group = GroupAction([
        PushRosNamespace('sjtu_drone'),
        # RSP publishes to /sjtu_drone/robot_description
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
        spawn_sjtu_drone,
        sjtu_drone_bridge,
    ])

    # -------------------------------------------------------
    # Additional nodes
    # -------------------------------------------------------
    drone_reach = Node(
        package='pfms',
        executable='reach',
        name='drone_reach',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/orange/odom', '/sjtu_drone/odom'),
            ('/register_goals', '/sjtu_drone/register_goals'),
            ('/check_goals', '/sjtu_drone/check_goals'),
        ]
    )

    # -------------------------------------------------------
    # RViz2
    # -------------------------------------------------------
    # rviz_config = os.path.join(pfms_dir, 'rviz', 'a3_search.rviz')
    rviz_config = os.path.join(sjtu_drone_bringup_dir, 'rviz', 'sjtu_drone.rviz')
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
        sjtu_drone_group,
        # drone_reach,
        rviz_node,
    ]


def generate_launch_description():
    # Set up model path for Ignition Gazebo
    pfms_dir = get_package_share_directory('pfms')
    models_path = os.path.join(pfms_dir, 'models')
    
    # Get existing IGN_GAZEBO_RESOURCE_PATH and append our models
    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if ign_resource_path:
        models_path = models_path + ':' + ign_resource_path
    
    return launch.LaunchDescription([
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
            default_value='true',
            description='Launch RViz (true) or not (false)'),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()
