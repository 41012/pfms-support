import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    # Package directories
    pfms_dir = get_package_share_directory('pfms')
    audibot_gazebo_dir = get_package_share_directory('audibot_gazebo')
    audibot_description_dir = get_package_share_directory('audibot_description')

    # -------------------------------------------------------
    # Gazebo Simulation (Ignition)
    # -------------------------------------------------------
    world_file = os.path.join(pfms_dir, 'worlds', 'race_track.world')

    gui_str = LaunchConfiguration('gui').perform(context)
    
    # Build Gazebo arguments: add -s flag for headless mode when gui=false
    gz_args = world_file
    if gui_str.lower() != 'true':
        gz_args += ' -s'  # Server only (headless)
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
    # Orange Audibot
    # -------------------------------------------------------
    orange_sdf_file = os.path.join(
        audibot_description_dir, 'models', 'orange_audibot', 'model.sdf')
    with open(orange_sdf_file, 'r') as f:
        orange_robot_desc = f.read()

    bridge_config_orange = os.path.join(
        audibot_gazebo_dir, 'config', 'ros_gz_bridge_orange.yaml')

    # Spawn the orange audibot into the world at runtime
    spawn_orange_audibot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_orange_audibot',
        arguments=['-file', orange_sdf_file, '-x', '19.0', '-y', '14.8', '-z', '0'],
        output='screen'
    )

    orange_group = GroupAction([
        PushRosNamespace('orange'),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='orange_bridge',
            parameters=[{
                'config_file': bridge_config_orange,
                'use_sim_time': True,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen',
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': orange_robot_desc},
                {'frame_prefix': 'orange/'},
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
                {'frame_prefix': 'orange/'},
                {'parent_frame': 'world'},
                {'child_frame': 'base_footprint'},
            ],
            remappings=[
                ('audibot/pose', '/orange/pose'),
            ],
            output='screen'
        ),
    ])

    # -------------------------------------------------------
    # RViz2
    # -------------------------------------------------------
    rviz_config = os.path.join(pfms_dir, 'rviz', 'a3_racing.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='a3_audi_rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return [
        gz_sim,
        clock_bridge,
        spawn_orange_audibot,
        orange_group,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Launch Gazebo with GUI (true) or headless mode (false)'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz (true) or not (false)'),
        OpaqueFunction(function=launch_setup)
    ])
