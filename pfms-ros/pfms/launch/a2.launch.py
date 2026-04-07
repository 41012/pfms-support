import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, GroupAction, SetEnvironmentVariable,
                             RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context):
    # Package directories
    pfms_dir = get_package_share_directory('pfms')
    audibot_gazebo_dir = get_package_share_directory('audibot_gazebo')
    audibot_description_dir = get_package_share_directory('audibot_description')
    husky_gazebo_dir = get_package_share_directory('husky_gazebo')
    
    # Set GZ_SIM_RESOURCE_PATH for Gazebo to find local models
    models_path = os.path.join(pfms_dir, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        models_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + models_path

    # -------------------------------------------------------
    # Gazebo Simulation (Ignition Harmonic)
    # -------------------------------------------------------
    world_file = os.path.join(pfms_dir, 'worlds', 'a2.world')

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
    # Orange Audibot
    # -------------------------------------------------------
    orange_sdf_file = os.path.join(
        audibot_description_dir, 'models', 'orange_audibot', 'model.sdf')
    with open(orange_sdf_file, 'r') as f:
        orange_robot_desc = f.read()

    bridge_config_orange = os.path.join(
        audibot_gazebo_dir, 'config', 'ros_gz_bridge_orange.yaml')

    # Spawn the orange audibot into the world at runtime (it is not embedded in a2.world)
    spawn_orange_audibot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_orange_audibot',
        arguments=['-file', orange_sdf_file, '-x', '0', '-y', '2', '-z', '0'],
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
        Node(
            package='pfms',
            executable='reach',
            name='reach',
            parameters=[{'use_sim_time': True}],
            output='screen',
            remappings=[
                ('/odom', '/orange/odom'),
                ('/register_goals', '/orange/register_goals'),
                ('/check_goals', '/orange/check_goals'),
            ]            
        ),
    ])

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
                   '-x', '0', '-y', '-5', '-z', '0.4']
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
    # RViz2
    # -------------------------------------------------------
    rviz_config = os.path.join(pfms_dir, 'rviz', 'a2.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    
    # Delay RViz start until after models are spawned to avoid timing issues
    delayed_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_husky,
            on_exit=[
                TimerAction(
                    period=2.0,  # Wait 2 seconds after spawn completes
                    actions=[rviz_node]
                )
            ]
        )
    )

    return [
        gz_sim,
        clock_bridge,
        spawn_orange_audibot,
        orange_group,
        husky_group,
        delayed_rviz,
    ]


def generate_launch_description():
    # Set GZ_SIM_RESOURCE_PATH early for Gazebo to find local models
    pfms_dir = get_package_share_directory('pfms')
    models_path = os.path.join(pfms_dir, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        models_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + models_path
    
    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
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
