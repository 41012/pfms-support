import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context):
    # Get package directories
    audibot_gazebo_dir = get_package_share_directory('audibot_gazebo')
    audibot_description_dir = get_package_share_directory('audibot_description')
    
    # Launch Gazebo with the dual audibot world
    verbose_mode_str = LaunchConfiguration('verbose').perform(context)
    verbose_mode = (verbose_mode_str.lower() == 'true')
    start_paused_str = LaunchConfiguration('start_paused').perform(context)
    start_paused = (start_paused_str.lower() == 'true')
    
    world_file = os.path.join(audibot_gazebo_dir, 'worlds', 'dual_audibot_world.sdf')
    gz_arg_str = world_file
    
    if verbose_mode:
        gz_arg_str += ' --verbose'
    
    if not start_paused:
        gz_arg_str += ' -r'
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': gz_arg_str
        }.items()
    )
    
    # Orange robot group with namespace
    orange_sdf_file = os.path.join(audibot_description_dir, 'models', 'orange_audibot', 'model.sdf')
    with open(orange_sdf_file, 'r') as f:
        orange_robot_desc = f.read()
    
    bridge_config_orange = os.path.join(audibot_gazebo_dir, 'config', 'ros_gz_bridge_orange.yaml')
    
    orange_group = GroupAction([
        PushRosNamespace('orange'),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
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
            executable='pose_to_tf.py',
            name='pose_to_tf',
            parameters=[
                {'use_sim_time': True},
                {'frame_prefix': 'orange/'},
                {'parent_frame': 'world'},
                {'child_frame': 'base_footprint'},
            ],
            remappings=[
                ('audibot/pose', '/audibot/pose'),
            ],
            output='screen'
        )
    ])
    
    # Blue robot group with namespace
    blue_sdf_file = os.path.join(audibot_description_dir, 'models', 'blue_audibot', 'model.sdf')
    with open(blue_sdf_file, 'r') as f:
        blue_robot_desc = f.read()
    
    bridge_config_blue = os.path.join(audibot_gazebo_dir, 'config', 'ros_gz_bridge_blue.yaml')
    
    blue_group = GroupAction([
        PushRosNamespace('blue'),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config_blue,
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
                {'robot_description': blue_robot_desc},
                {'frame_prefix': 'blue/'},
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ]
        ),
        Node(
            package='pfms',
            executable='pose_to_tf.py',
            name='pose_to_tf',
            parameters=[
                {'use_sim_time': True},
                {'frame_prefix': 'blue/'},
                {'parent_frame': 'world'},
                {'child_frame': 'base_footprint'},
            ],
            output='screen'
        )
    ])
    
    # RQT Publisher
    rqt_publisher_node = Node(
        package='rqt_publisher',
        executable='rqt_publisher',
        name='rqt_publisher'
    )
    
    # RViz2 (only one instance)
    rviz_config_file = os.path.join(audibot_gazebo_dir, 'rviz', 'audibot_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='audibot_test',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return [
        gz_sim,
        orange_group,
        blue_group,
        rqt_publisher_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='true', description='Configure Gazebo to put verbose output on terminal'),
        DeclareLaunchArgument('start_paused', default_value='false', description='Start the simulation in paused state'),
        OpaqueFunction(function=launch_setup)
    ])
