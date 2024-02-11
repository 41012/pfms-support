import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare("gazebo_tf"), "worlds", "demo.world"]
    ),
                          description='The world path, by default is demo.world'),
]

def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    audibot_options = dict(
        start_x = '0',
        start_y = '0',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
        blue = 'false'
    )

    spawn_audibot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('audibot_gazebo'), 'launch', 'audibot_robot.launch.py')
        ]),
        launch_arguments=audibot_options.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='single_vehicle_viz',
        arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'single_vehicle_example.rviz')]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_audibot,
        rviz
    ])
