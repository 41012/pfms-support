import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import xacro


ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare("gazebo_tf"), "worlds", "race_track.world"]),
        description='The world path, by default is race_track.world'),
    DeclareLaunchArgument('gui', default_value='false',
                          description='Whether to launch the GUI'),
    # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='/usr/share/gazebo-11/models:$HOME/.gazebo/models:$HOME/ros2_ws/install/gazebo_tf/share/gazebo_tf/models/'),                          
    #model_path =  os.environ['GAZEBO_MODEL_PATH'],
    AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(get_package_share_directory('gazebo_tf'), 'models')),                          
]


def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')
    # prefix = LaunchConfiguration('prefix')

    
    
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
        condition=IfCondition(LaunchConfiguration('gui')),
    )


    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
   
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='a3_audi_rviz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_tf'), 'rviz', 'audi.rviz')]
    )



    orange_audibot_options = dict(
        robot_name = 'orange',
        start_x = '24.2',
        start_y = '13.2',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
        blue = 'false'
    )
    
    spawn_orange_audibot = GroupAction(
        actions=[
            PushRosNamespace('orange'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('audibot_gazebo'), 'launch', 'audibot_robot.launch.py')
                ]),
                launch_arguments=orange_audibot_options.items()
            )
        ]
    )

    gazebo_connect = Node(
        package='gazebo_tf',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    # Make sure spawn_husky_velocixxxxty_controller starts after spawn_joint_state_broadcaster
    spawn_orange_robot_callback = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[
                LogInfo(msg='gzserver has started!'),
                TimerAction(
                    period=5.0,
                    actions=[spawn_orange_audibot],
                )
            ]
        )
    )    

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(gazebo_connect)
    ld.add_action(rviz)
    ld.add_action(spawn_orange_robot_callback)

    return ld
