#!/usr/bin/env python3

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"], description="Whether to execute gzclient")
    xacro_file_name = "sjtu_drone.urdf.xacro"

    world = os.path.join(get_package_share_directory('pfms'), 'worlds')
    pkg_pfms_models = get_package_share_directory('pfms')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_pfms_models + '/models'
    else:
        model_path =  pkg_pfms_models + '/models'


    gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_client = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
     )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    # Load the robot description for the first drone
    xacro_file_1 = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    robot_description_config_1 = xacro.process_file(xacro_file_1)
    robot_desc_1 = robot_description_config_1.toxml()
    model_ns_1 = "drone_1"

    # Load the robot description for the second drone
    xacro_file_2 = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    robot_description_config_2 = xacro.process_file(xacro_file_2)
    robot_desc_2 = robot_description_config_2.toxml()
    model_ns_2 = "drone_2"

    world_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "playground.world"
    )

    # def launch_gzclient(context, *args, **kwargs):
    #     if context.launch_configurations.get('use_gui') == 'true':
    #         return [IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #             ),
    #             launch_arguments={'verbose': 'true'}.items()
    #         )]
    #     return []

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[PythonExpression(['"',world,'" + "/new_a2.world"']),''],
          description='SDF world file'),

        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),

        launch.actions.DeclareLaunchArgument(
            name='extra_gazebo_args',
            default_value='--verbose',
            description='Extra plugins for (Gazebo)'),

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
                 # Robot State Publisher for the first drone
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_1",
            namespace=model_ns_1,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc_1}],
        ),

        # Robot State Publisher for the second drone
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_2",
            namespace=model_ns_2,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc_2}],
        ),

        # Joint State Publisher for the first drone
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_1',
            namespace=model_ns_1,
            output='screen',
        ),

        # Joint State Publisher for the second drone
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_2',
            namespace=model_ns_2,
            output='screen',
        ),

        # Spawn the first drone
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc_1, model_ns_1],
            output="screen"
        ),

        # Spawn the second drone
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc_2, model_ns_2],
            output="screen"
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='two_vehicle_viz',
            # output='screen',
            output={'both': 'log'},
            arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'two_drones.rviz')]
        )        ,
        
        gazebo_server,
        gazebo_client
    ])