import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
        [FindPackageShare("pfms"), "worlds", "terrain_1.world"]),
        description='The world path, by default is terrain_1.world'),
    DeclareLaunchArgument('gui', default_value='false',
                          description='Whether to launch the GUI'),
    AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(get_package_share_directory('pfms'), 'models')),                          
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
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    model_ns = "drone"

    gazebo_connect = Node(
        package='pfms',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='two_vehicle_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'a2.rviz')]
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        # namespace=model_ns,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
        arguments=[robot_desc]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # namespace=model_ns,
        output='screen',
    )

    sjtu_drone_bringup= Node(
        package="sjtu_drone_bringup",
        executable="spawn_drone",
        arguments=[robot_desc, model_ns],
        output="screen"
    )

    drone_reach = Node(
        package='pfms',
        executable='reach',
        name='drone_reach',
        output='screen',
        remappings=[
            ('/orange/odom', '/drone/gt_odom'),
            ('/orange/check_goals', '/drone/check_goals'),
            ('ackerman_check_goals', 'drone_check_goals'),
        ]
    )

    orange_audibot_options = dict(
        robot_name = 'orange',
        start_x = '0',
        start_y = '2',
        start_z = '0.2',
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


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(gazebo_connect)
    ld.add_action(rviz)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(sjtu_drone_bringup)
    ld.add_action(drone_reach)
    ld.add_action(spawn_orange_audibot)

    return ld
