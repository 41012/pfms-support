import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare("pfms"), "worlds", "a1.world"]),
        description='The world path, by default is a1.world'),
    DeclareLaunchArgument('gui', default_value='false',
                          description='Whether to launch the GUI'),
]


def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')
    # prefix = LaunchConfiguration('prefix')

    # # HUSKY COMMENTED OUT FOR NOW
    # config_husky_velocity_controller = PathJoinSubstitution(
    #     [FindPackageShare("husky_control"), "config", "control.yaml"]
    # )

    # # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
    #         ),
    #         " ",
    #         "name:=husky",
    #         " ",
    #         "prefix:=''",
    #         " ",
    #         "is_sim:=true",
    #         " ",
    #         "gazebo_controllers:=",
    #         config_husky_velocity_controller,
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}

    # spawn_husky_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['husky_velocity_controller', '-c', '/controller_manager'],
    #     output='screen',
    # #        respawn=True,
    # )

    # node_robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[{'use_sim_time': True}, robot_description],
    # )

    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    #     output='screen',
    # )

    # # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    # diffdrive_controller_spawn_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_joint_state_broadcaster,
    #         on_exit=[spawn_husky_velocity_controller],
    #     )
    # )
    
    # # Gazebo server
    # gzserver = ExecuteProcess(
    #     cmd=['gzserver',
    #          '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',
    #          world_path],
    #     output='screen',
    # )

    # # Gazebo client
    # gzclient = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('gui')),
    # )

    # # Spawn robot
    # # <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" 
    # # args="-urdf -unpause -param robot_description -model robot -z 0.0 -J elbow_joint -1.57" respawn="false" output="screen" />
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_husky',
    #     arguments=['-entity',
    #                'husky',
    #                '-topic',
    #                'robot_description',
    #                '-x 0.0', '-y -5.0'],
    #     output='screen',
    # )

    # gazebo_connect = Node(
    #     package='pfms',
    #     executable='gazebo_connect',
    #     name='gazebo_connect',
    #     parameters=[{'use_sim_time': True}]
    #     # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    # )

    # Launch Orange Audibot using gazebo_bringup pattern
    spawn_orange_audibot = GroupAction(
        actions=[
            PushRosNamespace('orange'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('audibot_gazebo'), 'launch', 'gazebo_bringup.launch.py')
                ]),
                launch_arguments={
                    'robot_sdf_file': os.path.join(get_package_share_directory('audibot_description'), 'models', 'orange_audibot', 'model.sdf'),
                    'world_sdf_file': os.path.join(get_package_share_directory('pfms'), 'worlds', 'a1.world'),
                    'gz_bridge_file': os.path.join(get_package_share_directory('audibot_gazebo'), 'config', 'ros_gz_bridge.yaml'),
                    'verbose': 'true',
                    'start_paused': 'false',
                }.items()
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='a1_sup_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'a1_sup.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    audi_reach = Node(
        package='pfms',
        executable='reach',
        name='audi_reach',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # # HUSKY REACH COMMENTED OUT FOR NOW
    # husky_reach = Node(
    #     package='pfms',
    #     executable='reach',
    #     name='husky_reach',
    #     output='screen',
    #     remappings=[
    #         ('/orange/odom', '/husky/odom'),
    #         ('/orange/check_goals', '/husky/check_goals'),
    #         ('ackerman_check_goals', 'husky_check_goals'),
    #     ]
    # )
    
    ld = LaunchDescription(ARGUMENTS)
    # # HUSKY ACTIONS COMMENTED OUT FOR NOW
    # ld.add_action(node_robot_state_publisher)
    # ld.add_action(spawn_joint_state_broadcaster)
    # ld.add_action(diffdrive_controller_spawn_callback)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)
    # ld.add_action(spawn_robot)
    # ld.add_action(gazebo_connect)
    ld.add_action(spawn_orange_audibot)
    ld.add_action(rviz)
    ld.add_action(audi_reach)
    # ld.add_action(husky_reach)
    
    return ld
