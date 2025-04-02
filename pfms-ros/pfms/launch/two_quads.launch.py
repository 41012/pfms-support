import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro

def generate_launch_description():

    mode = launch.substitutions.LaunchConfiguration('mode')
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
    # mode = launch.substitutions.LaunchConfiguration('mode')

    # gazebo_connect = Node(
    #     package='pfms',
    #     executable='gazebo_connect',
    #     name='gazebo_connect',
    #     parameters=[{'use_sim_time': False}]
    #     # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    # )

    drone1_options = dict(
        robot_name = 'drone1',
        start_x = '0',
        start_y = '-1',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
    )

    # drone2_options = dict(
    #     robot_name = 'drone2',
    #     start_x = '0',
    #     start_y = '1',
    #     start_z = '0',
    #     start_yaw = '0',
    #     pub_tf = 'true',
    #     tf_freq = '100.0',
    # )

    spawn_drone1 = GroupAction(
        actions=[
            PushRosNamespace('drone1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_robot.launch.py')
                ]),
                launch_arguments=drone1_options.items()
            )
        ]
    )

    # spawn_drone2 = GroupAction(
    #     actions=[
    #         PushRosNamespace('drone2'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_robot.launch.py')
    #             ]),
    #             launch_arguments=drone2_options.items()
    #         )
    #     ]
    # )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='two_quad_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'new_a2.rviz')]
    )


    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     # namespace=model_ns,
    #     output="screen",
    #     parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
    #     arguments=[robot_desc]
    # )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # namespace=model_ns,
        output='screen',
    )

    # audi_orange_reach = Node(
    #     package='pfms',
    #     executable='reach',
    #     name='audi_orange_reach',
    #     output='screen'
    #     # output={'both': 'log'},
    # )

    # audi_blue_reach = Node(
    #     package='pfms',
    #     executable='reach',
    #     name='audi_blue_reach',
    #     output='screen',
    #     # output={'both': 'log'},
    #     remappings=[
    #         ('/orange/odom', '/blue/odom'),
    #         ('/orange/check_goals', '/blue/check_goals'),
    #         ('ackerman_check_goals', 'ackerman_blue_check_goals'),
    #     ]        
    # )

    # Spawn robot
    # <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" 
    # args="-urdf -unpause -param robot_description -model robot -z 0.0 -J elbow_joint -1.57" respawn="false" output="screen" />
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity',
                   'husky',
                   '-topic',
                   'robot_description',
                   '-x 0.0', '-y -5.0'],
        output='screen',
    )

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
#        respawn=True,
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    gazebo_connect = Node(
        package='pfms',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': True}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[PythonExpression(['"',world,'" + "/a2.world"']),''],
          description='SDF world file'),

        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),

        # launch.actions.DeclareLaunchArgument(
        #   name='mode',
        #   default_value='night',
        #   description='day or night modes are available'),

        launch.actions.DeclareLaunchArgument(
            name='extra_gazebo_args',
            default_value='--verbose',
            description='Extra plugins for (Gazebo)'),

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
          
        gazebo_server,
        gazebo_client,
        gazebo_connect,
        # spawn_drone1,
        # spawn_drone2,
        # robot_state_publisher,
        joint_state_publisher,
        rviz,
        # audi_blue_reach,
        # audi_orange_reach,
        node_robot_state_publisher,
        spawn_joint_state_broadcaster,
        diffdrive_controller_spawn_callback,
        spawn_robot
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
