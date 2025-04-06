import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, FindExecutable, PathJoinSubstitution, Command
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

import xacro

def generate_launch_description():

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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='a2_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'new_a2.rviz')]
    )

    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_husky',
    #     arguments=['-entity',
    #                'husky',
    #                '-topic',
    #                'robot_description',
    #                '-x 0.0', '-y -2.0'],
    #     output='screen',
    # )

    # Delay spawn_robot by 5 seconds after rviz starts
    spawn_robot_delayed = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_husky',
                arguments=['-entity',
                        'husky',
                        '-topic',
                        'robot_description',
                        '-x 0.0', '-y -2.0'],
                output='screen',
            )
        ]
    )

    # Ensure spawn_robot starts after rviz
    spawn_robot_after_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz,
            on_start=[spawn_robot_delayed]
        )
    )


    gazebo_connect = Node(
        package='pfms',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': True}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    drone1_options = dict(
            robot_name = 'drone',
            start_x = '0',
            start_y = '2',
            start_z = '0',
            start_yaw = '0',
            pub_tf = 'true',
            tf_freq = '100.0',
        )
    spawn_drone = GroupAction(
        actions=[
            PushRosNamespace('drone'),
             IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_robot.launch.py')
                ]),
                launch_arguments=drone1_options.items()
            )
        ]
    )    


    drone1_reach = Node(
        package='pfms',
        executable='reach',
        name='drone1_reach',
        output='screen',
        # output={'both': 'log'},
        # arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'audi_husky.rviz')]
        remappings=[
            ('/orange/odom', '/drone/gt_odom'),
            ('/orange/check_goals', '/drone/check_goals'),
            ('ackerman_check_goals', 'drone_check_goals'),
        ]
    )

    husky_reach = Node(
        package='pfms',
        executable='reach',
        name='husky_reach',
        output='screen',
        remappings=[
            ('/orange/odom', '/husky/odom'),
            ('/orange/check_goals', '/husky/check_goals'),
            ('ackerman_check_goals', 'husky_check_goals'),
        ]
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[PythonExpression(['"',world,'" + "/new_a2.world"']),''],
          description='SDF world file'),

        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),

        launch.actions.DeclareLaunchArgument(
          name='mode',
          default_value='night',
          description='day or night modes are available'),

        launch.actions.DeclareLaunchArgument(
            name='extra_gazebo_args',
            default_value='--verbose',
            description='Extra plugins for (Gazebo)'),

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
          
        gazebo_server,
        gazebo_client,
        gazebo_connect,
        rviz,
        spawn_drone,
        node_robot_state_publisher,
        spawn_joint_state_broadcaster,
        diffdrive_controller_spawn_callback,
        spawn_robot_after_rviz,
        husky_reach,
        drone1_reach
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
