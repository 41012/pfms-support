import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():

    mode = launch.substitutions.LaunchConfiguration('mode')
    world = os.path.join(get_package_share_directory('gazebo_tf'), 'worlds')
    pkg_gazebo_tf_models = get_package_share_directory('gazebo_tf')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_gazebo_tf_models + '/models'
    else:
        model_path =  pkg_gazebo_tf_models + '/models'

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
    mode = launch.substitutions.LaunchConfiguration('mode')

    gazebo_connect = Node(
        package='gazebo_tf',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    orange_audibot_options = dict(
        robot_name = 'orange',
        start_x = '0',
        start_y = '2',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
        blue = 'false'
    )

    blue_audibot_options = dict(
        robot_name = 'blue',
        start_x = '0',
        start_y = '-2',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
        blue = 'true'
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

    spawn_blue_audibot = GroupAction(
        actions=[
            PushRosNamespace('blue'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('audibot_gazebo'), 'launch', 'audibot_robot.launch.py')
                ]),
                launch_arguments=blue_audibot_options.items()
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='two_audi_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_tf'), 'rviz', 'two_cars.rviz')]
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

    audi_orange_reach = Node(
        package='gazebo_tf',
        executable='reach',
        name='audi_orange_reach',
        output='screen'
        # output={'both': 'log'},
    )

    audi_blue_reach = Node(
        package='gazebo_tf',
        executable='reach',
        name='audi_blue_reach',
        output='screen',
        # output={'both': 'log'},
        remappings=[
            ('/orange/odom', '/blue/odom'),
            ('/orange/check_goals', '/blue/check_goals'),
            ('ackerman_check_goals', 'ackerman_blue_check_goals'),
        ]        
    )


    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[PythonExpression(['"',world,'" + "/demo.world"']),''],
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
        spawn_orange_audibot,
        spawn_blue_audibot,
        # robot_state_publisher,
        joint_state_publisher,
        rviz,
        audi_blue_reach,
        audi_orange_reach,
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
