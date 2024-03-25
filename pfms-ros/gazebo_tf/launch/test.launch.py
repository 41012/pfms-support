import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():

    mode = launch.substitutions.LaunchConfiguration('mode')
    world = os.path.join(get_package_share_directory('aws_robomaker_racetrack_world'), 'worlds')

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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    model_ns = "drone"



    orange_audibot_options = dict(
            robot_name = 'orange',
            start_x = '15',
            start_y = '0',
            start_z = '0',
            start_yaw = '3.1417',
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='two_vehicle_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_tf'), 'rviz', 'audi_husky.rviz')]
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

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[PythonExpression(['"',world,'" + "/racetrack_" + "', mode, '" + ".world"']),''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
          name='mode',
          default_value='night',
          description='day or night modes are available'),
        gazebo_server,
        gazebo_client,
        spawn_orange_audibot,
        rviz,
        robot_state_publisher,
        joint_state_publisher,
        sjtu_drone_bringup
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
