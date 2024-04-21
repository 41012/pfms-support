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

    # Gazebo server
    # gazebo_server = ExecuteProcess(
    #     cmd=['gzserver',
    #          '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',
    #          world + '/a2.world',],
    #     output='screen',
    # )

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
        package='gazebo_tf',
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
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_tf'), 'rviz', 'quiz4.rviz')]
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
        package='gazebo_tf',
        executable='reach',
        name='drone_reach',
        output='screen',
        remappings=[
            ('/orange/odom', '/drone/gt_odom'),
            ('/orange/check_goals', '/drone/check_goals'),
            ('ackerman_check_goals', 'drone_check_goals'),
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
        robot_state_publisher,
        joint_state_publisher,
        sjtu_drone_bringup,
        rviz,
        drone_reach
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
