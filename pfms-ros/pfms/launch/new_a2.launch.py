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
    
    # xacro_file_name = "sjtu_drone.urdf.xacro"
    # xacro_file = os.path.join(
    #     get_package_share_directory("sjtu_drone_description"),
    #     "urdf", xacro_file_name
    # )
    # robot_description_config = xacro.process_file(xacro_file)
    # robot_desc = robot_description_config.toxml()
    # model_ns = "drone"

    # gazebo_connect = Node(
    #     package='pfms',
    #     executable='gazebo_connect',
    #     name='gazebo_connect',
    #     parameters=[{'use_sim_time': True}]
    #     # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    # )

    drone1_options = dict(
            robot_name = 'drone1',
            start_x = '0',
            start_y = '2',
            start_z = '0',
            start_yaw = '0',
            pub_tf = 'true',
            tf_freq = '100.0',
        )
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='a2_viz',
        # output='screen',
        output={'both': 'log'},
        arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'quiz4.rviz')]
    )


    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher_drone",
    #     # namespace=model_ns,
    #     output="screen",
    #     parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
    #     arguments=[robot_desc]
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher_drone',
    #     # namespace=model_ns,
    #     output='screen',
    # )

    # sjtu_drone_bringup= Node(
    #     package="sjtu_drone_bringup",
    #     executable="spawn_drone",
    #     arguments=[robot_desc, model_ns],
    #     output="screen"
    # )

    # audi_reach = Node(
    #     package='pfms',
    #     executable='reach',
    #     name='audi_reach',
    #     output='screen'
    #     # output={'both': 'log'},
    #     # arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'audi_husky.rviz')]
    # )

    drone1_reach = Node(
        package='pfms',
        executable='reach',
        name='drone1_reach',
        output='screen',
        # output={'both': 'log'},
        # arguments=['-d', os.path.join(get_package_share_directory('pfms'), 'rviz', 'audi_husky.rviz')]
        remappings=[
            ('/orange/odom', '/drone1/gt_odom'),
            ('/orange/check_goals', '/drone1/check_goals'),
            ('ackerman_check_goals', 'drone1_check_goals'),
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
        # gazebo_connect,
        spawn_drone1,
        # robot_state_publisher,
        # joint_state_publisher,
        # sjtu_drone_bringup,
        rviz,
        # audi_reach,
        drone1_reach
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
