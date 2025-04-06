import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare("pfms"), "worlds", "demo.world"]),
        description='The world path, by default is demo.world'),
    DeclareLaunchArgument('gui', default_value='false',
                          description='Whether to launch the GUI'),
]


def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')

    world = os.path.join(get_package_share_directory('pfms'), 'worlds')
    pkg_pfms_models = get_package_share_directory('pfms')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_pfms_models + '/models'
    else:
        model_path =  pkg_pfms_models + '/models'

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

    drone1_options = dict(
        robot_name = 'drone3',
        start_x = '0',
        start_y = '2',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
    )

    drone2_options = dict(
        robot_name = 'drone4',
        start_x = '0',
        start_y = '-2',
        start_z = '0',
        start_yaw = '0',
        pub_tf = 'true',
        tf_freq = '100.0',
    )

    spawn_drone1 = GroupAction(
        actions=[
            PushRosNamespace('drone3'),
             IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_robot.launch.py')
                ]),
                launch_arguments=drone1_options.items()
            )
        ]
    )    

    spawn_drone2 = GroupAction(
        actions=[
            PushRosNamespace('drone4'),
             IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_robot.launch.py')
                ]),
                launch_arguments=drone2_options.items()
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='single_drone_viz',
        arguments=['-d', os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'rviz', 'single_quad.rviz')]
    )

    # return LaunchDescription([
    #     gzserver,
    #     gzclient,
    #     spawn_audibot,
    #     rviz
    # ])

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)
    ld.add_action(spawn_drone1)
    ld.add_action(spawn_drone2)
    # ld.add_action(rviz)
    return ld
