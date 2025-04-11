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

import yaml
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

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )   

    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()
    # get ns from yaml
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"] #+ "/"
    print("namespace: ", model_ns)


    gazebo_connect = Node(
        package='pfms',
        executable='gazebo_connect',
        name='gazebo_connect',
        parameters=[{'use_sim_time': False}]
        # arguments=['-d', os.path.join(get_package_share_directory('audibot_gazebo'), 'rviz', 'two_vehicle_example.rviz')]
    )

    orange_audibot_options = dict(
        robot_name = 'orange',
        start_x = '0',
        start_y = '2.0',
        start_z = '0',
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
        namespace=model_ns,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
        arguments=[robot_desc]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=model_ns,
        output='screen',
    )

    sjtu_drone_bringup= Node(
        package="sjtu_drone_bringup",
        executable="spawn_drone",
        arguments=[robot_desc, model_ns],
        output="screen"
    )

   
    audi_reach = Node(
        package='pfms',
        executable='reach',
        name='audi_reach',
        output='screen'
    )

    drone_reach = Node(
        package='pfms',
        executable='reach',
        name='drone_reach',
        output='screen',
        # output={'both': 'log'},
        remappings=[
            ('/orange/odom', '/drone/gt_odom'),
            ('/orange/check_goals', '/drone/check_goals'),
            ('ackerman_check_goals', 'drone_check_goals'),
        ]
    )

    drone_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
        output="screen"
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

        sjtu_drone_bringup,
        robot_state_publisher,
        joint_state_publisher,

        # gazebo_connect,
        rviz,
        spawn_orange_audibot,
        audi_reach,
        drone_reach,
        drone_tf
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
