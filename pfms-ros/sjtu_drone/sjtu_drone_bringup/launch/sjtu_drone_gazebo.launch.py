#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Include the higher-level scenario launch `a1_sup.launch.py` which
    # handles Gazebo (Ignition) bringup and robot spawning.
    pkg_pfms = get_package_share_directory('pfms')
    a1_sup_launch = os.path.join(pkg_pfms, 'launch', 'a1_sup.launch.py')

    # Path to sjtu_drone SDF
    sjtu_sdf = os.path.join(get_package_share_directory('sjtu_drone_description'), 'models', 'sjtu_drone', 'sjtu_drone.sdf')

    # Bridge configuration (passed into gazebo_bringup)
    bridge_config = os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'config', 'ros_ign_bridge.yaml')

    # Include the generic gazebo bringup from audibot_gazebo, but tell it
    # to spawn the sjtu_drone model and use our bridge config.
    gz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('audibot_gazebo'), 'launch', 'gazebo_bringup.launch.py')
        ),
        launch_arguments={
            'robot_sdf_file': sjtu_sdf,
            'world_sdf_file': os.path.join(get_package_share_directory('pfms'), 'worlds', 'a1.world'),
            'gz_bridge_file': bridge_config,
            'verbose': 'true',
            'start_paused': 'false',
        }.items()
    )

    return LaunchDescription([
        gz_bringup,
    ])