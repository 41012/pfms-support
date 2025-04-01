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

# -*- coding: utf-8 -*-
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
#from tf_transformations import quaternion_from_euler  # Import for quaternion conversion


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    namespace = sys.argv[2]

    # Get the initial position
    x = 0.0
    y = 0.0
    yaw =0.0
    if len(sys.argv) > 2:
        x = float(sys.argv[3])
    if len(sys.argv) > 3:
        y = float(sys.argv[4])
    # Get the initial orientation
    if len(sys.argv) > 43:
        yaw = float(sys.argv[5])

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"

    # Set the initial pose
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = 0.0
    
    # Convert yaw to quaternion
    # quaternion = quaternion_from_euler(0.0, 0.0, yaw)  # Roll and pitch are 0.0
    # initial_pose.orientation.x = quaternion[0]
    # initial_pose.orientation.y = quaternion[1]
    # initial_pose.orientation.z = quaternion[2]
    # initial_pose.orientation.w = quaternion[3]
    initial_pose.orientation.w = 1.0

    req.initial_pose = initial_pose    

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()