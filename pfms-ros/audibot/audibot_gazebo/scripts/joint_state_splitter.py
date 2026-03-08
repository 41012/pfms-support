#!/usr/bin/env python3
"""
Node to split combined joint_states by robot name
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('joint_state_splitter')
        
        # Declare parameters
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value
        
        # Subscribe to combined joint_states
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states_combined',
            self.joint_callback,
            10
        )
        
        # Publish filtered joint_states
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.get_logger().info(f'Filtering joint_states for robot: {self.robot_name}')
    
    def joint_callback(self, msg):
        # Check if this message is for our robot
        # The frame_id or header might contain the robot name
        # For now, just republish - we'll need to modify based on actual message format
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
