#!/usr/bin/env python3
"""
Simple node to convert Gazebo pose to TF with frame prefix
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        
        # Declare parameters
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('input_mode', 'pose_array')
        
        # Get parameters
        self.frame_prefix = self.get_parameter('frame_prefix').value
        self.parent_frame = self.get_parameter('parent_frame').value
        child_frame_base = self.get_parameter('child_frame').value
        self.child_frame = f"{self.frame_prefix}{child_frame_base}"
        publish_rate_raw = self.get_parameter('publish_rate_hz').value
        publish_rate_hz = float(publish_rate_raw) if publish_rate_raw is not None else 20.0
        input_mode_raw = self.get_parameter('input_mode').value
        input_mode = str(input_mode_raw) if input_mode_raw is not None else 'pose_array'

        # Protect against invalid rates and convert to a minimum publish period.
        if publish_rate_hz <= 0.0:
            publish_rate_hz = 20.0
        self.min_publish_period_ns = int(1e9 / publish_rate_hz)
        self.last_publish_time_ns = 0
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe with sensor-data style QoS to reduce transport overhead.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if input_mode == 'odom':
            self.source_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                qos
            )
        else:
            self.source_sub = self.create_subscription(
                PoseArray,
                'audibot/pose',
                self.pose_callback,
                qos
            )
        
        self.get_logger().info(
            f'Publishing TF: {self.parent_frame} -> {self.child_frame} at <= {publish_rate_hz:.1f} Hz (mode={input_mode})'
        )

    def should_publish(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_publish_time_ns < self.min_publish_period_ns:
            return False, now_ns
        return True, now_ns
    
    def pose_callback(self, msg):
        if len(msg.poses) == 0:
            return

        can_publish, now_ns = self.should_publish()
        if not can_publish:
            return
        
        # Take the first pose (should be the base_footprint)
        pose = msg.poses[0]
        
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        self.last_publish_time_ns = now_ns

    def odom_callback(self, msg):
        can_publish, now_ns = self.should_publish()
        if not can_publish:
            return

        pose = msg.pose.pose

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
        self.last_publish_time_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
