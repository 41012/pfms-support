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
import time


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
        
        # Protect against invalid rates
        if publish_rate_hz <= 0.0:
            publish_rate_hz = 20.0
            
        input_mode_raw = self.get_parameter('input_mode').value
        input_mode = str(input_mode_raw) if input_mode_raw is not None else 'pose_array'

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe with sensor-data style QoS to reduce transport overhead.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
    
        # Create a timer instead of relying on message callbacks
        # This decouples processing rate from message rate
        self.latest_pose = None
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        # Profiling metrics
        self.declare_parameter('enable_profiling', False)
        self.enable_profiling = self.get_parameter('enable_profiling').value
        self.callback_count = 0
        self.timer_count = 0
        self.callback_time_total = 0.0
        self.timer_time_total = 0.0
        self.last_profile_report = time.time()
        
        if self.enable_profiling:
            self.profile_timer = self.create_timer(5.0, self.report_profiling)
            self.get_logger().info('Profiling enabled - reports every 5 seconds')

        if input_mode == 'odom':
            self.source_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_store_callback,
                qos
            )
        else:
            self.source_sub = self.create_subscription(
                PoseArray,
                'audibot/pose',
                self.pose_store_callback,
                qos
            )
        
        self.get_logger().info(
            f'Publishing TF: {self.parent_frame} -> {self.child_frame} at {publish_rate_hz:.1f} Hz (mode={input_mode})'
        )
    
    def pose_store_callback(self, msg):
        """Store latest pose without processing - called at high frequency"""
        if self.enable_profiling:
            start = time.perf_counter()
        
        if len(msg.poses) > 0:
            self.latest_pose = msg.poses[0]
        
        if self.enable_profiling:
            self.callback_time_total += (time.perf_counter() - start)
            self.callback_count += 1

    def odom_store_callback(self, msg):
        """Store latest pose from odometry without processing"""
        if self.enable_profiling:
            start = time.perf_counter()
        
        self.latest_pose = msg.pose.pose
        
        if self.enable_profiling:
            self.callback_time_total += (time.perf_counter() - start)
            self.callback_count += 1

    def timer_callback(self):
        """Publish TF at fixed rate from latest stored pose"""
        if self.enable_profiling:
            start = time.perf_counter()
        
        if self.latest_pose is None:
            if self.enable_profiling:
                self.timer_time_total += (time.perf_counter() - start)
                self.timer_count += 1
            return
        
        pose = self.latest_pose
        
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
        
        if self.enable_profiling:
            self.timer_time_total += (time.perf_counter() - start)
            self.timer_count += 1

    def report_profiling(self):
        """Report profiling statistics every 5 seconds"""
        elapsed = time.time() - self.last_profile_report
        
        if self.callback_count > 0:
            avg_callback_us = (self.callback_time_total / self.callback_count) * 1e6
            callback_hz = self.callback_count / elapsed
        else:
            avg_callback_us = 0.0
            callback_hz = 0.0
        
        if self.timer_count > 0:
            avg_timer_us = (self.timer_time_total / self.timer_count) * 1e6
            timer_hz = self.timer_count / elapsed
        else:
            avg_timer_us = 0.0
            timer_hz = 0.0
        
        total_time_ms = (self.callback_time_total + self.timer_time_total) * 1000
        cpu_percent = (total_time_ms / (elapsed * 1000)) * 100
        
        self.get_logger().info(
            f'\n=== Profiling Report (last {elapsed:.1f}s) ===\n'
            f'Callbacks: {self.callback_count} calls ({callback_hz:.1f} Hz), avg {avg_callback_us:.2f} μs/call\n'
            f'Timer: {self.timer_count} calls ({timer_hz:.1f} Hz), avg {avg_timer_us:.2f} μs/call\n'
            f'Total compute: {total_time_ms:.2f} ms, est CPU: {cpu_percent:.1f}%'
        )
        
        # Reset counters
        self.callback_count = 0
        self.timer_count = 0
        self.callback_time_total = 0.0
        self.timer_time_total = 0.0
        self.last_profile_report = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
