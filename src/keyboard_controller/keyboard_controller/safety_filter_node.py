#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('safety_filter_node')

        # Parameters
        self.declare_parameter('safe_distance_m', 0.5) 
        self.declare_parameter('front_cone_angle_deg', 60.0) 
        self.declare_parameter('lidar_timeout_sec', 0.5)
        
        # Add a 180-degree offset since the Lidar is mounted backwards
        self.declare_parameter('lidar_yaw_offset_deg', 180.0) 

        self.safe_distance = self.get_parameter('safe_distance_m').value
        self.front_cone_angle = self.get_parameter('front_cone_angle_deg').value
        self.lidar_timeout = self.get_parameter('lidar_timeout_sec').value
        self.lidar_yaw_offset = math.radians(self.get_parameter('lidar_yaw_offset_deg').value)

        self.obstacle_detected = False
        self.last_scan_time = None

        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/teleop_cmd_vel', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Safety filter active. Intercepting /teleop_cmd_vel -> /cmd_vel")

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        
        cone_half_rad = math.radians(self.front_cone_angle / 2.0)
        min_distance = float('inf')
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min or range_val > msg.range_max:
                continue
            
            # Calculate the raw angle from the lidar
            raw_angle = msg.angle_min + i * msg.angle_increment
            
            # Apply the physical offset to rotate the data forward
            adjusted_angle = raw_angle + self.lidar_yaw_offset
            
            # Normalize the angle back to the standard -pi to +pi range
            angle = math.atan2(math.sin(adjusted_angle), math.cos(adjusted_angle))
            
            # Check if this measurement is within the newly rotated front cone
            if abs(angle) <= cone_half_rad:
                if range_val < min_distance:
                    min_distance = range_val
        
        self.obstacle_detected = min_distance < self.safe_distance

    def cmd_callback(self, msg: Twist):
        safe_msg = Twist()
        safe_msg.angular.z = msg.angular.z  # Always allow steering
        
        # Check if lidar is active using the watchdog timer
        lidar_active = False
        if self.last_scan_time is not None:
            dt = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
            if dt < self.lidar_timeout:
                lidar_active = True

        if lidar_active and self.obstacle_detected and msg.linear.x > 0.0:
            self.get_logger().warn("Obstacle ahead! Halting forward movement.", throttle_duration_sec=1.0)
            safe_msg.linear.x = 0.0 # Force speed to zero
        else:
            if not lidar_active and msg.linear.x != 0.0:
                self.get_logger().info("Lidar offline. Passing raw commands through.", throttle_duration_sec=2.0)
            
            safe_msg.linear.x = msg.linear.x # Pass command through normally
            
        self.cmd_pub.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()