#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SectorObstacleAvoider(Node):
    def __init__(self):
        super().__init__('sector_obstacle_avoider')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Sector Obstacle Avoider Node Started")

    def scan_callback(self, msg: LaserScan):
        # sector parameters
        sector_angle = math.radians(45)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        start_idx = int((0 - sector_angle - angle_min) / angle_increment)
        end_idx = int((0 + sector_angle - angle_min) / angle_increment)

        if start_idx < 0:
            start_idx = 0
        if end_idx >= len(ranges):
            end_idx = len(ranges) - 1

        sector_ranges = ranges[start_idx:end_idx + 1]

        valid_ranges = [r for r in sector_ranges if 0.05 < r < float('inf')]

        min_dist = min(valid_ranges) if valid_ranges else float('inf')

        twist = Twist()
        threshold_distance = 0.5  # meters

        if min_dist < threshold_distance:
            # Obstacle detected in sector: gently turn right
            twist.linear.x = 0.05  # slow forward while turning
            twist.angular.z = -0.5  # gentle right turn
            self.get_logger().info(f"Obstacle at {min_dist:.2f}m in 90° front sector. Turning right.")
        else:
            # Path clear: move forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Path clear. Moving forward.")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SectorObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()