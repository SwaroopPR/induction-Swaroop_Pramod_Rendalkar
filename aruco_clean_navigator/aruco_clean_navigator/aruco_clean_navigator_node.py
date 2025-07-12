#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import time

class ArucoCleanNavigator(Node):
    def __init__(self):
        super().__init__('aruco_clean_navigator_node')
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.tag_count = 0
        self.max_tags = 5
        self.processing_tag = False
        self.current_tag_id = None

        self.forward_timer = self.create_timer(0.1, self.keep_moving_forward)

        self.get_logger().info("Aruco Clean Navigator Node started.")

    def keep_moving_forward(self):
        if not self.processing_tag and self.tag_count < self.max_tags:
            twist = Twist()
            twist.linear.x = 0.25  # Faster forward speed
            self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if self.tag_count >= self.max_tags:
            self.stop_robot()
            self.get_logger().info("Processed 5 tags, stopping in 5 seconds.")
            time.sleep(5)
            rclpy.shutdown()
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        detected_ids = ids.flatten() if ids is not None else []

        if self.current_tag_id is not None and self.current_tag_id not in detected_ids:
            self.get_logger().info(f"Tag {self.current_tag_id} lost, ready for next.")
            self.current_tag_id = None
            self.processing_tag = False

        if ids is not None and not self.processing_tag:
            for tag_id in detected_ids:
                if tag_id in [0, 1]:
                    self.processing_tag = True
                    self.current_tag_id = tag_id

                    self.stop_robot()
                    time.sleep(0.2)

                    if tag_id == 0:
                        self.get_logger().info(f"Detected Tag 0 (count: {self.tag_count + 1}): Turning Right 90°")
                        self.turn('right')
                    elif tag_id == 1:
                        self.get_logger().info(f"Detected Tag 1 (count: {self.tag_count + 1}): Turning Left 90°")
                        self.turn('left')
                    break

    def turn(self, direction):
        twist = Twist()
        angular_speed = 0.6  # Slightly faster turn
        target_angle = np.pi / 2
        duration = target_angle / angular_speed

        twist.angular.z = -angular_speed if direction == 'right' else angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)

        self.stop_robot()
        self.tag_count += 1
        self.processing_tag = False

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCleanNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
