#!/usr/bin/env python3
"""
Object Detection Node using OpenCV and ROS2
Detects colored objects (default: red objects) in camera feed
Publishes annotated images and detection results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    """ROS2 node for detecting colored objects using OpenCV"""

    def __init__(self):
        super().__init__('object_detector')

        # Declare parameters for color detection (HSV range for red object)
        self.declare_parameter('lower_hue_1', 0)
        self.declare_parameter('lower_saturation_1', 100)
        self.declare_parameter('lower_value_1', 100)
        self.declare_parameter('upper_hue_1', 10)
        self.declare_parameter('upper_saturation_1', 255)
        self.declare_parameter('upper_value_1', 255)

        # Second range for red (wraps around in HSV)
        self.declare_parameter('lower_hue_2', 170)
        self.declare_parameter('lower_saturation_2', 100)
        self.declare_parameter('lower_value_2', 100)
        self.declare_parameter('upper_hue_2', 180)
        self.declare_parameter('upper_saturation_2', 255)
        self.declare_parameter('upper_value_2', 255)

        # Detection parameters
        self.declare_parameter('min_area', 500)
        self.declare_parameter('camera_topic', '/my_robot/camera/image_raw')
        self.declare_parameter('show_debug', True)

        # Get parameters
        self.lower_hsv_1 = np.array([
            self.get_parameter('lower_hue_1').value,
            self.get_parameter('lower_saturation_1').value,
            self.get_parameter('lower_value_1').value
        ])
        self.upper_hsv_1 = np.array([
            self.get_parameter('upper_hue_1').value,
            self.get_parameter('upper_saturation_1').value,
            self.get_parameter('upper_value_1').value
        ])
        self.lower_hsv_2 = np.array([
            self.get_parameter('lower_hue_2').value,
            self.get_parameter('lower_saturation_2').value,
            self.get_parameter('lower_value_2').value
        ])
        self.upper_hsv_2 = np.array([
            self.get_parameter('upper_hue_2').value,
            self.get_parameter('upper_saturation_2').value,
            self.get_parameter('upper_value_2').value
        ])
        self.min_area = self.get_parameter('min_area').value
        camera_topic = self.get_parameter('camera_topic').value
        self.show_debug = self.get_parameter('show_debug').value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/object_detection/image_annotated',
            10
        )
        self.detection_pub = self.create_publisher(
            Point,
            '/object_detection/center',
            10
        )
        self.detected_pub = self.create_publisher(
            Bool,
            '/object_detection/detected',
            10
        )

        # Subscriber to camera feed
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.get_logger().info('Object Detector Node Initialized')
        self.get_logger().info(f'Subscribed to: {camera_topic}')
        self.get_logger().info(f'HSV Range 1: {self.lower_hsv_1} to {self.upper_hsv_1}')
        self.get_logger().info(f'HSV Range 2: {self.lower_hsv_2} to {self.upper_hsv_2}')
        self.get_logger().info(f'Minimum detection area: {self.min_area} pixels')

    def image_callback(self, msg):
        """Process incoming camera images and detect objects"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Create masks for color detection (red has two ranges in HSV)
            mask1 = cv2.inRange(hsv, self.lower_hsv_1, self.upper_hsv_1)
            mask2 = cv2.inRange(hsv, self.lower_hsv_2, self.upper_hsv_2)
            mask = cv2.bitwise_or(mask1, mask2)

            # Morphological operations to remove noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CONTOUR_APPROX_SIMPLE
            )

            # Create annotated image
            annotated_image = cv_image.copy()
            object_detected = False
            center_point = Point()

            # Process contours
            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area > self.min_area:
                    object_detected = True

                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    # Calculate center
                    cx = x + w // 2
                    cy = y + h // 2

                    # Store center point
                    center_point.x = float(cx)
                    center_point.y = float(cy)
                    center_point.z = float(area)  # Store area in z coordinate

                    # Draw bounding box
                    cv2.rectangle(
                        annotated_image,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        3
                    )

                    # Draw center point
                    cv2.circle(
                        annotated_image,
                        (cx, cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Draw crosshair
                    cv2.line(
                        annotated_image,
                        (cx - 20, cy),
                        (cx + 20, cy),
                        (0, 0, 255),
                        2
                    )
                    cv2.line(
                        annotated_image,
                        (cx, cy - 20),
                        (cx, cy + 20),
                        (0, 0, 255),
                        2
                    )

                    # Add text information
                    info_text = f"Object Detected | Area: {int(area)}"
                    cv2.putText(
                        annotated_image,
                        info_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    position_text = f"Center: ({cx}, {cy})"
                    cv2.putText(
                        annotated_image,
                        position_text,
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    if self.show_debug:
                        self.get_logger().info(
                            f'Object detected at ({cx}, {cy}) with area {int(area)}'
                        )

            # Add status text if no object detected
            if not object_detected:
                cv2.putText(
                    annotated_image,
                    "No Object Detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

            # Publish results
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(
                    annotated_image,
                    encoding='bgr8'
                )
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')

            # Publish detection status
            detected_msg = Bool()
            detected_msg.data = object_detected
            self.detected_pub.publish(detected_msg)

            # Publish center point if object detected
            if object_detected:
                self.detection_pub.publish(center_point)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
