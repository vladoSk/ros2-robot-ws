#!/usr/bin/env python3
"""
Object Follower Node
Makes the robot follow detected objects using proportional control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool


class ObjectFollower(Node):
    """ROS2 node that makes the robot follow detected objects"""

    def __init__(self):
        super().__init__('object_follower')

        # Declare parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('center_tolerance', 50)
        self.declare_parameter('min_area', 5000)
        self.declare_parameter('max_area', 100000)
        self.declare_parameter('kp_angular', 0.003)  # Proportional gain for angular velocity
        self.declare_parameter('kp_linear', 0.00001)  # Proportional gain for linear velocity

        # Get parameters
        self.enabled = self.get_parameter('enabled').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kp_linear = self.get_parameter('kp_linear').value

        # State variables
        self.object_detected = False
        self.object_center = None
        self.object_area = 0.0
        self.image_center_x = self.image_width / 2

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers
        self.detection_sub = self.create_subscription(
            Bool,
            '/object_detection/detected',
            self.detection_callback,
            10
        )
        self.center_sub = self.create_subscription(
            Point,
            '/object_detection/center',
            self.center_callback,
            10
        )

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Object Follower Node Initialized')
        self.get_logger().info(f'Follower enabled: {self.enabled}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Image center X: {self.image_center_x}')

    def detection_callback(self, msg):
        """Update object detection status"""
        self.object_detected = msg.data

    def center_callback(self, msg):
        """Update object center position and area"""
        self.object_center = msg
        self.object_area = msg.z  # Area is stored in z coordinate

    def control_loop(self):
        """Main control loop for following objects"""
        twist = Twist()

        if not self.enabled:
            # If disabled, stop the robot
            self.cmd_vel_pub.publish(twist)
            return

        if not self.object_detected or self.object_center is None:
            # No object detected, stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Calculate error from center
        cx = self.object_center.x
        error_x = cx - self.image_center_x

        # Calculate angular velocity (proportional control)
        # Negative because positive angular velocity turns left
        angular_z = -self.kp_angular * error_x

        # Clamp angular velocity
        if angular_z > self.angular_speed:
            angular_z = self.angular_speed
        elif angular_z < -self.angular_speed:
            angular_z = -self.angular_speed

        # Calculate linear velocity based on object area
        # Move forward if object is small (far away)
        # Move backward if object is too large (too close)
        # Stop if object is at desired distance
        linear_x = 0.0

        if self.object_area < self.min_area:
            # Object too far, move forward
            area_error = self.min_area - self.object_area
            linear_x = self.kp_linear * area_error
            if linear_x > self.linear_speed:
                linear_x = self.linear_speed
        elif self.object_area > self.max_area:
            # Object too close, move backward
            area_error = self.max_area - self.object_area
            linear_x = self.kp_linear * area_error
            if linear_x < -self.linear_speed:
                linear_x = -self.linear_speed
        else:
            # Object at good distance, just rotate to center it
            linear_x = 0.0

        # If object is not centered, prioritize rotation
        if abs(error_x) > self.center_tolerance:
            # Reduce linear speed while centering
            linear_x *= 0.3

        # Create and publish Twist message
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.cmd_vel_pub.publish(twist)

        # Log status
        self.get_logger().info(
            f'Following: cx={int(cx)}, error_x={int(error_x)}, '
            f'area={int(self.object_area)}, '
            f'linear={linear_x:.2f}, angular={angular_z:.2f}',
            throttle_duration_sec=1.0  # Log once per second
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
