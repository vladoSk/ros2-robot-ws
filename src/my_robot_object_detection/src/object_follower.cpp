/**
 * @file object_follower.cpp
 * @brief Object Follower Node
 *
 * Makes the robot follow detected objects using proportional control
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

class ObjectFollower : public rclcpp::Node
{
public:
    ObjectFollower() : Node("object_follower")
    {
        // Declare parameters
        this->declare_parameter("enabled", true);
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("angular_speed", 0.5);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("center_tolerance", 50);
        this->declare_parameter("min_area", 5000.0);
        this->declare_parameter("max_area", 100000.0);
        this->declare_parameter("kp_angular", 0.003);
        this->declare_parameter("kp_linear", 0.00001);

        // Get parameters
        enabled_ = this->get_parameter("enabled").as_bool();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        center_tolerance_ = this->get_parameter("center_tolerance").as_int();
        min_area_ = this->get_parameter("min_area").as_double();
        max_area_ = this->get_parameter("max_area").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        kp_linear_ = this->get_parameter("kp_linear").as_double();

        // State variables
        object_detected_ = false;
        object_center_x_ = 0.0;
        object_center_y_ = 0.0;
        object_area_ = 0.0;
        image_center_x_ = image_width_ / 2.0;

        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribers
        detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/object_detection/detected", 10,
            std::bind(&ObjectFollower::detectionCallback, this, std::placeholders::_1));

        center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_detection/center", 10,
            std::bind(&ObjectFollower::centerCallback, this, std::placeholders::_1));

        // Timer for control loop (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObjectFollower::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Object Follower Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Follower enabled: %s", enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f m/s", linear_speed_);
        RCLCPP_INFO(this->get_logger(), "Angular speed: %.2f rad/s", angular_speed_);
        RCLCPP_INFO(this->get_logger(), "Image center X: %.1f", image_center_x_);
    }

    ~ObjectFollower()
    {
        // Stop the robot on shutdown
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
    }

private:
    void detectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        object_detected_ = msg->data;
    }

    void centerCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        object_center_x_ = msg->x;
        object_center_y_ = msg->y;
        object_area_ = msg->z;  // Area is stored in z coordinate
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        if (!enabled_) {
            // If disabled, stop the robot
            cmd_vel_pub_->publish(twist);
            return;
        }

        if (!object_detected_) {
            // No object detected, stop the robot
            cmd_vel_pub_->publish(twist);
            return;
        }

        // Calculate error from center
        double error_x = object_center_x_ - image_center_x_;

        // Calculate angular velocity (proportional control)
        // Negative because positive angular velocity turns left
        double angular_z = -kp_angular_ * error_x;

        // Clamp angular velocity
        if (angular_z > angular_speed_) {
            angular_z = angular_speed_;
        } else if (angular_z < -angular_speed_) {
            angular_z = -angular_speed_;
        }

        // Calculate linear velocity based on object area
        // Move forward if object is small (far away)
        // Move backward if object is too large (too close)
        // Stop if object is at desired distance
        double linear_x = 0.0;

        if (object_area_ < min_area_) {
            // Object too far, move forward
            double area_error = min_area_ - object_area_;
            linear_x = kp_linear_ * area_error;
            if (linear_x > linear_speed_) {
                linear_x = linear_speed_;
            }
        } else if (object_area_ > max_area_) {
            // Object too close, move backward
            double area_error = max_area_ - object_area_;
            linear_x = kp_linear_ * area_error;
            if (linear_x < -linear_speed_) {
                linear_x = -linear_speed_;
            }
        } else {
            // Object at good distance, just rotate to center it
            linear_x = 0.0;
        }

        // If object is not centered, prioritize rotation
        if (std::abs(error_x) > center_tolerance_) {
            // Reduce linear speed while centering
            linear_x *= 0.3;
        }

        // Create and publish Twist message
        twist.linear.x = linear_x;
        twist.angular.z = angular_z;
        cmd_vel_pub_->publish(twist);

        // Log status (throttled to once per second)
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,  // milliseconds
            "Following: cx=%.0f, error_x=%.0f, area=%.0f, linear=%.2f, angular=%.2f",
            object_center_x_, error_x, object_area_, linear_x, angular_z
        );
    }

    // Parameters
    bool enabled_;
    double linear_speed_;
    double angular_speed_;
    int image_width_;
    int image_height_;
    int center_tolerance_;
    double min_area_;
    double max_area_;
    double kp_angular_;
    double kp_linear_;

    // State variables
    bool object_detected_;
    double object_center_x_;
    double object_center_y_;
    double object_area_;
    double image_center_x_;

    // ROS2 publishers, subscribers, and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
