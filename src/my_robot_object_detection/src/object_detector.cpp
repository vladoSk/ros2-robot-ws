/**
 * @file object_detector.cpp
 * @brief Object Detection Node using OpenCV and ROS2
 *
 * Detects colored objects (default: red objects) in camera feed
 * Publishes annotated images and detection results
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ObjectDetector : public rclcpp::Node
{
public:
    ObjectDetector() : Node("object_detector")
    {
        // Declare parameters for color detection (HSV range for red object)
        this->declare_parameter("lower_hue_1", 0);
        this->declare_parameter("lower_saturation_1", 100);
        this->declare_parameter("lower_value_1", 100);
        this->declare_parameter("upper_hue_1", 10);
        this->declare_parameter("upper_saturation_1", 255);
        this->declare_parameter("upper_value_1", 255);

        // Second range for red (wraps around in HSV)
        this->declare_parameter("lower_hue_2", 170);
        this->declare_parameter("lower_saturation_2", 100);
        this->declare_parameter("lower_value_2", 100);
        this->declare_parameter("upper_hue_2", 180);
        this->declare_parameter("upper_saturation_2", 255);
        this->declare_parameter("upper_value_2", 255);

        // Detection parameters
        this->declare_parameter("min_area", 500);
        this->declare_parameter("camera_topic", "/my_robot/camera/image_raw");
        this->declare_parameter("show_debug", true);

        // Get parameters
        lower_hsv_1_ = cv::Scalar(
            this->get_parameter("lower_hue_1").as_int(),
            this->get_parameter("lower_saturation_1").as_int(),
            this->get_parameter("lower_value_1").as_int()
        );
        upper_hsv_1_ = cv::Scalar(
            this->get_parameter("upper_hue_1").as_int(),
            this->get_parameter("upper_saturation_1").as_int(),
            this->get_parameter("upper_value_1").as_int()
        );
        lower_hsv_2_ = cv::Scalar(
            this->get_parameter("lower_hue_2").as_int(),
            this->get_parameter("lower_saturation_2").as_int(),
            this->get_parameter("lower_value_2").as_int()
        );
        upper_hsv_2_ = cv::Scalar(
            this->get_parameter("upper_hue_2").as_int(),
            this->get_parameter("upper_saturation_2").as_int(),
            this->get_parameter("upper_value_2").as_int()
        );
        min_area_ = this->get_parameter("min_area").as_int();
        show_debug_ = this->get_parameter("show_debug").as_bool();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();

        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/object_detection/image_annotated", 10);
        detection_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/object_detection/center", 10);
        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/object_detection/detected", 10);

        // Subscriber to camera feed
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&ObjectDetector::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Object Detector Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "HSV Range 1: [%d, %d, %d] to [%d, %d, %d]",
            (int)lower_hsv_1_[0], (int)lower_hsv_1_[1], (int)lower_hsv_1_[2],
            (int)upper_hsv_1_[0], (int)upper_hsv_1_[1], (int)upper_hsv_1_[2]);
        RCLCPP_INFO(this->get_logger(), "HSV Range 2: [%d, %d, %d] to [%d, %d, %d]",
            (int)lower_hsv_2_[0], (int)lower_hsv_2_[1], (int)lower_hsv_2_[2],
            (int)upper_hsv_2_[0], (int)upper_hsv_2_[1], (int)upper_hsv_2_[2]);
        RCLCPP_INFO(this->get_logger(), "Minimum detection area: %d pixels", min_area_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS Image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat cv_image = cv_ptr->image;

            // Convert BGR to HSV color space
            cv::Mat hsv;
            cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

            // Create masks for color detection (red has two ranges in HSV)
            cv::Mat mask1, mask2, mask;
            cv::inRange(hsv, lower_hsv_1_, upper_hsv_1_, mask1);
            cv::inRange(hsv, lower_hsv_2_, upper_hsv_2_, mask2);
            cv::bitwise_or(mask1, mask2, mask);

            // Morphological operations to remove noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::erode(mask, mask, kernel, cv::Point(-1, -1), 2);
            cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Create annotated image
            cv::Mat annotated_image = cv_image.clone();
            bool object_detected = false;
            geometry_msgs::msg::Point center_point;

            // Process contours
            if (!contours.empty()) {
                // Find the largest contour
                auto largest_contour = std::max_element(
                    contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                        return cv::contourArea(c1) < cv::contourArea(c2);
                    }
                );

                double area = cv::contourArea(*largest_contour);

                if (area > min_area_) {
                    object_detected = true;

                    // Get bounding rectangle
                    cv::Rect bounding_rect = cv::boundingRect(*largest_contour);
                    int x = bounding_rect.x;
                    int y = bounding_rect.y;
                    int w = bounding_rect.width;
                    int h = bounding_rect.height;

                    // Calculate center
                    int cx = x + w / 2;
                    int cy = y + h / 2;

                    // Store center point
                    center_point.x = static_cast<double>(cx);
                    center_point.y = static_cast<double>(cy);
                    center_point.z = area;  // Store area in z coordinate

                    // Draw bounding box
                    cv::rectangle(annotated_image,
                                cv::Point(x, y),
                                cv::Point(x + w, y + h),
                                cv::Scalar(0, 255, 0), 3);

                    // Draw center point
                    cv::circle(annotated_image, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);

                    // Draw crosshair
                    cv::line(annotated_image,
                           cv::Point(cx - 20, cy),
                           cv::Point(cx + 20, cy),
                           cv::Scalar(0, 0, 255), 2);
                    cv::line(annotated_image,
                           cv::Point(cx, cy - 20),
                           cv::Point(cx, cy + 20),
                           cv::Scalar(0, 0, 255), 2);

                    // Add text information
                    std::string info_text = "Object Detected | Area: " + std::to_string(static_cast<int>(area));
                    cv::putText(annotated_image, info_text,
                              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                              0.7, cv::Scalar(0, 255, 0), 2);

                    std::string position_text = "Center: (" + std::to_string(cx) + ", " + std::to_string(cy) + ")";
                    cv::putText(annotated_image, position_text,
                              cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                              0.7, cv::Scalar(0, 255, 0), 2);

                    if (show_debug_) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Object detected at (%d, %d) with area %.0f", cx, cy, area);
                    }
                }
            }

            // Add status text if no object detected
            if (!object_detected) {
                cv::putText(annotated_image, "No Object Detected",
                          cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                          0.7, cv::Scalar(0, 0, 255), 2);
            }

            // Publish annotated image
            try {
                sensor_msgs::msg::Image::SharedPtr annotated_msg =
                    cv_bridge::CvImage(msg->header, "bgr8", annotated_image).toImageMsg();
                image_pub_->publish(*annotated_msg);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to publish annotated image: %s", e.what());
            }

            // Publish detection status
            std_msgs::msg::Bool detected_msg;
            detected_msg.data = object_detected;
            detected_pub_->publish(detected_msg);

            // Publish center point if object detected
            if (object_detected) {
                detection_pub_->publish(center_point);
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    // Parameters
    cv::Scalar lower_hsv_1_;
    cv::Scalar upper_hsv_1_;
    cv::Scalar lower_hsv_2_;
    cv::Scalar upper_hsv_2_;
    int min_area_;
    bool show_debug_;

    // ROS2 publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr detection_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
