#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class FrameNode : public rclcpp::Node
{
public:
  FrameNode()
  : Node("frame_node")
  {
    // Create a subscriber to the input image topic
    sub_ = image_transport::create_subscription(this, "/camera/image_raw", 
                                                std::bind(&FrameNode::imageCallback, this, std::placeholders::_1),
                                                "raw");

    // Create a publisher for the output image topic
    pub_ = image_transport::create_publisher(this, "/camera/image_framed");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try
    {
      // Convert the ROS image message to an OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // Draw a rectangle in the center of the image
      int width = cv_ptr->image.cols;
      int height = cv_ptr->image.rows;
      int rect_width = 100;
      int rect_height = 100;
      cv::rectangle(cv_ptr->image, 
                    cv::Point(width / 2 - rect_width / 2, height / 2 - rect_height / 2),
                    cv::Point(width / 2 + rect_width / 2, height / 2 + rect_height / 2),
                    CV_RGB(255, 0, 0), 2);

      // Convert the OpenCV image back to a ROS image message and publish it
      pub_.publish(*cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameNode>());
  rclcpp::shutdown();
  return 0;
}