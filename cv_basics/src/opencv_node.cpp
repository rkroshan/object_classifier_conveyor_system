#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class OpenCVNodeSubscriber : public rclcpp::Node
{
  public:
    OpenCVNodeSubscriber()
    : Node("opencv_subscriber")
    {
      // auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      // qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
   
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&OpenCVNodeSubscriber::topic_callback, this, std::placeholders::_1));
    
      // publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      // "cv_image", qos);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    { 
      try{
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("Camera", frame);
        cv::waitKey(10);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      }
      
      // // Convert ROS Image to CV Image
      // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv::Mat image_raw =  cv_ptr->image;

      // // Image processing
      // cv::Mat cv_image = image_processing(image_raw);

      // // Convert OpenCV Image to ROS Image
      // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);
      // sensor_msgs::msg::Image out_image; // >> message to be sent
      // img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

      // // Publish the data
      // publisher_ -> publish(out_image);

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCVNodeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
