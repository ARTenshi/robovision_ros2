#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisherNode : public rclcpp::Node
{
public:
    CameraPublisherNode() : 
        Node("image_publisher"), capture_(-1) // Default invalid camera index
    {
        // Declare and get the camera index parameter (default is 0)
        this->declare_parameter<int>("camera_index", 0);
        int camera_index = this->get_parameter("camera_index").as_int();

        // Open the camera
        capture_.open(camera_index);

        // Check if the camera opened successfully
        if (!capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            throw std::runtime_error("Camera not available");
        }

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        image_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                               std::bind(&CameraPublisherNode::publish_image, this));
        
        RCLCPP_INFO(this->get_logger(), "Starting image_publisher application in cpp...");
    }

private:

    cv::VideoCapture capture_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr image_timer_;

    void publish_image()
    {
        cv::Mat cv_image;
        capture_ >> cv_image; // Capture an image frame

        if (cv_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        //Convert the output data into a ROS message format
        sensor_msgs::msg::Image::SharedPtr image_msg_;
	    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
        image_publisher_->publish(*image_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraPublisherNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
