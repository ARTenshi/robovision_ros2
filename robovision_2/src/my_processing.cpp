#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode() : Node("image_processing"), is_image_(false)
    {
        counter_ = 0;

        //Subscribers
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageProcessingNode::callback_image, this, std::placeholders::_1));
        
        //Processing
        image_processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ImageProcessingNode::image_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting image_processing application in cpp...");
    }

private:

    bool is_image_;
    int counter_;
    cv::Mat image_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::TimerBase::SharedPtr image_processing_timer_;
    
    void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        is_image_ = true;
    }

    void image_processing()
    {
        if (is_image_){
            if (counter_ == 0)
			    std::cout << "size: rows: " << image_.rows << 
                             ", cols: " << image_.cols << 
                             ", depth: " << image_.channels() << std::endl;

            cv::imshow("view", image_);
            cv::waitKey(1);

            counter_++;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
