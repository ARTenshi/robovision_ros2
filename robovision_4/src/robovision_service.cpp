#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "robovision_interfaces/srv/get_point_center.hpp"
#include "robovision_interfaces/msg/object_centroid.hpp"

#include <string>
#include <vector>


class PointCloudCentroidNode : public rclcpp::Node
{
public:
    PointCloudCentroidNode() : Node("point_cloud_service"), is_ptcld_(false), display_(true)
    {
        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_rect_color", 10,
            std::bind(&PointCloudCentroidNode::callback_rgb_rect, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth_registered/image", 10,
            std::bind(&PointCloudCentroidNode::callback_depth_rect, this, std::placeholders::_1));

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10,
            std::bind(&PointCloudCentroidNode::callback_point_cloud, this, std::placeholders::_1));

        // Services
        get_point_center_service_ = this->create_service<robovision_interfaces::srv::GetPointCenter>(
            "get_point_center", 
            std::bind(&PointCloudCentroidNode::point_cloud_processing, this, 
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Starting point_cloud_service application in cpp...");
    }

private:

    bool is_ptcld_;
    bool display_;

    cv::Mat rgb_, depth_, depth_mat_, point_cloud_;
    
    geometry_msgs::msg::Pose centroid_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    
    rclcpp::Service<robovision_interfaces::srv::GetPointCenter>::SharedPtr get_point_center_service_;

    void callback_rgb_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr bridge_rgb = cv_bridge::toCvCopy(msg, msg->encoding);
            rgb_ = bridge_rgb->image.clone();
            cv::cvtColor(rgb_, rgb_, cv::COLOR_RGB2BGR);
        }
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
        }
    }

    void callback_depth_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr bridge_depth = cv_bridge::toCvCopy(msg, "32FC1");
            depth_ = bridge_depth->image.clone();

            depth_mat_ = depth_.clone();
            depth_.convertTo(depth_mat_, CV_32F);
            cv::normalize(depth_mat_, depth_, 0, 1, cv::NORM_MINMAX);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing Depth image: %s", e.what());
        }
    }

    void callback_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
        try 
        {
            // Ensure the point cloud is organized
            if (msg->height > 1)
            {
                // Convert PointCloud2 to cv::Mat
                point_cloud_ = cv::Mat(msg->height, msg->width, CV_32FC4);
                const uint8_t *data_ptr = msg->data.data();

                for (size_t i = 0; i < msg->height; ++i) 
                {
                    for (size_t j = 0; j < msg->width; ++j) 
                    {
                        const float *point = reinterpret_cast<const float*>(data_ptr + (i * msg->row_step + j * msg->point_step));
                        point_cloud_.at<cv::Vec4f>(i, j) = cv::Vec4f(point[0], point[1], point[2], point[3]);
                    }
                }
                
                is_ptcld_ = true;
                RCLCPP_INFO(this->get_logger(), "Point cloud received with size: %d x %d", msg->height, msg->width);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Point cloud is not organized. Cannot convert to cv::Mat.");
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing PointCloud: %s", e.what());
        }
    }

    void point_cloud_processing(
        const std::shared_ptr<robovision_interfaces::srv::GetPointCenter::Request> request,
        std::shared_ptr<robovision_interfaces::srv::GetPointCenter::Response> response) 
    {
        response->point.x = 0.0;
        response->point.y = 0.0;
        response->point.z = 0.0;
        response->point.centroid = {0.0, 0.0, 0.0};

        if (!is_ptcld_) 
        {
            RCLCPP_ERROR(this->get_logger(), "No PointCloud data available for processing.");
            return;
        }

        try 
        {
            int row_id = request->y;
            int col_id = request->x;

            if (row_id >= 0 && row_id < point_cloud_.rows && col_id >= 0 && col_id < point_cloud_.cols) 
            {
                cv::Vec4f point = point_cloud_.at<cv::Vec4f>(row_id, col_id);

                response->point.x = point[0];
                response->point.y = point[1];
                response->point.z = point[2];
                response->point.centroid = {point[0], point[1], point[2]};

                RCLCPP_INFO(this->get_logger(), "Centroid at (%d, %d): [%f, %f, %f]",
                            row_id, col_id, point[0], point[1], point[2]);
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Requested coordinates are out of bounds.");
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error in processing PointCloud: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCentroidNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
