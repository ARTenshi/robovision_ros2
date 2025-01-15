#include "rclcpp/rclcpp.hpp"

#include "robovision_interfaces/srv/get_point_center.hpp"
#include "robovision_interfaces/msg/object_centroid.hpp"

#include <string>


class PointCloudCentroidNode : public rclcpp::Node
{
public:
    PointCloudCentroidNode() : Node("point_cloud_client"), point_({0.0, 0.0, 0.0})
    {
        //Declare parameters
        this->declare_parameter<int>("x", 320);
        this->declare_parameter<int>("y", 240);

        // Get parameter values
        x_ = this->get_parameter("x").as_int();
        y_ = this->get_parameter("y").as_int();

        point_.clear();

        // Processing timer
        //We call our service once each 2.5 seconds
        client_call_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2500),
            std::bind(&PointCloudCentroidNode::client_caller, this));

        RCLCPP_INFO(this->get_logger(), "Starting point_cloud_client application in cpp...");
    }

private:

    int x_, y_;
    std::vector<double> point_;

    rclcpp::TimerBase::SharedPtr client_call_timer_;
    std::vector<std::thread> threads_;

    void client_caller() 
    {
        try
        {
            get_point_center(x_, y_);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }

        // Do something with the result
        if (!point_.empty()) 
        {
            RCLCPP_INFO(this->get_logger(),
                        "(%d, %d) position is: [%f, %f, %f]",
                        x_, y_, point_[0], point_[1], point_[2]);

            // Clear point data
            point_.clear();
        }
    }

    void get_point_center(int x, int y)
    {
        threads_.push_back(std::thread(std::bind(&PointCloudCentroidNode::call_get_point_center_server, this, x, y)));
    }

    void call_get_point_center_server(int x, int y) 
    {
        auto client = this->create_client<robovision_interfaces::srv::GetPointCenter>("get_point_center");
        while (!client->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_point_center service...");
        }

        auto request = std::make_shared<robovision_interfaces::srv::GetPointCenter::Request>();
        request->x = x;
        request->y = y;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(),
                        "(%d, %d) position is: [%f, %f, %f]",
                        x, y, response->point.centroid[0], response->point.centroid[1], response->point.centroid[2]);

            point_ = response->point.centroid;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
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
