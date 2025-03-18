#include "rclcpp/rclcpp.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_attach_link");

    auto client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");

    while (!client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the AttachLink service...");
    }

    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    request->model1_name = "cobot";
    request->link1_name = "finger_right";
    request->model2_name = "object_pick";
    request->link2_name = "link_0";

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Object attached successfully.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to attach object.");
    }

    rclcpp::shutdown();
    return 0;
}
