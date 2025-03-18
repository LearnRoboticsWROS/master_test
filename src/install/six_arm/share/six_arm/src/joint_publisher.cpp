#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_trajectory_controller/joint_trajectory", 10);
        timer_ = this-> create_wall_timer(std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names.push_back("base_link__link1");
        msg.joint_names.push_back("link1__link2");
        msg.joint_names.push_back("link2__link3");
        msg.joint_names.push_back("link3__link4");
        msg.joint_names.push_back("link4__link5");
        msg.joint_names.push_back("link5__link6");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1 = 0.0;
        double position2 = 1.57*(1-cos(count_*0.1));
        double position3 = -position2;
        double position4 = 0.0;
        double position5 = 0.0;
        double position6 = 0.0;
        point.positions.push_back(position1);
        point.positions.push_back(position2);
        point.positions.push_back(position3);
        point.positions.push_back(position4);
        point.positions.push_back(position5);
        point.positions.push_back(position6);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        msg.points.push_back(point);
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", position1, position2);
        count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}