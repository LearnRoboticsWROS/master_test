
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // Inizializza ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_interface");

    // Logger
    auto logger = rclcpp::get_logger("move_group_interface");

    // Spinner con più thread per evitare blocchi
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Attendere l'inizializzazione
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Creare l'interfaccia MoveIt2
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm"); 

    // Imposta il frame di riferimento
    move_group.setPoseReferenceFrame("base_link");

    // Creazione della posa target
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 2, -tau / 4, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = -0.25;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.3;

    // Imposta il target per il link `tool0`
    move_group.setPoseTarget(target_pose, "picking_point");

    // Pianificazione
    MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

    // Esecuzione del piano se la pianificazione ha successo
    if (success)
    {
        move_group.move();
        RCLCPP_INFO(logger, "Motion execution completed.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Motion planning failed!");
    }

    // Arresta lo spinner e chiude ROS2
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
