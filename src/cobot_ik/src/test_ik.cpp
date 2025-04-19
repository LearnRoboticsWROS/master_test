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


    // add
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Imposta il frame di riferimento
    move_group.setPoseReferenceFrame("base_link");

    // Creazione della posa target
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.5;

    // Imposta il target per il link `tool0`
    move_group.setPoseTarget(target_pose, "picking_point");


    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Primo oggetto: Tavolo 1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "world";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {0.608, 2.0, 1.0};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.75;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.5;
    collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Secondo oggetto: Tavolo 2
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "world";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {1.3, 0.8, 1.0};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.0;
    collision_objects[1].primitive_poses[0].position.y = 1;
    collision_objects[1].primitive_poses[0].position.z = 0.5;
    collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Terzo oggetto: Base Cilindrica
    collision_objects[2].id = "basement";
    collision_objects[2].header.frame_id = "world";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    collision_objects[2].primitives[0].dimensions = {0.8, 0.2}; // {altezza, raggio}
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.0;
    collision_objects[2].primitive_poses[0].position.y = 0.0;
    collision_objects[2].primitive_poses[0].position.z = 0.4;
    collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Quarto oggetto: Oggetto piccolo
    collision_objects[3].id = "object";
    collision_objects[3].header.frame_id = "world";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[3].primitives[0].dimensions = {0.02, 0.02, 0.2};
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.6;
    collision_objects[3].primitive_poses[0].position.y = 0.0;
    collision_objects[3].primitive_poses[0].position.z = 1.1;
    collision_objects[3].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Aggiunta degli oggetti alla scena
    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(logger, "Collision objects added to the planning scene.");

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




















