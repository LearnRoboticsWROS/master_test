#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "arm"),

          gripper(node, "gripper"),
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace"))
    {
        // Inizializza il gruppo di movimento
        move_group.setPoseReferenceFrame("base_link");
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.04);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.0);
        gripper.move();
    }

    

    void pick()
    {
        // Creazione della posa target per il pick
        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        
        orientation.setRPY(-tau/4, 0.225, 0);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.6;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 0.3;

        move_group.setPoseTarget(pick_pose, "picking_point");

        // Pianificazione
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
    }

    void place()
    {
        // Creazione della posa target per il pick
        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = -0.5;
        place_pose.position.y = -0.5;
        place_pose.position.z = 0.5;

        move_group.setPoseTarget(place_pose, "picking_point");

        // Pianificazione
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
    }

    void addCollisionObjects()
    {
        // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
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

        // Aggiungi gli oggetti alla scena
        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(logger, "Collision objects added to the planning scene.");
    }

    void attachCollisionObject()
    {
        //moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "picking_point";
        attached_object.object = collision_objects[3];  

        attached_object.object.operation = attached_object.object.ADD;

        std::vector<std::string> touch_links;
        // touch_links.push_back("picking_point");
        // attached_object.touch_links = touch_links;

        touch_links.push_back("finger_left");
        touch_links.push_back("finger_right");
        touch_links.push_back("picking_point");
        attached_object.touch_links = touch_links;

        planning_scene_interface.applyAttachedCollisionObject(attached_object);
    }

    void detachCollisionObject()
    {
        // Specify the link to which the object is currently attached
        attached_object.link_name = "picking_point";
        // Define the operation as removing the attachment
        attached_object.object.operation = attached_object.object.REMOVE;
        // Apply the detachment operation to the planning scene
        planning_scene_interface.applyAttachedCollisionObject(attached_object);
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    rclcpp::Logger logger;
};

int main(int argc, char **argv)
{
    // Inizializza ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");

    // Crea l'oggetto PickAndPlace
    PickAndPlace pick_and_place(node);

    // Aggiungi oggetti di collisione
    pick_and_place.addCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Esegui il pick
    pick_and_place.pick();

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Close the gripper
    pick_and_place.close_gripper();
    rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.attachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));
    // Esequi il place
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(1));
    // Open gripper
    pick_and_place.open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.detachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}



