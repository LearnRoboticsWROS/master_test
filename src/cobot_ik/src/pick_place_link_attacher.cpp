#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

//#include "ros2_linkattacher/gazebo_link_attacher.hpp"
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>



const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "arm"),
          gripper(node, "gripper"),
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),

          // link attacher
          node_(node)
    {
        // Inizializza il gruppo di movimento
        move_group.setPoseReferenceFrame("base_link");
        // attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/link_attacher_node/attach");
        // detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/link_attacher_node/detach");

        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.01);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.0);
        gripper.move();
    }

    

    // void pick()
    // {
    //     // Creazione della posa target per il pick
    //     geometry_msgs::msg::Pose pick_pose;
    //     tf2::Quaternion orientation;
        
    //     orientation.setRPY(-tau/4, 0.225, 0);
    //     pick_pose.orientation = tf2::toMsg(orientation);
    //     pick_pose.position.x = 0.6;
    //     pick_pose.position.y = 0.0;
    //     pick_pose.position.z = 0.3;

    //     move_group.setPoseTarget(pick_pose, "picking_point");

    //     // Pianificazione
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //     RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

    //     // Esecuzione del piano se la pianificazione ha successo
    //     if (success)
    //     {
    //         move_group.move();
    //         RCLCPP_INFO(logger, "Motion execution completed.");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(logger, "Motion planning failed!");
    //     }
    // }

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
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Esecuzione del piano se la pianificazione ha successo
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");

            
            close_gripper();
            rclcpp::sleep_for(std::chrono::seconds(1));

            
            attachObject();
            rclcpp::sleep_for(std::chrono::seconds(1));

            
            RCLCPP_INFO(logger, "Sollevamento post-pick...");
            geometry_msgs::msg::Pose post_pick_pose = pick_pose;
            post_pick_pose.position.z += 0.1; // Alza di 10 cm
            move_group.setPoseTarget(post_pick_pose, "picking_point");

            if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "Sollevamento post-pick fallito!");
                return;
            }

            RCLCPP_INFO(logger, "Post-pick completato, pronto per il place!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
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

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "link6"; // Nome del link del gripper
        request->model2_name = "object_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_0";    // Nome del link dell'oggetto

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "link6"; // Nome del link del gripper
        request->model2_name = "object_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_0";    // Nome del link dell'oggetto

        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
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
        touch_links.push_back("link6");

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

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
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
    //pick_and_place.close_gripper();
    //rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.attachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    //pick_and_place.attachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Esequi il place
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(1));
    // Open gripper
    pick_and_place.open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.detachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    pick_and_place.detachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));


    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}