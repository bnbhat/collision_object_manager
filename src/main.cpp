#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.h>
#include "collision_object_manager/collision_object_manager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("collision_object_manager_test");

    // Initialize MoveIt PlanningSceneInterface
    auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");

    // Create CollisionObjectManager instance
    auto collision_object_manager = std::make_shared<CollisionObjectManager>(node, planning_scene_interface, move_group_interface);

    std::cout<< "initialized" << std::endl;

    collision_object_manager->clearAllCollisionObjects();

    // Define the primitive and pose for the collision object
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.8;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    // Add collision object
    collision_object_manager->addRTCollisionObject("box1", primitive);

    std::map<std::string, geometry_msgs::msg::Pose> collision_object_poses;
    collision_object_poses["box1"] = box_pose;
    collision_object_manager->updateRTCollisionObject(collision_object_poses);

    // Loop to continuously move the collision object
    bool path = true;

    while (rclcpp::ok())
    {   
        std::cout<< "in loop" << std::endl;

        if (path)
        {
            box_pose.position.x += 0.01;
            //box_pose.position.y += 0.01;
            //box_pose.position.z += 0.01;
//
            std::cout<< "x : " << box_pose.position.x << "  |  y : " << box_pose.position.y << "  |  z : " << box_pose.position.z << std::endl;
            collision_object_poses["box1"] = box_pose;
            collision_object_manager->updateRTCollisionObject(collision_object_poses);
        }
        else
        {
            box_pose.position.x -= 0.01;
            //box_pose.position.y -= 0.01;
            //box_pose.position.z -= 0.01;

            std::cout<< "x : " << box_pose.position.x << "  |  y : " << box_pose.position.y << "  |  z : " << box_pose.position.z << std::endl;
            collision_object_poses["box1"] = box_pose;
            collision_object_manager->updateRTCollisionObject(collision_object_poses);
        }

        if(box_pose.position.x >= 0.8){
            path = false;
        }
        else if(box_pose.position.x <= 0.5){
            path = true;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
