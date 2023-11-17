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
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.5;
    primitive.dimensions[primitive.BOX_Z] = 0.4;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.8;
    box_pose.position.z = 0.25;

    // Add collision object
    collision_object_manager->addRTCollisionObject("box1", primitive);

    std::map<std::string, geometry_msgs::msg::Pose> collision_object_poses;
    collision_object_poses["box1"] = box_pose;
    //collision_object_manager->updateRTCollisionObject(collision_object_poses);
    collision_object_manager->moveRTCollisionObject(collision_object_poses);

    // Loop to continuously move the collision object
    bool path = true;

    std::cout<< "running....!" << std::endl;
    while (rclcpp::ok())
    {   
        //std::cout << std::string(40,' ') << "\r";
        //std::string out;
        if (path)
        {
            //box_pose.position.x += 0.01;
            box_pose.position.y += 0.005;
            //box_pose.position.z += 0.01;
            //std::cout <<"x : " << box_pose.position.x << "  |  y : " << box_pose.position.y << "  |  z : " << box_pose.position.z << "\r";
            //out = "x : " + std::to_string(box_pose.position.x) + "  |  y : " + std::to_string(box_pose.position.y) + "  |  z : " + std::to_string(box_pose.position.z);
            //std::cout << "\r" << std::string(out.length(), ' ') << "\r" << out << std::flush;
            collision_object_poses["box1"] = box_pose;
            collision_object_manager->moveRTCollisionObject(collision_object_poses);
        }
        else
        {
            //box_pose.position.x -= 0.01;
            box_pose.position.y -= 0.005;
            //box_pose.position.z -= 0.01;
            //std::cout << "x : " << box_pose.position.x << "  |  y : " << box_pose.position.y << "  |  z : " << box_pose.position.z << "\r";
            //out = "x : " + std::to_string(box_pose.position.x) + "  |  y : " + std::to_string(box_pose.position.y) + "  |  z : " + std::to_string(box_pose.position.z);
            //std::cout << "\r" << std::string(out.length(), ' ') << "\r" << out << std::flush;
            collision_object_poses["box1"] = box_pose;
            collision_object_manager->moveRTCollisionObject(collision_object_poses);
        }

        if(box_pose.position.y >= 0.9){
            path = false;
        }
        else if(box_pose.position.y <= 0.65){
            path = true;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::shutdown();
    return 0;
}
