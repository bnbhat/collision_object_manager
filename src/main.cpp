#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "collision_object_manager/moveit_pose_interface.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("pose_moveit_interface");
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    std::shared_ptr<CollisionObjectManager> collision_object_manager = std::make_shared<CollisionObjectManager>(node, planning_scene_interface, move_group_interface);

    collision_object_manager->clearAllCollisionObjects();
    std::shared_ptr<PoseMoveItInterface> pose_moveit_interface = std::make_shared<PoseMoveItInterface>(node, collision_object_manager);

    RCLCPP_INFO(node->get_logger(), "Initialized");

    while(rclcpp::ok())
    {
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}
