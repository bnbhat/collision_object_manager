/**
 * @file collision_object_manager.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief Collision object manager to manage the collision obnjects in the MoveIt 2 planning scene
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <random>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.h>

//#include "collision_object.hpp"
/*
class CollisionObjectManager{
	private:
		rclcpp::Node::SharedPtr m_node;

		std::string PLANNING_GROUP = "ur_manipulator";
		std::string frame_base = "base_link";
		std::string frame_tcp = "tool0";

		moveit::planning_interface::MoveGroupInterface *m_move_group_interface;
		moveit::planning_interface::PlanningSceneInterface *m_planning_scene_interface;

		geometry_msgs::msg::Pose m_pose;
		//std::vector<CollisionObject> m_collision_objects;

	public:
		CollisionObjectManager(rclcpp::Node::SharedPtr node);
		~CollisionObjectManager();

		void test();

		void update();


};
*/

//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <geometry_msgs/msg/pose.h>

class CollisionObjectManager
{
public:
    CollisionObjectManager(const std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface, const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface);
    void addStaticCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose);
	void addRTCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive);
    void updateRTCollisionObject(const std::map<std::string, geometry_msgs::msg::Pose>& collision_object_poses);
    void removeCollisionObject(const std::string& id);
	void removeRTCollisionObject(const std::string& id);

    void clearAllCollisionObjects();
    bool hasCollisionObject(const std::string& id) const;
    std::vector<std::string> getAllCollisionObjectIds() const;

private:
    void updatePlanningScene();
	std::string generateRandomHash();

	std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene_interface;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group_interface;
    std::map<std::string, moveit_msgs::msg::CollisionObject> m_static_collision_objects;
	std::map<std::string, moveit_msgs::msg::CollisionObject> m_rt_collision_object_prototypes;
	std::map<std::string, moveit_msgs::msg::CollisionObject> m_rt_collision_objects;
};


