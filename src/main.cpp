#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "collision_object_manager/collision_object_manager.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>(
		"collision_object_interface",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
	);
	auto collisionObjectManager = std::make_shared<CollisionObjectManager>(node);
	collisionObjectManager->test();
	collisionObjectManager->update();
	//rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
