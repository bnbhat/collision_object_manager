/**
 * @file collision_object.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief Wrapper class for MoveIt 2 collision object
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


class CollisionObject{
    private:
        std::string m_name;
        geometry_msgs::msg::Pose pose;
        //shape_msgs::msg::SolidPrimitivem_shape;

    public:
        CollisionObject();
        CollisionObject(std::string name, geometry_msgs::msg::Pose pose, shape_msgs::msg::SolidPrimitive shape);
        ~CollisionObject();
        void add();
        void remove();
        void update(geometry_msgs::msg::Pose pose);
}