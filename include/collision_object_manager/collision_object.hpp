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
#include <iostream>
#include <vector>
#include <cmath>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "collision_object_manager/custom_data_objects.hpp"


class CollisionObject{
    protected:
        std::string m_name;
        geometry_msgs::msg::Pose m_pose;
        shape_msgs::msg::SolidPrimitive m_shape;

    public:
        CollisionObject();
        CollisionObject(const geometry_msgs::msg::Pose& pose, const shape_msgs::msg::SolidPrimitive& shape);
        CollisionObject(const std::string& name, const geometry_msgs::msg::Pose& pose, const shape_msgs::msg::SolidPrimitive& shape);
        ~CollisionObject();
        void set_pose(const geometry_msgs::msg::Pose& pose);
        geometry_msgs::msg::Pose get_pose();
        shape_msgs::msg::SolidPrimitive get_shape();
        void add();
        void remove();
        void update(const geometry_msgs::msg::Pose& pose);
};

class ArmObject : public CollisionObject{
    public:
        ArmObject(const float& length, const float& diameter);
        ArmObject(const std::string& name, const float& length, const float& diameter);
        ArmObject(const std::vector<Point3d>& arm_points, float diameter=0.15f);
        ArmObject(const std::string name, const std::vector<Point3d>& arm_points, float diameter=0.15f);
        float get_length();
        geometry_msgs::msg::Pose get_pose();
    
    private:
        float m_diameter;
        std::vector<Point3d> m_arm_points;
        float m_length;
};