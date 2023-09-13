/**
 * @file human_skeleton.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief contains human skeleton related functions
 * @version 1.0
 * @date 2023-09-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <vector>
#include <iostream>
#include "collision_object_manager/custom_data_objects.hpp"

class Skeleton {
    public:
        Skeleton(const arc_interfaces::msg::ArcHumanPose& pose);
        //virtual std::vector<Point3d> get_arm_points() = 0;
        virtual std::vector<Point3d> get_right_arm_points() = 0;
        virtual std::vector<Point3d> get_left_arm_points() = 0;
    
    protected:
        PoseMsg m_pose;
        template <typename EnumType>
        Point3d get_point_of(EnumType enum_value);
};

class Body18 : public Skeleton {
    public:
        Body18(const ArcHumanPose& pose);
        std::vector<Point3d> get_right_arm_points() override;
        std::vector<Point3d> get_left_arm_points() override;
};

class Body34 : public Skeleton {
    public:
        Body34(const ArcHumanPose& pose);
        std::vector<Point3d> get_right_arm_points() override;
        std::vector<Point3d> get_left_arm_points() override;
};

class Body38 : public Skeleton {
    public:
        Body38(const ArcHumanPose& pose);
        std::vector<Point3d> get_right_arm_points() override;
        std::vector<Point3d> get_left_arm_points() override;
};

class SkeletonFactory {
    public:
        static std::unique_ptr<Skeleton> get_skeleton(const ArcHumanPose& pose);
};


