/**
 * @file custom_data_objects.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#pragma once

#include <geometry_msgs/msg/point.hpp>
#include "arc_interfaces/msg/arc_human_pose.hpp"
#include "arc_interfaces/msg/arc_human_pose_pred.hpp"
#include "collision_object_manager/skeleton_types.hpp"

using ArcHumanPose = arc_interfaces::msg::ArcHumanPose;
using ArcHumanPosePred = arc_interfaces::msg::ArcHumanPosePred;
using Point3d = geometry_msgs::msg::Point;
//using Pose = geometry_msgs::msg::Pose;

struct PoseMsg
{
    std::vector<Point3d> key_points;
    HUMAN_SKELETON_TYPE type;
};

/*
struct Position{ 
    float x;
    float y;
    float z;
};

struct Orientation{
    float x;
    float y;
    float z;
    float w;
};

struct Pose {
    Position position;
    Orientation orientation;
};
*/
/*
class Point3d {
    public:
        Point3d() :x(0), y(0), z(0){}
        Point3d(float x, float y, float z) : x(x), y(y), z(z) {}
        float x;
        float y;
        float z;
};
*/
