/**
 * @file moveit-pose-interface.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief this file contains the pose interpretor functions 
 * @version 0.1
 * @date 2023-08-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "arc_interfaces/msg/arc_human_pose.hpp"
#include "arc_interfaces/msg/arc_human_pose_pred.hpp"
#include "collision_object_manager/ros_timer.hpp"
#include "collision_object_manager/human_skeleton.hpp"
#include "collision_object_manager/collision_object_manager.hpp"


class PoseMoveItInterface {
    public:
        PoseMoveItInterface(const std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<CollisionObjectManager> collision_object_manager);
        
    private:
        void rt_pose_callback(const ArcHumanPose& pose);
        void pred_pose_callback(const ArcHumanPosePred& pose);
        void process(const ArcHumanPose& pose);
        void validate_pred(const ArcHumanPosePred& pose);
        void watchdog_callback();
        void init_arms(); //temp
        int64_t time_to_nanoseconds(const builtin_interfaces::msg::Time& time);


        std::shared_ptr<rclcpp::Node> m_node;
        std::shared_ptr<CollisionObjectManager> m_collision_object_manager;
        rclcpp::Subscription<ArcHumanPose>::SharedPtr m_rt_pose_subscriber;
        rclcpp::Subscription<ArcHumanPosePred>::SharedPtr m_pred_pose_subscriber;

        mutable ArcHumanPose previous_rt_pose;
        mutable ArcHumanPosePred previous_pred_pose;   

        int allowed_time_delay = 1; // in sec    
        bool is_pred = false; // to use pred?
        mutable bool is_rec_first_msg = false;

        std::unique_ptr<CustomTimer> m_watchdog_timer;

        std::shared_ptr<ArmObject> right_arm;
        std::shared_ptr<ArmObject> left_arm;
};