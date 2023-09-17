/**
 * @file ros_timer.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief Basic timer with a callback
 * @version 1.0
 * @date 2023-09-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

class CustomTimer {
    public:
        CustomTimer(std::shared_ptr<rclcpp::Node> node);
        void set_timer(int time_in_sec);
        void set_timer(int time_in_sec, std::function<void()> callback);
        void set_callback(std::function<void()> callback);
        void reset_callback();
        void reset_timer();
        void stop_timer();
        void start_timer();
        bool is_running() const;
        int get_time() const;

    private:
        std::shared_ptr<rclcpp::Node> m_node;
        rclcpp::TimerBase::SharedPtr m_timer;
        std::function<void()> m_callback;
        int m_time_in_sec;
        bool m_is_running;
};