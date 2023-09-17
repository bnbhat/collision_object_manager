#include "collision_object_manager/ros_timer.hpp"

CustomTimer::CustomTimer(std::shared_ptr<rclcpp::Node> node)
    : m_node(node), m_time_in_sec(0), m_is_running(false)
{   
    

    // Initialize the timer with a dummy value; it will be set later using the set_timer function
    m_timer = m_node->create_wall_timer(std::chrono::seconds(1), [this]() -> void {
        if (m_callback) {
            m_callback();
        } else {
            RCLCPP_INFO(m_node->get_logger(), "Timer expired! No callback set.");
        }
    });
    m_timer->cancel();  // Cancel the timer immediately since it's just a placeholder
}

void CustomTimer::set_timer(int time_in_sec)
{
    m_time_in_sec = time_in_sec;
    if (m_is_running) {
        m_timer->cancel();
        m_timer = m_node->create_wall_timer(std::chrono::seconds(m_time_in_sec), [this]() -> void {
            m_callback();
        });
    }
}

void CustomTimer::set_timer(int time_in_sec, std::function<void()> callback)
{
    set_callback(callback);
    set_timer(time_in_sec);
}

void CustomTimer::reset_timer()
{
    if (m_is_running) {
        m_timer->reset();
    }
}

void CustomTimer::stop_timer()
{
    m_timer->cancel();
    m_is_running = false;
}

void CustomTimer::start_timer()
{
    if (!m_is_running) {
        m_timer = m_node->create_wall_timer(std::chrono::seconds(m_time_in_sec), [this]() -> void {
            m_callback();
        });
        m_is_running = true;
    }
}

void CustomTimer::set_callback(std::function<void()> callback)
{
    m_callback = callback;
}

void CustomTimer::reset_callback()
{
    m_callback = [this]() -> void { 
        RCLCPP_INFO(m_node->get_logger(), "Timer expired! No callback set."); 
    };
}

bool CustomTimer::is_running() const
{
    return m_is_running;
}

int CustomTimer::get_time() const
{
    return m_time_in_sec;
}