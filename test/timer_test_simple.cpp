#include <iostream>
#include "collision_object_manager/ros_timer.hpp"

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<CustomTimer> timer;

void SetUp() {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
    timer = std::make_shared<CustomTimer>(node);
}

void TearDown() {
    timer.reset();
    node.reset();
    rclcpp::shutdown();
}

void TimerInitialization() {
    SetUp();
    if (!timer->is_running()) {
        std::cout << "TimerInitialization passed." << std::endl;
    } else {
        std::cout << "TimerInitialization failed." << std::endl;
    }
    TearDown();
}

void SetTimer() {
    SetUp();
    timer->set_timer(5);
    if (timer->get_time() == 5) {
        std::cout << "SetTimer passed." << std::endl;
    } else {
        std::cout << "SetTimer failed." << std::endl;
    }
    TearDown();
}

void SetTimerWithCallback() {
    SetUp();
    bool callback_called = false;
    timer->set_timer(2, [&]() { callback_called = true; });
    timer->start_timer();
    rclcpp::spin_some(node);
    if (callback_called) {
        std::cout << "SetTimerWithCallback passed." << std::endl;
    } else {
        std::cout << "SetTimerWithCallback failed." << std::endl;
    }
    TearDown();
}

void StartTimer() {
    SetUp();
    timer->set_timer(3);
    timer->start_timer();
    if (timer->is_running()) {
        std::cout << "StartTimer passed." << std::endl;
    } else {
        std::cout << "StartTimer failed." << std::endl;
    }
    TearDown();
}

void StopTimer() {
    SetUp();
    timer->set_timer(4);
    timer->start_timer();
    timer->stop_timer();
    if (!timer->is_running()) {
        std::cout << "StopTimer passed." << std::endl;
    } else {
        std::cout << "StopTimer failed." << std::endl;
    }
    TearDown();
}

void ResetTimer() {
    SetUp();
    timer->set_timer(5);
    timer->start_timer();
    timer->reset_timer();
    if (timer->is_running()) {
        std::cout << "ResetTimer passed." << std::endl;
    } else {
        std::cout << "ResetTimer failed." << std::endl;
    }
    TearDown();
}

void CallbackExecution() {
    SetUp();
    bool callback_called = false;
    timer->set_callback([&]() { callback_called = true; });
    timer->set_timer(1);
    timer->start_timer();
    rclcpp::spin_some(node);
    if (callback_called) {
        std::cout << "CallbackExecution passed." << std::endl;
    } else {
        std::cout << "CallbackExecution failed." << std::endl;
    }
    TearDown();
}

int main() {
    TimerInitialization();
    SetTimer();
    SetTimerWithCallback();
    StartTimer();
    StopTimer();
    ResetTimer();
    CallbackExecution();
    return 0;
}
