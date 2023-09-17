#include <gtest/gtest.h>
#include "collision_object_manager/ros_timer.hpp"

class CustomTimerTest : public ::testing::Test 
{
    protected:
        std::shared_ptr<rclcpp::Node> node;
        std::shared_ptr<CustomTimer> timer;
    
        void SetUp() override {
            rclcpp::init(0, nullptr);
            node = std::make_shared<rclcpp::Node>("test_node");
            timer = std::make_shared<CustomTimer>(node);
        }
    
        void TearDown() override {
            timer.reset();
            node.reset();
            rclcpp::shutdown();
        }
};

TEST_F(CustomTimerTest, TimerInitialization) {
    // Ensure the timer is not running upon initialization
    ASSERT_FALSE(timer->is_running());
}

TEST_F(CustomTimerTest, SetTimer) {
    timer->set_timer(5);
    ASSERT_EQ(timer->get_time(), 5);
}

TEST_F(CustomTimerTest, SetTimerWithCallback) {
    bool callback_called = false;
    timer->set_timer(2, [&]() { callback_called = true; });
    timer->start_timer();
    rclcpp::spin_some(node);
    ASSERT_TRUE(callback_called);
}

TEST_F(CustomTimerTest, StartTimer) {
    timer->set_timer(3);
    timer->start_timer();
    ASSERT_TRUE(timer->is_running());
}

TEST_F(CustomTimerTest, StopTimer) {
    timer->set_timer(4);
    timer->start_timer();
    timer->stop_timer();
    ASSERT_FALSE(timer->is_running());
}

TEST_F(CustomTimerTest, ResetTimer) {
    timer->set_timer(5);
    timer->start_timer();
    timer->reset_timer();
    ASSERT_TRUE(timer->is_running());
}

TEST_F(CustomTimerTest, CallbackExecution) {
    bool callback_called = false;
    timer->set_callback([&]() { callback_called = true; });
    timer->set_timer(1);
    timer->start_timer();
    rclcpp::spin_some(node);
    ASSERT_TRUE(callback_called);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
