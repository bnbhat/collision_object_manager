#include <gtest/gtest.h>
#include "collision_object_manager/human_skeleton.hpp"

TEST(HumanSkeletonTest, TestBody18Skeleton) {
    ArcHumanPose pose;
    pose.type = static_cast<int>(HUMAN_SKELETON_TYPE::BODY_18);
    // ... set other fields of pose ...

    auto skeleton = SkeletonFactory::get_skeleton(pose);
    auto right_arm_points = skeleton->get_right_arm_points();
    auto left_arm_points = skeleton->get_left_arm_points();

    // Check the correctness of the retrieved points.
    ASSERT_EQ(right_arm_points.size(), static_cast<size_t>(2));
    ASSERT_EQ(left_arm_points.size(), static_cast<size_t>(2));
}

TEST(HumanSkeletonTest, TestBody34Skeleton) {
    ArcHumanPose pose;
    pose.type = static_cast<int>(HUMAN_SKELETON_TYPE::BODY_34);
    // ... set other fields of pose ...

    auto skeleton = SkeletonFactory::get_skeleton(pose);
    auto right_arm_points = skeleton->get_right_arm_points();
    auto left_arm_points = skeleton->get_left_arm_points();

    // Check the correctness of the retrieved points.
    ASSERT_EQ(right_arm_points.size(), static_cast<size_t>(2));
    ASSERT_EQ(left_arm_points.size(), static_cast<size_t>(2));
}

// Similarly, you can add tests for Body38 and other functionalities.

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
