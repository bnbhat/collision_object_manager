#include <gtest/gtest.h>
#include "collision_object_manager/collision_object.hpp"
#include "geometry_msgs/msg/pose.hpp"

TEST(CollisionObjectTestSuite, testGetPose)
{
    geometry_msgs::msg::Pose expected_pose;
    expected_pose.position.x = 1.0;
    expected_pose.position.y = 2.0;
    expected_pose.position.z = 3.0;
    
    // Assuming your quaternion conversion is correct, you'd fill the expected orientation here
    // For simplicity, we'll use dummy values:
    expected_pose.orientation.w = 1.0;
    expected_pose.orientation.x = 0.0;
    expected_pose.orientation.y = 0.0;
    expected_pose.orientation.z = 0.0;
    
    CollisionObject obj(expected_pose, /* ... shape ... */);
    geometry_msgs::msg::Pose result_pose = obj.get_pose();

    EXPECT_FLOAT_EQ(expected_pose.position.x, result_pose.position.x);
    EXPECT_FLOAT_EQ(expected_pose.position.y, result_pose.position.y);
    EXPECT_FLOAT_EQ(expected_pose.position.z, result_pose.position.z);
    EXPECT_FLOAT_EQ(expected_pose.orientation.w, result_pose.orientation.w);
    EXPECT_FLOAT_EQ(expected_pose.orientation.x, result_pose.orientation.x);
    EXPECT_FLOAT_EQ(expected_pose.orientation.y, result_pose.orientation.y);
    EXPECT_FLOAT_EQ(expected_pose.orientation.z, result_pose.orientation.z);
}

TEST(ArmObjectTestSuite, testArmObjectPoseFromPoints)
{
    std::vector<Point3d> points;
    Point3d point1, point2;
    point1.x = 0.0;
    point1.y = 0.0;
    point1.z = 0.0;

    point2.x = 1.0;
    point2.y = 0.0;
    point2.z = 0.0;

    points.push_back(point1);
    points.push_back(point2);

    float diameter = 0.5; // example diameter
    ArmObject arm_obj(points, diameter);

    geometry_msgs::msg::Pose result_pose = arm_obj.get_pose();

    // Checking if the pose position matches the first point
    EXPECT_FLOAT_EQ(point1.x, result_pose.position.x);
    EXPECT_FLOAT_EQ(point1.y, result_pose.position.y);
    EXPECT_FLOAT_EQ(point1.z, result_pose.position.z);

    // ... You may also want to test orientation values, but that requires more elaborate checks or dummy values ...

    // Additionally, you could check the shape and dimensions if needed.
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
