#include "collision_object_manager/mock_gripper.hpp"

MockGripper::MockGripper(std::shared_ptr<rclcpp::Node> node)
: node_(node){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "ur_manipulator");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    source_frame_ = "tool0";
    target_frame_ = "base_link";

    timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MockGripper::run, this));
    std::cout << "initialized" << std::endl;
}

void MockGripper::lookupTransform() {
    try {
        auto transform = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
        RCLCPP_INFO(
            node_->get_logger(),
            "Transform (translation): x=%.2f, y=%.2f, z=%.2f",
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "TF Lookup failed: %s", ex.what());
    }
}

void MockGripper::run() {
    std::cout << "run()" << std::endl;
    lookupTransform();
    try {
        auto transform = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

        geometry_msgs::msg::Pose attach_pose;
        attach_pose.orientation = transform.transform.rotation;
        attach_pose.position.x = transform.transform.translation.x;
        attach_pose.position.y = transform.transform.translation.y;
        attach_pose.position.z = transform.transform.translation.z - 0.08;

        moveit_msgs::msg::CollisionObject object_to_attach;
        object_to_attach.id = "mock_gripper";
        object_to_attach.header.frame_id = target_frame_;

        shape_msgs::msg::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = cylinder_primitive.CYLINDER;
        cylinder_primitive.dimensions.resize(2);
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.16;
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.035;

        object_to_attach.primitives.push_back(cylinder_primitive);
        object_to_attach.primitive_poses.push_back(attach_pose);
        object_to_attach.operation = object_to_attach.ADD;
        planning_scene_interface_->applyCollisionObject(object_to_attach);

        move_group_interface_->attachObject(object_to_attach.id, "wrist_3_link");
        exit(0);

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "TF Lookup failed: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mock_gripper_node");
    auto mock_gripper = std::make_unique<MockGripper>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
