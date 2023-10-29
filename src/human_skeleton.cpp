#include "collision_object_manager/human_skeleton.hpp"


std::unique_ptr<Skeleton> SkeletonFactory::get_skeleton(const ArcHumanPose &pose)
{   
    switch (static_cast<HUMAN_SKELETON_TYPE>(pose.type)) {
            case HUMAN_SKELETON_TYPE::BODY_18:
                return std::make_unique<Body18>(pose);
            case HUMAN_SKELETON_TYPE::BODY_34:
                return std::make_unique<Body34>(pose);
            case HUMAN_SKELETON_TYPE::BODY_38:
                return std::make_unique<Body38>(pose);
            default:
                throw std::invalid_argument("Invalid skeleton type");
        }
}

Skeleton::Skeleton(const ArcHumanPose &pose)
{   
    m_pose = PoseMsg();
    m_pose.key_points = pose.key_points;
    m_pose.type = static_cast<HUMAN_SKELETON_TYPE>(pose.type);
}

template <typename EnumType>
Point3d Skeleton::get_point_of(EnumType enum_value) {
    int index = static_cast<int>(enum_value);

    if (index >= 0 && index < static_cast<int>(m_pose.key_points.size())) {
        return this->m_pose.key_points[index];
    } else {
        throw std::out_of_range("Invalid enum value provided.");
    }
}

Body18::Body18(const ArcHumanPose &pose)
: Skeleton(pose)
{
}

Body34::Body34(const ArcHumanPose &pose)
: Skeleton(pose)
{
}

Body38::Body38(const ArcHumanPose &pose)
: Skeleton(pose)
{
}

std::vector<Point3d> Body18::get_right_arm_points()
{   
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_18::RIGHT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_18::RIGHT_ELBOW));
    return arm_points;
}

std::vector<Point3d> Body34::get_right_arm_points()
{
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_34::RIGHT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_34::RIGHT_ELBOW));
    return arm_points;
}

std::vector<Point3d> Body38::get_right_arm_points()
{
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_38::RIGHT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_38::RIGHT_ELBOW));
    return arm_points;
}

std::vector<Point3d> Body18::get_left_arm_points()
{   
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_18::LEFT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_18::LEFT_ELBOW));

    return arm_points;
}

std::vector<Point3d> Body34::get_left_arm_points()
{
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_34::LEFT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_34::LEFT_ELBOW));
    return arm_points;
}

std::vector<Point3d> Body38::get_left_arm_points()
{
    std::vector<Point3d> arm_points;
    arm_points.emplace_back(get_point_of(BODY_38::LEFT_WRIST));
    arm_points.emplace_back(get_point_of(BODY_38::LEFT_ELBOW));
    return arm_points;
}