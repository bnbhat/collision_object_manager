#include "collision_object_manager/collision_object.hpp"

CollisionObject::CollisionObject()
{
}

CollisionObject::CollisionObject(const geometry_msgs::msg::Pose& pose, const shape_msgs::msg::SolidPrimitive& shape)
: m_pose(pose), m_shape(shape)
{
}

CollisionObject::CollisionObject(const std::string& name, const geometry_msgs::msg::Pose& pose, const shape_msgs::msg::SolidPrimitive& shape)
: CollisionObject(pose, shape)
{
    m_name = name;
}

CollisionObject::~CollisionObject(){}

void CollisionObject::set_pose(const geometry_msgs::msg::Pose& pose){m_pose = pose;}

geometry_msgs::msg::Pose CollisionObject::get_pose(){ return m_pose;}

shape_msgs::msg::SolidPrimitive CollisionObject::get_shape() {return m_shape;}

// TODO
void CollisionObject::add(){}
void CollisionObject::remove(){}
void CollisionObject::update(const geometry_msgs::msg::Pose& pose){}

ArmObject::ArmObject(const float& length, const float& diameter)
: CollisionObject(), m_length(length), m_diameter(diameter)
{   
    m_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    m_shape.dimensions.resize(3);
    m_shape.dimensions[m_shape.CYLINDER_HEIGHT] = m_length;
    m_shape.dimensions[m_shape.CYLINDER_RADIUS] = m_diameter / 2;
    m_pose = geometry_msgs::msg::Pose();
}

ArmObject::ArmObject(const std::string& name, const float& length, const float& diameter)
: ArmObject(length, diameter)
{   
    m_name = name;
}

ArmObject::ArmObject(const std::vector<Point3d> &arm_points, float diameter)
    : CollisionObject(), m_arm_points(arm_points), m_diameter(diameter)
{   
    if(arm_points.size() != 2 )
    {
        throw std::runtime_error("The arm_points list must contain two values.");
    }
    m_length = this->get_length();
    m_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    m_shape.dimensions.resize(3);
    m_shape.dimensions[m_shape.CYLINDER_HEIGHT] = m_length;
    m_shape.dimensions[m_shape.CYLINDER_RADIUS] = m_diameter / 2;
    m_pose = this->get_pose();
}

ArmObject::ArmObject(const std::string name, const std::vector<Point3d> &arm_points, float diameter)
: ArmObject(arm_points, diameter)
{
    m_name = name;
}

float ArmObject::get_length()
{
    return std::sqrt(std::pow(m_arm_points[0].x - m_arm_points[1].x, 2) + 
                    std::pow(m_arm_points[0].y - m_arm_points[1].y, 2) + 
                    std::pow(m_arm_points[0].z - m_arm_points[1].z, 2));
}

geometry_msgs::msg::Pose ArmObject::get_pose()
{   
    geometry_msgs::msg::Pose pose;
    pose.position.x = (m_arm_points[0].x + m_arm_points[1].x) / 2.0;
    pose.position.y = (m_arm_points[0].y + m_arm_points[1].y) / 2.0;
    pose.position.z = (m_arm_points[0].z + m_arm_points[1].z) / 2.0;

    Point3d direction;
    direction.x = m_arm_points[1].x - m_arm_points[0].x;
    direction.y = m_arm_points[1].y - m_arm_points[0].y;
    direction.z = m_arm_points[1].z - m_arm_points[0].z;

    float length = std::sqrt(std::pow(m_arm_points[0].x - m_arm_points[1].x, 2) + 
                    std::pow(m_arm_points[0].y - m_arm_points[1].y, 2) + 
                    std::pow(m_arm_points[0].z - m_arm_points[1].z, 2));

    direction.x /= length;
    direction.y /= length;
    direction.z /= length;   

    // Calculate pitch and yaw from the direction vector
    double pitch = std::atan2(std::sqrt(direction.x * direction.x + direction.y * direction.y), direction.z);
    double yaw = std::atan2(direction.y, direction.x);
    double roll = 0;  // Assuming roll is 0 for a direction vector

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    // Convert to quaternion
    pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.orientation.z = cr * cp * sy - sr * sp * cy;

    return pose;
}
