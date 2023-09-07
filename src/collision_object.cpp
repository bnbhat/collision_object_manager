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
    pose.position.x = m_arm_points[0].x;
    pose.position.y = m_arm_points[0].y;
    pose.position.z = m_arm_points[0].z;

    Point3d direction;
    direction.x = m_arm_points[1].x - m_arm_points[0].x;
    direction.y = m_arm_points[1].y - m_arm_points[0].y;
    direction.z = m_arm_points[1].z - m_arm_points[0].z;

    if(!m_length){
        m_length = this->get_length();
    }
    
    direction.x /= m_length;
    direction.y /= m_length;
    direction.z /= m_length;   

    Point3d reference;
    reference.x = 0.0f; reference.y = 0.0f; reference.z = 0.0f;

    Point3d rotation_axis;
    rotation_axis.x = reference.y * direction.z - reference.z * direction.y;
    rotation_axis.y = reference.z * direction.x - reference.x * direction.z;
    rotation_axis.z = reference.x * direction.y - reference.y * direction.x;

    double dotProduct = reference.x * direction.x + reference.y * direction.y + reference.z * direction.z;
    double angle = acos(dotProduct);

    // Quaternion from axis-angle representation
    pose.orientation.w = cos(angle / 2);
    pose.orientation.x = rotation_axis.x * sin(angle / 2);
    pose.orientation.y = rotation_axis.y * sin(angle / 2);
    pose.orientation.z = rotation_axis.z * sin(angle / 2);

    return pose;
}
