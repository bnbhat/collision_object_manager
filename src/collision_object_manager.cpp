//#include "../include/collision_object_manager/collision_object_manager.hpp"
#include "collision_object_manager/collision_object_manager.hpp"
//#include "collision_object_manager.hpp"

CollisionObjectManager::CollisionObjectManager(rclcpp::Node::SharedPtr node)
{   
    m_node = node;
    m_move_group_interface = new moveit::planning_interface::MoveGroupInterface(m_node, PLANNING_GROUP);
    m_planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
}

CollisionObjectManager::~CollisionObjectManager()
{
}

void CollisionObjectManager::test()
{
    auto const collision_object = [frame_id =
                                m_move_group_interface->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.2;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.25;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    std::vector<moveit_msgs::msg::CollisionObject> collision_obj_vector;
    collision_obj_vector.push_back(collision_object); 
    m_planning_scene_interface->addCollisionObjects(collision_obj_vector);

}

void CollisionObjectManager::update()
{   
    std::vector<std::string> known_object_ids = {"box1"};
    while(rclcpp::ok()){
       std::map< std::string, moveit_msgs::msg::CollisionObject> known_object_map = m_planning_scene_interface->getObjects(known_object_ids);
         moveit_msgs::msg::CollisionObject object = known_object_map.at("box1");
            object.primitive_poses[0].position.x += 0.01;
            object.primitive_poses[0].position.y += 0.01;
            object.primitive_poses[0].position.z += 0.01;
            m_planning_scene_interface->applyCollisionObject(object);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
}
