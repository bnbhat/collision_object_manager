#include "collision_object_manager/collision_object_manager.hpp"

CollisionObjectManager::CollisionObjectManager(const std::shared_ptr<rclcpp::Node> node, std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface, const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface)
    : m_node(node), m_planning_scene_interface(planning_scene_interface), m_move_group_interface(move_group_interface)
{
    m_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(m_node, "robot_description",
                                                                                                "planning_scene_monitor");
    if (!m_planning_scene_monitor->getPlanningScene())
    {
        RCLCPP_ERROR(m_node->get_logger(), "The planning scene was not retrieved!");
        return;
    }
    else
    {
        m_planning_scene_monitor->startStateMonitor();
        m_planning_scene_monitor->providePlanningSceneService();  // let RViz display query PlanningScene
        m_planning_scene_monitor->setPlanningScenePublishingFrequency(100);
        m_planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
        m_planning_scene_monitor->startSceneMonitor();
    }

}

void CollisionObjectManager::addStaticCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose)
{   
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = m_move_group_interface->getPlanningFrame();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    m_static_collision_objects[id] = collision_object;
    this->updatePlanningScene();
}

void CollisionObjectManager::addRTCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive)
{   
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = m_move_group_interface->getPlanningFrame();
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    m_rt_collision_object_prototypes[id] = collision_object;
    //this->updatePlanningScene();
}

void CollisionObjectManager::updateRTCollisionObject(const std::map<std::string, geometry_msgs::msg::Pose>& collision_object_poses)
{   
    std::map<std::string, moveit_msgs::msg::CollisionObject> previous_objects = m_rt_collision_objects;

    m_rt_collision_objects.clear();
    for (auto& object : collision_object_poses)
    {
        moveit_msgs::msg::CollisionObject collision_object = m_rt_collision_object_prototypes[object.first];
        collision_object.id = collision_object.id + this->generateRandomHash();
        collision_object.primitive_poses[0] = object.second;
        collision_object.operation = collision_object.ADD;

        m_rt_collision_objects[collision_object.id] = collision_object;
    }
    this->updatePlanningScene();
    if (!previous_objects.empty())
    {
        for (auto& object : previous_objects)
        {
            object.second.operation = object.second.REMOVE;
            m_rt_collision_objects[object.first] = object.second;
        }
        this->updatePlanningScene();
    }
}

void CollisionObjectManager::addObject(const std::string& id, const geometry_msgs::msg::Pose& pose){
    moveit_msgs::msg::CollisionObject& collision_object = m_rt_collision_object_prototypes[id];
    collision_object.primitive_poses[0] = pose;
    collision_object.operation = collision_object.ADD;
    m_rt_collision_objects[collision_object.id] = collision_object;
}

void CollisionObjectManager::moveObject(const std::string &id, const geometry_msgs::msg::Pose &pose)
{
    moveit_msgs::msg::CollisionObject& collision_object = m_rt_collision_object_prototypes[id];
    collision_object.primitives.clear(); // avoid warning
    collision_object.primitive_poses[0] = pose;
    collision_object.pose = pose;
    collision_object.operation = collision_object.MOVE;
    m_rt_collision_objects[collision_object.id] = collision_object;
}

void CollisionObjectManager::moveRTCollisionObject(const std::map<std::string, geometry_msgs::msg::Pose> &collision_object_poses)
{
    for (auto& object: collision_object_poses){
        if(m_rt_collision_objects.count(object.first) != 0){
            this->moveObject(object.first, object.second);
        }
        else
        {
            this->addObject(object.first, object.second);
        }
    }
    this->updatePlanningScene();
}


void CollisionObjectManager::removeRTCollisionObject(const std::string &id)
{
    if(m_rt_collision_objects.count(id) != 0){
        moveit_msgs::msg::CollisionObject& object = m_rt_collision_objects[id];
        object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    }
    else
    {
        RCLCPP_WARN(m_node->get_logger(), "Collision object with id '%s' does not exist", id.c_str());
    }
}

void CollisionObjectManager::removeCollisionObject(const std::string& id)
{
    if (m_static_collision_objects.count(id) != 0)
    {
        moveit_msgs::msg::CollisionObject& object = m_static_collision_objects[id];
        object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        this->updatePlanningScene();
        m_static_collision_objects.erase(id);
    }
    else if(m_rt_collision_objects.count(id) != 0){
        moveit_msgs::msg::CollisionObject& object = m_rt_collision_objects[id];
        object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        this->updatePlanningScene();
        m_rt_collision_objects.erase(id);
    }
    else
    {
        RCLCPP_WARN(m_node->get_logger(), "Collision object with id '%s' does not exist", id.c_str());
    }
}

void CollisionObjectManager::clearAllCollisionObjects()
{   

    std::map<std::string, moveit_msgs::msg::CollisionObject> all_objects;
    std::vector<std::string> all_object_vector;
    all_objects = m_planning_scene_interface->getObjects();
    for (auto& entry : all_objects)
    {
        //moveit_msgs::msg::CollisionObject& object = entry.second;
        //object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        std::cout<< "removing " << entry.first << std::endl;
        all_object_vector.push_back(entry.first);
    }
    m_planning_scene_interface->removeCollisionObjects(all_object_vector);
    m_static_collision_objects.clear();
    m_rt_collision_objects.clear();
    this->updatePlanningScene();
}

void CollisionObjectManager::clearCollisionObjects(const std::vector<std::string>& ids)
{
    m_planning_scene_interface->removeCollisionObjects(ids);
    m_static_collision_objects.clear();
    m_rt_collision_objects.clear();
    this->updatePlanningScene();
}

bool CollisionObjectManager::hasCollisionObject(const std::string& id) const
{
    return m_static_collision_objects.count(id) != 0 || m_rt_collision_objects.count(id) != 0;
}

std::vector<std::string> CollisionObjectManager::getAllCollisionObjectIds() const
{
    std::vector<std::string> ids;
    for (const auto& entry : m_static_collision_objects)
    {
        ids.emplace_back(entry.first);
    }

    for (const auto& entry : m_rt_collision_objects)
    {
        ids.emplace_back(entry.first);
    }
    return ids;
}

void CollisionObjectManager::updatePlanningScene()
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    for (const auto& entry : m_static_collision_objects)
    {
        collision_objects.emplace_back(entry.second);
    }

    for (const auto& entry : m_rt_collision_objects)
    {
        collision_objects.emplace_back(entry.second);
    }

    {
        planning_scene_monitor::LockedPlanningSceneRW scene(m_planning_scene_monitor);
        for(const moveit_msgs::msg::CollisionObject& object : collision_objects)
        {
            scene->processCollisionObjectMsg(object);
        }
    }
    //m_planning_scene_interface->applyCollisionObjects(collision_objects);
}

std::string CollisionObjectManager::generateRandomHash()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    std::stringstream ss;
    for (int i = 0; i < 8; ++i)
    {
        ss << std::setw(2) << std::setfill('0') << std::hex << dis(gen);
    }

    return ss.str();
}