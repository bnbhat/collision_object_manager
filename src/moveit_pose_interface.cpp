#include "collision_object_manager/moveit_pose_interface.hpp"

PoseMoveItInterface::PoseMoveItInterface(const std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<CollisionObjectManager> collision_object_manager)
: m_node(node), m_collision_object_manager(collision_object_manager) {
    m_rt_pose_subscriber = m_node->create_subscription<ArcHumanPose>("/arc_rt_human_pose", 1, std::bind(&PoseMoveItInterface::rt_pose_callback, this, std::placeholders::_1));
    m_pred_pose_subscriber = m_node->create_subscription<ArcHumanPosePred>("/arc_pred_human_pose", 1, std::bind(&PoseMoveItInterface::pred_pose_callback, this, std::placeholders::_1));
    m_watchdog_timer = std::make_unique<CustomTimer>(m_node);
    m_watchdog_timer->set_callback(std::bind(&PoseMoveItInterface::watchdog_callback, this));
    this->init_arms();
}

void PoseMoveItInterface::init_arms()
{   
    float default_length = 0.4;
    float default_diameter = 0.15;
    right_arm = std::make_shared<ArmObject>("right_arm", default_length, default_diameter);
    left_arm = std::make_shared<ArmObject>("left_arm", default_length, default_diameter); 

    m_collision_object_manager->addRTCollisionObject("right_arm", right_arm->get_shape());
    m_collision_object_manager->addRTCollisionObject("left_arm", left_arm->get_shape());
}

int64_t PoseMoveItInterface::time_to_nanoseconds(const builtin_interfaces::msg::Time& time) {
    return static_cast<int64_t>(time.sec) * 1'000'000'000 + time.nanosec;
}


void PoseMoveItInterface::rt_pose_callback(const ArcHumanPose &pose)  
{       
        if(!is_rec_first_msg){
        previous_rt_pose = pose;
        is_rec_first_msg = true;
    }
    else
    {
        int64_t time_diff = time_to_nanoseconds(pose.time_stamp) - time_to_nanoseconds(previous_rt_pose.time_stamp);
        if(time_diff < allowed_time_delay * 1'000'000'000 ){
            process(pose);
            previous_rt_pose = pose;
        }
        else{
            previous_rt_pose = pose;
        }
    }
}

void PoseMoveItInterface::pred_pose_callback(const ArcHumanPosePred &pose)
{
}

void PoseMoveItInterface::process(const ArcHumanPose &pose)
{   
    m_watchdog_timer->reset_timer();
    m_watchdog_timer->set_timer(allowed_time_delay);
    std::unique_ptr<Skeleton> skeleton = SkeletonFactory::get_skeleton(pose);
    std::shared_ptr<ArmObject> rt_right_arm =  std::make_shared<ArmObject>(skeleton->get_right_arm_points());
    std::shared_ptr<ArmObject> rt_left_arm =  std::make_shared<ArmObject>(skeleton->get_left_arm_points());
    std::map<std::string, geometry_msgs::msg::Pose> pose_map;
    pose_map["right_arm"] = rt_right_arm->get_pose();
    pose_map["left_arm"] = rt_left_arm->get_pose();
    m_collision_object_manager->updateRTCollisionObject(pose_map);
}

void PoseMoveItInterface::validate_pred(const ArcHumanPosePred &pose)
{
}


void PoseMoveItInterface::watchdog_callback()
{   
    RCLCPP_WARN(m_node->get_logger(), "Not received msg in the allowed time, clearing objects");
    m_collision_object_manager->clearAllCollisionObjects();
}


