import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

kinematics_yaml = load_yaml(
        "arc_hybrid_planner", "config/hybrid_kinematics.yaml"
    )

sys.path.append(os.path.dirname(__file__))
from robot_description import *
    
def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="collision_object_manager",
        executable="collision_object_test",
        name="collision_object_test",
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return launch.LaunchDescription([demo_node])
