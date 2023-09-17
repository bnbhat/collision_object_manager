import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from robot_description import *
    
def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="collision_object_manager",
        executable="collision_object_manager_main",
        name="collision_object_manager_main",
        #prefix=["gdbserver localhost:3000"],
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([demo_node])
