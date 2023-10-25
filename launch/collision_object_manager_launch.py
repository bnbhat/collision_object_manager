import launch
import os
import sys
import yaml

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.dirname(__file__))
from robot_description import *

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    pose_interface_config = load_yaml('collision_object_manager', 'config/config.yaml')

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
            pose_interface_config,
        ],
    )

    return launch.LaunchDescription([demo_node])
