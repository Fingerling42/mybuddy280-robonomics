import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Creating node for robot
    mybuddy280_node = Node(
        package='mybuddy280_driver',
        executable='mybuddy280_ros_wrapper',
    )

    ld.add_action(mybuddy280_node)
    return ld
