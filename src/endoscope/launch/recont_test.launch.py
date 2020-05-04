import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    return LaunchDescription([
        Node(package='endoscope', node_executable='reconstruction_BA', output='screen'),
        Node(package='endoscope', node_executable='reconstruction_BA_class', output='screen')
    ])
