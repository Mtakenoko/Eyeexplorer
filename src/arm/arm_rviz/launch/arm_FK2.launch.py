import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('arm_rviz'), 'urdf', 'eyeexplorer3_fixed2.urdf')
    return LaunchDescription([
        Node(package='map_server', node_executable='map_server', output='screen'),
        Node(package='arm_status', node_executable='joint_publisher', output='screen'),
        Node(package='arm_status', node_executable='arm_state_holder', output='screen'),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf])
    ])
