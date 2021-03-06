import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('arm_rviz'), 'urdf', 'eyeexplorer3.urdf')
    return LaunchDescription([
        # map_server
        Node(package='map_server', node_executable='map_server', output='screen'),
        # arm_state_publisher
        Node(package='arm_status', node_executable='arm_state_publisher', output='screen'),
        # robot_state_publisher
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        # Insertion Point
        Node(package='arm_status', node_executable='insertpoint_estimator'),
        # Esitimate EyeBall
        Node(package='map', node_executable='eyeball_estimator_insertion_point')  
    ])
