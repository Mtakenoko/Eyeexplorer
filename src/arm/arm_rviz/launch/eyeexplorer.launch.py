import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('arm_rviz'), 'urdf', 'eyeexplorer3.urdf')
    return LaunchDescription([
        # TS01
        Node(package='ts01', node_executable='ts01_manager', output='screen'),
        # 内視鏡キャプチャ
        Node(package='endoscope', node_executable='cap_endoscope'),
        # map_server
        Node(package='map_server', node_executable='map_server'),
        # joint_state_publisher
        Node(package='arm_status', node_executable='joint_publisher', output='screen'),
        # arm_state_publisher
        Node(package='arm_status', node_executable='arm_state_publisher'),
        # robot_state_publisher
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        # Insertion Point
        # Node(package='arm_status', node_executable='insertpoint_estimator'),
        # Reconstructor
        # Node(package='endoscope', node_executable='reconstructor'),
        # Esitimate Insertion Point
        # Node(package='map', node_executable='eyeball_publisher'),
        # Esitimate Insertion Point
        # Node(package='map', node_executable='eyeball_estimator_insertion_point'),
        # Pullout
        # Node(package='map', node_executable='pullout', output='screen')
    ])
