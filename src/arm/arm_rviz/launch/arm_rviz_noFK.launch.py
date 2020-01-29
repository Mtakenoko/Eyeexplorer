import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    urdf = os.path.join(get_package_share_directory('arm_rviz'), 'urdf', 'eyeexplorer3_nooffset.urdf')
    return LaunchDescription([
        Node(package='map_server', node_executable='map_server', output='screen'),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='arm_status', node_executable='joint_publisher', output='screen')
    ])
