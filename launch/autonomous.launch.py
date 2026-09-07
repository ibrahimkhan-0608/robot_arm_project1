import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'six_axis_arm.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='robot_arm',
            executable='driver_node',
        ),
        Node(
            package='robot_arm',
            executable='planner_node',
        ),
        Node(
            package='robot_arm',
            executable='trail_node',
        ),
        Node(
            package='robot_arm',
            executable='obstacle_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        ),
    ])