import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params_left = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_left_parameters.yaml'
        )
    
    aruco_params_right = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_right_parameters.yaml'
        )

    aruco_node_left = Node(
        package='ros2_aruco',
        executable='aruco_node',
        namespace='aruco_node_left',
        parameters=[aruco_params_left],
    )

    aruco_node_right = Node(
        package='ros2_aruco',
        executable='aruco_node',
        namespace='aruco_node_right',
        parameters=[aruco_params_right],
    )

    belt_manipulator_node = Node(
        package='ros2_aruco',
        executable='BeltChanger',
    )

    return LaunchDescription([
        aruco_node_left,
        aruco_node_right,
        belt_manipulator_node
    ])
