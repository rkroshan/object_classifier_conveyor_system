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
        parameters=[aruco_params_left],
        remappings=[
            ('/aruco_markers', '/aruco_markers_left'),
            ('/aruco_poses', '/aruco_poses_left'),
        ]
    )

    aruco_node_right = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params_right],
        remappings=[
            ('/aruco_markers', '/aruco_markers_right'),
            ('/aruco_poses', '/aruco_poses_right'),
        ]
    )

    return LaunchDescription([
        aruco_node_left,
        aruco_node_right
    ])
