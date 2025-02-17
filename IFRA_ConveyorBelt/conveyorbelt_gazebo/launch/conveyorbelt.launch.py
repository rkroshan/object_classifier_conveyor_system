#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) Gazebo-ROS2 Conveyor Belt Plugin. URL: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.

# conveyorbelt.launch.py:
# Launch file for the IFRA_ConveyorBelt GAZEBO SIMULATION in ROS2:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    conveyorbelt_gazebo = os.path.join(
        get_package_share_directory('conveyorbelt_gazebo'),
        'worlds',
        'conveyorbelt.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': conveyorbelt_gazebo}.items(),
             )
    
    #spawn bin
    bin_path = PathJoinSubstitution(
        [FindPackageShare('conveyorbelt_gazebo'), "urdf", "bin.urdf"]
    )
    #spawn bin for left belt
    gazebo_left_spawn_bin = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "left_bin", "-file", bin_path, "-x", "-2.47", "-y", "0","-z", "0.05"],
        output="screen",
    )
    #spawn bin for left belt
    gazebo_right_spawn_bin = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "right_bin", "-file", bin_path, "-x", "2.47", "-y", "1","-z", "0.05"],
        output="screen",
    )
    #spawn bin for left belt
    gazebo_center_spawn_bin = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "center_bin", "-file", bin_path, "-x", "0", "-y", "2.23","-z", "0.05"],
        output="screen",
    )

    #spawn bin
    camera_path = PathJoinSubstitution(
        [FindPackageShare('conveyorbelt_gazebo'), "urdf", "camera.urdf"]
    )
    gazebo_spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "setupcam", "-file", camera_path],
        output="screen",
    )

    # DECLARE ros2_aruco LAUNCH file:
    ros2_aruco_detection = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_aruco'), 'launch'), '/aruco_recognition.launch.py'])
             )

    
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo,
        gazebo_spawn_camera,
        gazebo_center_spawn_bin,
        gazebo_left_spawn_bin,
        gazebo_right_spawn_bin,
        ros2_aruco_detection,
    ])