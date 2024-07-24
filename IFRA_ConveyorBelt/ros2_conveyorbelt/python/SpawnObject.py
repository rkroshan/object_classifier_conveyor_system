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

# ===================================== COPYRIGHT ===================================== #
# SpawnObject.py script taken from:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl -> ros2srrc_execution ROS2 Package.

# IMPORT LIBRARIES:
import os
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy
import time
import random
import uuid
# Reference to SPAWN OBJECT (.urdf or .xacro file) from the terminal shell:
# REFERENCE: ros2 run ros2_conveyorbelt SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
# EXAMPLE: BOX -> ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.0 --y -0.5 --z 0.76

sdf_package_name = 'conveyorbelt_gazebo'
sdf_relative_location = 'models/aruco_marker'
boxes_type = ['model_red.sdf', 'model_blue.sdf', 'model_green.sdf']

Duration_btw_bxs_spawn_sec = 30

def main():
    # Start node:
    rclpy.init()
    node = rclpy.create_node('entity_spawner_boxes_model')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # Set data for request:
    request = SpawnEntity.Request()

    try:
        while True:
            _rand_box_num = random.randint(0,2)
            request.name = str(uuid.uuid4())
            sdf_file_path = os.path.join(get_package_share_directory(sdf_package_name), sdf_relative_location, boxes_type[_rand_box_num]) 
            content = ""
            with open(sdf_file_path, 'r') as content_file:
                content = content_file.read()
            request.xml = content

            request.initial_pose.position.x = 0.0
            request.initial_pose.position.y = -1.8
            request.initial_pose.position.z = 0.76

            node.get_logger().info('Spawning OBJECT using service: `/spawn_entity`')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                print('response: %r' % future.result())
            else:
                raise RuntimeError(
                    'exception while calling service: %r' % future.exception())
            #sleep
            time.sleep(Duration_btw_bxs_spawn_sec)
    except Exception as e:
        print(e)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()