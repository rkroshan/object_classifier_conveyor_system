import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers


class BeltChanger(Node):

    def __init__(self, node_name="BeltChanger", marker_to_subscribe="dummy"):
        self.marker_to_subscribe = marker_to_subscribe
        self.node_name = node_name
        super().__init__(node_name)

        self.subscription = self.create_subscription(
            ArucoMarkers, self.marker_to_subscribe, self.belt_change_callback, 10
        )

    def belt_change_callback(self, msg):
        self.get_logger().info('%s: %d' % (self.node_name, msg.marker_ids[0]))
        self.get_logger().info('%s: %f' % (self.node_name, msg.poses[0].position.x))
    
def main(args=None):
    rclpy.init(args=args)

    left_beltchanger = BeltChanger(node_name="left_beltchanger", marker_to_subscribe="aruco_node_left/aruco_markers")
    right_beltchanger = BeltChanger(node_name="right_beltchanger", marker_to_subscribe="aruco_node_right/aruco_markers")

    executor = MultiThreadedExecutor()
    executor.add_node(left_beltchanger)
    executor.add_node(right_beltchanger)
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    executor.shutdown()

if __name__ == "__main__":
    main()
