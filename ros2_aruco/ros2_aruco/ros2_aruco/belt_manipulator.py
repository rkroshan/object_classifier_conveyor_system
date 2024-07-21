import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco_interfaces.msg import ArucoMarkers
from conveyorbelt_msgs.srv import ConveyorBeltChanger

class BeltChanger(Node):

    def __init__(self, node_name="BeltChanger", marker_to_subscribe="dummy", marker_id=999, left_belt_on=False, right_belt_on=False):
        self.marker_to_subscribe = marker_to_subscribe
        self.node_name = node_name
        self.marker_id = marker_id
        self.left_belt_on = left_belt_on
        self.right_belt_on = right_belt_on
        super().__init__(node_name)

        self.subscription = self.create_subscription(
            ArucoMarkers, self.marker_to_subscribe, self.belt_change_callback, 10
        )

        self.cli = self.create_client(ConveyorBeltChanger, 'ENABLEBELT')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ConveyorBeltChanger.Request()
        # initial setup for req since req is same all time
        self.req.left_belt_on = self.left_belt_on
        self.req.right_belt_on = self.right_belt_on

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def belt_change_callback(self, msg):
        # self.get_logger().info('%s: %d' % (self.node_name, msg.marker_ids[0]))
        # self.get_logger().info('%s: %f' % (self.node_name, msg.poses[0].position.x))
        if((self.marker_id == msg.marker_ids[0]) and ((msg.poses[0].position.x * 100.0) < 1.0) and ((msg.poses[0].position.x * 100.0) >= 0.0 )):
            # call the service to push the box
            self.get_logger().info('Pushed %s: %f' % (self.node_name, msg.poses[0].position.x))
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
    

    
def main(args=None):
    rclpy.init(args=args)

    left_beltchanger = BeltChanger(node_name="left_beltchanger", marker_to_subscribe="aruco_node_left/aruco_markers", marker_id=1, left_belt_on=True, right_belt_on=False)
    right_beltchanger = BeltChanger(node_name="right_beltchanger", marker_to_subscribe="aruco_node_right/aruco_markers", marker_id=2, left_belt_on=False, right_belt_on=True)

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
