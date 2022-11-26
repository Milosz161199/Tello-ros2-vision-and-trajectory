import sys

from trajectory_interfaces_msgs.srv import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class PathGeneratorClientAsync(Node):

    def __init__(self):
        super().__init__('path_generator_client_async')
        self.cli = self.create_client(Path, 'path_generator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Path.Request()

    def send_request(self):
        tmp = sys.argv[1]
        self.get_logger().info(str(tmp))
        self.req.image = Image(sys.argv[1])
        self.future = self.cli.call_async(self.req)

        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    path_generator_client = PathGeneratorClientAsync()
    path_generator_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(path_generator_client)
        # See if the service has replied
        if path_generator_client.future.done():
            try:
                response = path_generator_client.future.result()
            except Exception as e:
                path_generator_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                path_generator_client.get_logger().info('Result')
            break
 
    path_generator_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()