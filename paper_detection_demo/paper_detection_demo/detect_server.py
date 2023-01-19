import cv2
import numpy as np
import time
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer
from rclpy.node import Node

from action_detect.action import Detect
from sensor_msgs.msg import Image
from imutils.perspective import order_points
from Colors import Colors
from Point3D import Point3D
from PathDetector import PathDetector

import matplotlib.pyplot as plt
import time


WEBCAM_TEST = False
TEST = False
TEST_TELLO = False


class DetectActionServer(Node):
    def __init__(self):
        super().__init__('detect_action_server')
        
        self.__simulation = False
        self.__image_topic = str()
        
        if self.__simulation:
            self.__image_topic = 'drone1/image_raw'
        else:
            self.__image_topic = '/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            self.__image_topic,
            self.listener_callback,
            10) 
        self.subscription  # prevent unused variable warning

        self._action_server = ActionServer(
            self,
            Detect,
            'Detect',
            self.execute_callback)

        self.image = None

        self.__path_detector = None
        self.__image = []
        self.__image_ros = Image()
        self.__br = CvBridge()
        self.__result = None
        
        
    def listener_callback(self, msg):
        self.image = self.__br.imgmsg_to_cv2(msg)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        x_a = [1.00, 1.00]
        y_a = [0.00, 0.00]
        z_a = [1.00, 1.00]
        pitch_a = [0.0, 0.0, 0.0]
        roll_a = [0.0, 0.0, 0.0]
        yaw_a = [0.0, 0.0, 0.0]
        is_visited_a = [False, False, False]

        goal_handle.succeed()

        self.__result = Detect.Result()

        self.__result.number_of_points = len(list(x_a))
        self.__result.x = list(x_a)
        self.__result.y = list(y_a)
        self.__result.z = list(z_a)
        self.__result.pitch = list(pitch_a)
        self.__result.roll = list(roll_a)
        self.__result.yaw = list(yaw_a)
        self.__result.is_visited = list(is_visited_a)

        return self.__result


def main(args=None):
    rclpy.init(args=args)

    detect_action_server = DetectActionServer()

    rclpy.spin(detect_action_server)


if __name__ == '__main__':
    main()
