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

if TEST_TELLO:
    from threading import Thread
    from djitellopy import Tello


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

        self.__image_ros = self.detect_paper()
        self.__image = self.__br.imgmsg_to_cv2(self.__image_ros)

        if TEST:
            self.__image = cv2.imread("src/path_04.png")
            cv2.imshow('image to find path', self.__image)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

        self.__path_detector = PathDetector(self.__image)
        self.__path_detector.preparePath()

        x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a = self.__path_detector.getArrays()

        if not(self.__validateData(x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a)):
            number_of_points = 0
            x_a = list()
            y_a = list()
            z_a = list()
            pitch_a = list()
            roll_a = list()
            yaw_a = list()
            is_visited_a = list()
            goal_handle.abort()
            return self.__result

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
        
        # ''' BEGIN TO DELETE '''
        plt.scatter(list(x_a), list(y_a), s=1)
        plt.plot(list(x_a), list(y_a))
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.show()
        # ''' END TO DELETE '''

        return self.__result


    def __validateData(self, x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a) -> bool:
        if len(x_a) == len(y_a) == len(z_a) == len(pitch_a) == len(roll_a) == len(yaw_a) == len(is_visited_a):
            return True
        else:
            return False


    def perspective_transformation(self, img, box):
        box = np.array(box, dtype="int")
        src_pts = order_points(box)

        # use Euclidean distance to get width & height
        width = int(np.linalg.norm(src_pts[0] - src_pts[1]))
        height = int(np.linalg.norm(src_pts[0] - src_pts[3]))

        dst_pts = np.array([[0,0], [width,0], [width,height], [0,height]], dtype=np.float32)

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped_img = cv2.warpPerspective(img, M, (width, height))

        return warped_img
        

    def detect_paper(self):
        if TEST_TELLO:
            self.__tello.connect()
            self.__tello.streamon()
            frame_read = self.__tello.get_frame_read()     
        
        if TEST:
            cv2.namedWindow("Trackbars")
            cv2.createTrackbar("Min Threshold", "Trackbars", 17, 250, lambda x: None)
            cv2.createTrackbar("Max Threshold", "Trackbars", 70, 250, lambda x: None)
        
        if WEBCAM_TEST:
            # Load the webcam
            cap = cv2.VideoCapture('src/paper_detection/paper_detection/video_2.avi')
            
        previous_contour = None

        # start timer
        start = time.time()

        while True:
            print('Looking for image...',  end='\r')
            
            # Read the webcam
            if WEBCAM_TEST:
                _, img = cap.read()
            
            if self.image is None:
                continue
            else:
                img = self.image

            min_threshold = 17
            max_threshold = 70
            
            if TEST:
                min_threshold = cv2.getTrackbarPos("Min Threshold", "Trackbars")
                max_threshold = cv2.getTrackbarPos("Max Threshold", "Trackbars")

            if TEST_TELLO:
                img = self.__tello.get_frame_read().frame
                print(img.shape)
                img = cv2.imread("src/ARL_PROJ_GRUPA_IV/paper_detection/paper_detection/room_with_grid2.png")
                if img == None or len(img) == 0:
                    continue
            
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            blur = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=2)

            canny = cv2.Canny(blur, min_threshold, max_threshold)
            canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=2)
            
            contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            biggest = None
            roi = None
            
            max_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:

                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
                    if area > max_area and len(approx) == 4:
                        warp_box = approx.reshape(4,2)
                        biggest = contour
                        # print(approx)
                        max_area = area
            
            
            if biggest is not None:
                rect = cv2.minAreaRect(biggest)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

            # calculate elapsed time
            elapsed = time.time() - start
            print(f'Time to be sure about image: {round(elapsed, 2)}s/4.0s',  end='\r')
            # if elapsed time is greater than 1 second
            if elapsed > 4 and previous_contour is not None:
                # calculate the difference between the current contour and the previous contour
                difference = cv2.matchShapes(box, previous_contour, 2, 0.0)
                # print(difference)
                # reset timer
                if difference < 0.1 and biggest is not None:
                    print("Paper detected!")
                    x, y, w, h = cv2.boundingRect(box)
                    roi = img[y:y+h, x:x+w]
                    roi = self.perspective_transformation(img, warp_box)
                    # Write image
                    cv2.imwrite('roi.png', roi)
                    if TEST:
                        cv2.imshow('image', roi)
                    cv2.waitKey(1000)
                    self.__image_ros = self.__br.cv2_to_imgmsg(roi)
                    return self.__image_ros
                 
                start = time.time()
                # set previous contour to current contour
                previous_contour = box
            if TEST:
                # ''' BEGIN TO DELETE '''
                cv2.imshow('frame', img)
                cv2.imshow('Trackbars', canny)
                # ''' END TO DELETE '''
            cv2.waitKey(1)
            if previous_contour is None and biggest is not None:
                previous_contour = box      
            time.sleep(0.1)
            rclpy.spin_once(self)
        
        if WEBCAM_TEST:
            # Release the webcam
            cap.release()
        
        # Close all windows
        cv2.destroyAllWindows()
        


def main(args=None):
    rclpy.init(args=args)

    detect_action_server = DetectActionServer()

    rclpy.spin(detect_action_server)


if __name__ == '__main__':
    main()
