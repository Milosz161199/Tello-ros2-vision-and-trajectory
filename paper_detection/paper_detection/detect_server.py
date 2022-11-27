import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_detect.action import Detect

import cv2 as cv
import numpy as np
import time
import rclpy # Python Client Library for ROS 2
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from imutils.perspective import order_points


class DetectActionServer(Node):

    def __init__(self):
        super().__init__('detect_action_server')
        self._action_server = ActionServer(
            self,
            Detect,
            'Detect',
            self.execute_callback)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]

        roi = self.detect_paper()

        goal_handle.succeed()

        result = Detect.Result()
        result.image = roi
        return result

    def perspective_transformation(self, img, box):
        box = np.array(box, dtype="int")
        src_pts = order_points(box)

        # use Euclidean distance to get width & height
        width = int(np.linalg.norm(src_pts[0] - src_pts[1]))
        height = int(np.linalg.norm(src_pts[0] - src_pts[3]))

        dst_pts = np.array([[0,0], [width,0], [width,height], [0,height]], dtype=np.float32)

        M = cv.getPerspectiveTransform(src_pts, dst_pts)
        warped_img = cv.warpPerspective(img, M, (width, height))

        return warped_img
        
    def detect_paper(self):
        br = CvBridge()
        result = Image()

        # from itertools import combinations

        # Load the webcam
        cap = cv.VideoCapture(0)

        # Add sliders for canny parameters
        # cv.namedWindow("Trackbars")
        # cv.createTrackbar("Min Threshold", "Trackbars", 1, 250, lambda x: None)
        # cv.createTrackbar("Max Threshold", "Trackbars", 84, 250, lambda x: None)

        previous_contour = None
        # start timer
        start = time.time()

        while True:
            

            # Read trackbars
            # min_threshold = cv.getTrackbarPos("Min Threshold", "Trackbars")
            # max_threshold = cv.getTrackbarPos("Max Threshold", "Trackbars")

            # Read the webcam
            _, img = cap.read()
            # img = cv.imread("room_with_grid2.png")
            
            # img = cv.resize(img, (0, 0), fx=0.4, fy=0.4)
            # Convert to grayscale
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            blur = cv.GaussianBlur(gray, (5, 5), 0)

            blur = cv.morphologyEx(blur, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)

            # canny = cv.Canny(blur, 1, 84)
            canny = cv.Canny(blur, 30, 50)
            canny = cv.morphologyEx(canny, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)
            
            contours, hierarchy = cv.findContours(canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            biggest = None
            roi = None
            # biggest = []
            
            max_area = 0
            for contour in contours:
                area = cv.contourArea(contour)
                if area > 100:

                    peri = cv.arcLength(contour, True)
                    approx = cv.approxPolyDP(contour, 0.02 * peri, True)
                    if area > max_area and len(approx) == 4:
                        # biggest = []
                        # biggest.append(approx)
                        warp_box = approx.reshape(4,2)
                        biggest = contour
                        # print(approx)
                        max_area = area
            
            
            if biggest is not None:
                # cv.drawContours(img, biggest, -1, (0, 255, 0), 2)
                rect = cv.minAreaRect(biggest)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cv.drawContours(img, [box], 0, (0, 0, 255), 2)

            # calculate elapsed time
            elapsed = time.time() - start

            # if elapsed time is greater than 1 second
            if elapsed > 4 and previous_contour is not None:
                # calculate the difference between the current contour and the previous contour
                difference = cv.matchShapes(box, previous_contour, 2, 0.0)
                print(difference)
                # reset timer
                if difference < 0.1 and biggest is not None:
                    print("Paper detected")
                    x, y, w, h = cv.boundingRect(box)
                    roi = img[y:y+h, x:x+w]
                    roi = self.perspective_transformation(img, warp_box)
                    cv.imwrite('roi.png', roi)
                    result = br.cv2_to_imgmsg(roi)
                    return result
                start = time.time()
                # set previous contour to current contour
                previous_contour = box

            if previous_contour is None and biggest is not None:
                previous_contour = box

            cv.imshow("Corners", img)
            # cv.imshow("ROI", roi)
            # cv.imshow("Thresh", thresh)

            # Stop if escape key is pressed
            k = cv.waitKey(30) & 0xff
            if k==27:
                break
            


        # Release the webcam
        cap.release()

        # Close all windows
        cv.destroyAllWindows()




def main(args=None):
    rclpy.init(args=args)

    detect_action_server = DetectActionServer()

    rclpy.spin(detect_action_server)


if __name__ == '__main__':
    main()