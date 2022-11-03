import cv2
import numpy as np


class TrajectoryDetector:
    def __init__(self, image):
        self.img_color = image
        self.trajectory = list()

        self.img_hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)

        self.blue_upper = np.array([120, 255, 255])
        self.blue_lower = np.array([94, 80, 2])

        self.red_lower = np.array([170, 50, 50])
        self.red_upper = np.array([180, 255, 255])

        self.green_lower = np.array([32, 120, 100])
        self.green_upper = np.array([85, 242, 130])

        self.yellow_lower = np.array([19, 107, 89])
        self.yellow_upper = np.array([35, 255, 255])

    def getTrajectory(self):
        pass

    def findWhitePaper(self):
        pass

    def findRedColor(self):
        mask = cv2.inRange(self.img_hsv, self.red_lower, self.red_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cv2.imshow("RED", result)


    def findBlueColor(self):
        mask = cv2.inRange(self.img_hsv, self.blue_lower, self.blue_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cv2.imshow("BLUE", result)

        # # Creating contour to track blue color
        # contours, hierarchy = cv2.findContours(blue_mask,
        #                                        cv2.RETR_TREE,
        #                                        cv2.CHAIN_APPROX_SIMPLE)
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if (area > 1000):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        #         color = "Blue"
        #         colorFound = True
        #         cv2.putText(self.img_color, "Blue", (x + w + 20, y + h),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0,
        #                     (255, 0, 0))


    def findGreenColor(self):
        mask = cv2.inRange(self.img_hsv, self.green_lower, self.green_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cv2.imshow("GREEN", result)

    def findYellowColor(self):
        mask = cv2.inRange(self.img_hsv, self.yellow_lower, self.yellow_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cv2.imshow("YELLOW", result)

    def detectColors(self):
        self.findBlueColor()
        self.findGreenColor()
        self.findRedColor()
        self.findYellowColor()

        # yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        # red_mask = cv2.inRange(hsv, red_lower, red_upper)
        # green_mask = cv2.inRange(hsv, green_lower, green_upper)
        #
        # red_result = cv2.bitwise_and(self.img_color, self.img_color, mask=red_mask)
        # yellow_result = cv2.bitwise_and(self.img_color, self.img_color, mask=yellow_mask)
        # green_result = cv2.bitwise_and(self.img_color, self.img_color, mask=green_mask)
        #
        # # Creating contour to track red color
        # contours, hierarchy = cv2.findContours(yellow_mask,
        #                                        cv2.RETR_TREE,
        #                                        cv2.CHAIN_APPROX_SIMPLE)
        #
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if (area > 1000):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #         color = "Yellow"
        #         colorFound = True
        #         cv2.putText(self.img_color, "Yellow", (x + w + 20, y + h),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0,
        #                     (255, 255, 0))
        #
        # # Creating contour to track red color
        # contours, hierarchy = cv2.findContours(red_mask,
        #                                        cv2.RETR_TREE,
        #                                        cv2.CHAIN_APPROX_SIMPLE)
        #
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if (area > 1000):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #         color = "Red"
        #         colorFound = True
        #         cv2.putText(self.img_color, "Red", (x + w + 20, y + h),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0,
        #                     (0, 0, 255))
        #
        # # Creating contour to track green color
        # contours, hierarchy = cv2.findContours(green_mask,
        #                                        cv2.RETR_TREE,
        #                                        cv2.CHAIN_APPROX_SIMPLE)
        #
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if (area > 1000):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #         color = "Green"
        #         colorFound = True
        #         cv2.putText(self.img_color, "Green", (x + w + 20, y + h),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0,
        #                     (0, 255, 0))
        #
        #
