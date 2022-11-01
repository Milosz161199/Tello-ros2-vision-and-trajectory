import cv2
import numpy as np


class TrajectoryDetector:
    def __init__(self, image):
        self.img_color = image

    def getTrajectory(self):
        pass

    def findWhitePaper(self):
        pass



    def detectColors(self):
        hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)

        blue_lower = np.array([94, 80, 2])
        blue_upper = np.array([120, 255, 255])

        red_lower = np.array([136, 87, 111])
        red_upper = np.array([180, 255, 255])

        green_lower = np.array([32, 120, 100])
        green_upper = np.array([85, 242, 130])

        yellow_lower = np.array([19, 107, 89])
        yellow_upper = np.array([35, 255, 255])

        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_result = cv2.bitwise_and(self.img_color, self.img_color, mask=red_mask)
        yellow_result = cv2.bitwise_and(self.img_color, self.img_color, mask=yellow_mask)
        green_result = cv2.bitwise_and(self.img_color, self.img_color, mask=green_mask)
        blue_result = cv2.bitwise_and(self.img_color, self.img_color, mask=blue_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(yellow_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
                x, y, w, h = cv2.boundingRect(contour)
                # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                color = "Yellow"
                colorFound = True
                cv2.putText(self.img_color, "Yellow", (x + w + 20, y + h),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (255, 255, 0))

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
                x, y, w, h = cv2.boundingRect(contour)
                # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                color = "Red"
                colorFound = True
                cv2.putText(self.img_color, "Red", (x + w + 20, y + h),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (0, 0, 255))

        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
                x, y, w, h = cv2.boundingRect(contour)
                # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                color = "Green"
                colorFound = True
                cv2.putText(self.img_color, "Green", (x + w + 20, y + h),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (0, 255, 0))

        # Creating contour to track blue color
        contours, hierarchy = cv2.findContours(blue_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 1000):
                x, y, w, h = cv2.boundingRect(contour)
                # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                color = "Blue"
                colorFound = True
                cv2.putText(self.img_color, "Blue", (x + w + 20, y + h),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (255, 0, 0))
