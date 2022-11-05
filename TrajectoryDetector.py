import cv2
import numpy as np
from Point3D import *


class TrajectoryDetector:
    def __init__(self, image):

        self.GRID_SIZE = 5

        self.img_color = image
        self.trajectory = list()

        self.img_hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)

        self.blue_upper = np.array([120, 255, 255])
        self.blue_lower = np.array([94, 80, 2])

        # lower mask (0-10)
        self.red_lower_0 = np.array([0, 50, 50])
        self.red_upper_0 = np.array([10, 255, 255])

        # upper mask (170-180)
        self.red_lower_1 = np.array([170, 50, 50])
        self.red_upper_1 = np.array([180, 255, 255])

        self.green_lower = np.array([36, 25, 25])
        self.green_upper = np.array([70, 255, 255])

        self.yellow_lower = np.array([19, 107, 89])
        self.yellow_upper = np.array([35, 255, 255])

    def getTrajectory(self):
        return self.trajectory

    def findWhitePaper(self):
        pass

    def findRedColor(self):
        mask_0 = cv2.inRange(self.img_hsv, self.red_lower_0, self.red_upper_0)
        mask_1 = cv2.inRange(self.img_hsv, self.red_lower_1, self.red_upper_1)
        mask = mask_0 + mask_1
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)
        height, width = result.shape[:2]

        for y in range(0, height, self.GRID_SIZE):
            for x in range(0, width, self.GRID_SIZE):
                if str(result[y, x]) != "[0 0 0]":
                    point3d = Point3D(x, y, [255, 0, 0])
                    self.trajectory.append(point3d.getPoint3D())

        # cv2.imshow("RED", result)

    def findBlueColor(self):
        mask = cv2.inRange(self.img_hsv, self.blue_lower, self.blue_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        height, width = result.shape[:2]

        for y in range(0, height, self.GRID_SIZE):
            for x in range(0, width, self.GRID_SIZE):
                if str(result[y, x]) != "[0 0 0]":
                    point3d = Point3D(x, y, [0, 0, 255])
                    self.trajectory.append(point3d.getPoint3D())

        # cv2.imshow("BLUE", result)

    def findGreenColor(self):
        mask = cv2.inRange(self.img_hsv, self.green_lower, self.green_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)
        height, width = result.shape[:2]

        for y in range(0, height, self.GRID_SIZE):
            for x in range(0, width, self.GRID_SIZE):
                if str(result[y, x]) != "[0 0 0]":
                    point3d = Point3D(x, y, [0, 255, 0])
                    self.trajectory.append(point3d.getPoint3D())

        # cv2.imshow("GREEN", result)

    def findYellowColor(self):
        mask = cv2.inRange(self.img_hsv, self.yellow_lower, self.yellow_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        height, width = result.shape[:2]

        for y in range(0, height, self.GRID_SIZE):
            for x in range(0, width, self.GRID_SIZE):
                if str(result[y, x]) != "[0 0 0]":
                    point3d = Point3D(x, y, [255, 255, 0])
                    self.trajectory.append(point3d.getPoint3D())

        # cv2.imshow("YELLOW", result)

    def detectColors(self):
        self.findBlueColor()
        self.findGreenColor()
        self.findRedColor()
        self.findYellowColor()
