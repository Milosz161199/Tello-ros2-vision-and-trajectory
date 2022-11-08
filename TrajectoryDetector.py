import cv2
from Point3D import *
import imutils
import numpy as np


class TrajectoryDetector:
    def __init__(self, image):

        self.GRID_SIZE = 5

        self.start_point = Point3D(0, 0, [0, 0, 0])

        self.img_color = image
        self.trajectory = []
        self.sorted_trajectory = [self.start_point.getPoint3DArray()]

        self.img_hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)

        self.blue_lower = np.array([94, 80, 2])
        self.blue_upper = np.array([120, 255, 255])

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
        return self.sorted_trajectory

    def findWhitePaper(self):
        pass

    def findRedColor(self):
        mask_0 = cv2.inRange(self.img_hsv, self.red_lower_0, self.red_upper_0)
        mask_1 = cv2.inRange(self.img_hsv, self.red_lower_1, self.red_upper_1)
        mask = mask_0 + mask_1
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        # a = np.argwhere(result != [0, 0, 0])
        # a = np.delete(a, 2, 1)  # delete third column in a
        # counter = self.GRID_SIZE

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            point3d = Point3D(cX, cY, [255, 0, 0])
            self.trajectory.append(point3d.getPoint3DArray())

        # for point2d in a:
        #     if counter == self.GRID_SIZE:
        #         point3d = Point3D(point2d[0], point2d[1], [255, 0, 0])
        #         self.trajectory.append(point3d.getPoint3DArray())
        #         counter = 0
        #         continue
        #     counter += 1

        cv2.imshow("RED", result)

    def findBlueColor(self):
        mask = cv2.inRange(self.img_hsv, self.blue_lower, self.blue_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            point3d = Point3D(cX, cY, [0, 0, 255])
            self.trajectory.append(point3d.getPoint3DArray())

        # a = np.argwhere(result != [0, 0, 0])
        # a = np.delete(a, 2, 1)  # delete third column in a
        # counter = self.GRID_SIZE
        # for point2d in a:
        #     if counter == self.GRID_SIZE:
        #         point3d = Point3D(point2d[0], point2d[1], [0, 0, 255])
        #         self.trajectory.append(point3d.getPoint3DArray())
        #         counter = 0
        #         continue
        #     counter += 1
        #
        cv2.imshow("BLUE", result)

    def findGreenColor(self):
        mask = cv2.inRange(self.img_hsv, self.green_lower, self.green_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            point3d = Point3D(cX, cY, [0, 255, 0])
            self.trajectory.append(point3d.getPoint3DArray())

        # a = np.argwhere(result != [0, 0, 0])
        # a = np.delete(a, 2, 1)  # delete third column in a
        # counter = self.GRID_SIZE
        # for point2d in a:
        #     if counter == self.GRID_SIZE:
        #         point3d = Point3D(point2d[0], point2d[1], [0, 255, 0])
        #         self.trajectory.append(point3d.getPoint3DArray())
        #         counter = 0
        #         continue
        #     counter += 1

        cv2.imshow("GREEN", result)

    def findYellowColor(self):
        mask = cv2.inRange(self.img_hsv, self.yellow_lower, self.yellow_upper)
        result = cv2.bitwise_and(self.img_color, self.img_color, mask=mask)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            point3d = Point3D(cX, cY, [255, 255, 0])
            self.trajectory.append(point3d.getPoint3DArray())
        # a = np.argwhere(result != [0, 0, 0])
        # a = np.delete(a, 2, 1)  # delete third column in a
        # counter = self.GRID_SIZE
        # for point2d in a:
        #     if counter == self.GRID_SIZE:
        #         point3d = Point3D(point2d[0], point2d[1], [255, 255, 0])
        #         self.trajectory.append(point3d.getPoint3DArray())
        #         counter = 0
        #         continue
        #     counter += 1

        cv2.imshow("YELLOW", result)

    def detectColors(self):
        self.findBlueColor()
        self.findGreenColor()
        self.findRedColor()
        self.findYellowColor()

    def sortPoints(self):
        while self.trajectory:
            self.trajectory.sort(key=lambda point3d: np.sqrt(
                (point3d[0] - self.sorted_trajectory[-1][0]) ** 2 + (
                        point3d[1] - self.sorted_trajectory[-1][1]) ** 2 + (
                        point3d[2] - self.sorted_trajectory[-1][2]) ** 2))
            self.sorted_trajectory.append(self.trajectory[0])
            self.trajectory.pop(0)

    def calculateYaw(self):
        for i in range(1, len(self.sorted_trajectory)):
            diff_x = self.sorted_trajectory[i][0] - self.sorted_trajectory[i - 1][0]
            diff_y = self.sorted_trajectory[i][1] - self.sorted_trajectory[i - 1][1]
            # print("diff_x: ", diff_x, " diff_y: ", diff_y)
            self.sorted_trajectory[i][5] = np.round(np.arctan2(diff_y, diff_x) * 180 / np.pi, 2)
            # print(self.sorted_trajectory[i][5])

    def prepareTrajectory(self) -> None:
        self.detectColors()
        self.sortPoints()
        self.calculateYaw()
