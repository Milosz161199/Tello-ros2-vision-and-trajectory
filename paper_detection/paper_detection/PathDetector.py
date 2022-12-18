import cv2
import numpy as np

from Point3D import *
from itertools import groupby, product
 



DEBUG_MODE = False

class PathDetector:
    def __init__(self, image):
        self.__img_color = image    

        # ksize = (5, 5)
        # self.__img_color = cv2.medianBlur(self.__img_color, 3)         
        self.__img_hsv = cv2.cvtColor(self.__img_color, cv2.COLOR_BGR2HSV)
        # self.__img_color = cv2.cvtColor(self.__img_color, cv2.COLOR_BGR2RGB)

        # count of squers
        self.__w_count = 55
        self.__h_count = 38
        self.__diff_square = 0.0
        self.__square_cx = 0.0
        self.__square_cy = 0.0

        # Image shape
        self.__IMAGE_MIN_X = 0.0
        self.__IMAGE_MAX_X = self.__img_color.shape[0]
        self.__IMAGE_MIN_Y = 0.0
        self.__IMAGE_MAX_Y = self.__img_color.shape[1]

        # Paper shape
        self.__PAPER_MIN_Y = 0.0
        self.__PAPER_MAX_Y = 18.5
        self.__PAPER_MIN_X = 0.0
        self.__PAPER_MAX_X = 27.0

        # Classroom shape
        self.__CLASSROOM_MIN_Y = -90.0   # 0.0 
        self.__CLASSROOM_MAX_Y =  90.0   # 180.0 
        self.__CLASSROOM_MIN_X = -200.0  # 0.0
        self.__CLASSROOM_MAX_X =  200.0  # 400.0

        self.__HOW_OFTEN = 2 # 2 

        # self.__start_point = Point3D(int(self.__CLASSROOM_MIN_X), int(self.__CLASSROOM_MAX_Y), [0, 0, 0])
        self.__start_point = Point3D(int(self.__IMAGE_MIN_X), int(self.__IMAGE_MAX_Y), [0, 0, 0])
        

        self.__path = []
        self.__sorted_path = [self.__start_point.getPoint3DArray()]

        self.__blue_lower = np.array([94, 80, 2])
        self.__blue_upper = np.array([120, 255, 255])
        self.__blue_lower_1 = np.array([84, 25, 20])
        self.__blue_upper_1 = np.array([131, 255, 130])

        self.__red_lower_0 = np.array([0, 50, 50])
        self.__red_upper_0 = np.array([10, 255, 255])
        self.__red_lower_1 = np.array([170, 50, 50])
        self.__red_upper_1 = np.array([180, 255, 255])
        self.__red_lower_2 = np.array([67, 58, 58]) 
        self.__red_upper_2 = np.array([255, 255, 255])

        self.__green_lower = np.array([36, 25, 25])
        self.__green_upper = np.array([70, 255, 255])
        self.__green_lower_1 = np.array([32, 0, 0]) 
        self.__green_upper_1 = np.array([116, 28, 117])

        self.__yellow_lower = np.array([19, 107, 89])
        self.__yellow_upper = np.array([35, 255, 255])
        self.__yellow_lower_1 = np.array([10, 30, 101])
        self.__yellow_upper_1 = np.array([101, 255, 224])

        self.__green_mask = None
        self.__yellow_mask = None
        self.__red_mask = None
        self.__blue_mask = None

    def __Manhattan(self, tup1, tup2):
        return abs(tup1[0] - tup2[0]) + abs(tup1[1] - tup2[1])

    def __groupPoints(self, path, distance):
        # initializing list
        test_list = list()
        result_array = list()

        for point in path:
            test_list.append(tuple(point))
                
        # Group Adjacent Coordinates
        # Using product() + groupby() + list comprehension
        man_tups = [sorted(sub) for sub in product(test_list, repeat = 2)
                                                if self.__Manhattan(*sub) <= distance]
        
        res_dict = {ele: {ele} for ele in test_list}
        for tup1, tup2 in man_tups:
            res_dict[tup1] |= res_dict[tup2]
            res_dict[tup2] = res_dict[tup1]
        
        res = [[*next(val)] for key, val in groupby(
                sorted(res_dict.values(), key = id), id)]

        for group in res:
            middle_index = int(len(group) / 2.0)
            result_array.append(list(group[middle_index]))

        return result_array

    def __changePositionOfCoordinateSystem(self) -> None:
        for i in range(1, len(self.__sorted_path)):
            self.__sorted_path[i][0] = int(self.__sorted_path[i][0] - int(self.__CLASSROOM_MAX_X / 2))
            self.__sorted_path[i][1] = int(self.__sorted_path[i][1] + int(self.__CLASSROOM_MAX_Y / 2))

    def getPath(self):
        return self.__sorted_path

    def __sortPoints(self, path :list) -> None:
        while path:
            path.sort(key=lambda point3d: np.sqrt((point3d[0] - self.__sorted_path[-1][0]) ** 2 + (point3d[1] - self.__sorted_path[-1][1]) ** 2 + (point3d[2] - self.__sorted_path[-1][2]) ** 2))
            self.__sorted_path.append(path[0])
            path.pop(0)

    def __findWrongPoints(self):
        path_tmp = list()
        for i in range(len(self.__sorted_path) - 1):
            distance = np.sqrt((self.__sorted_path[i][0] - self.__sorted_path[i+1][0]) ** 2 + (self.__sorted_path[i][1] - self.__sorted_path[-1][1]) ** 2 )
            if distance <= (self.__IMAGE_MAX_X * 0.8):
                path_tmp.append(self.__sorted_path[i])
        self.__sorted_path.clear()
        self.__sorted_path = path_tmp    
    
    def __mostFrequent(self, list : list()) -> float:
        return max(set(list), key = list.count)
    
    def __findCommonDistanceBetweenPoints(self, path :list()) -> float:
        distance_array = list()
        for i in range(len(path) - 1):
            distance = np.sqrt((path[i][0] - path[i+1][0]) ** 2 + (path[i][1] - path[-1][1]) ** 2 )
            distance_array.append(distance)

        return min(distance_array)
        
    def __calculateYaw(self) -> None:
        for i in range(1, len(self.__sorted_path)):
            diff_x = self.__sorted_path[i][0] - self.__sorted_path[i - 1][0]
            diff_y = self.__sorted_path[i][1] - self.__sorted_path[i - 1][1]
            self.__sorted_path[i][5] = float(np.round(np.arctan2(diff_y, diff_x) * 180 / np.pi, 2))
            if DEBUG_MODE:
                print(f"diff_x: {diff_x}, diff_y: {diff_y}, yaw: {self.sorted_trajectory[i][5]}")

    def preparePath(self) -> None:
        self.__newWayOfDetect()

        # self.__replace_to_close_points()

        # self.__sortPoints()
        # self.__sorted_path = self.__path
        distance = np.sqrt(np.power(self.__square_cx, 2) + np.power(self.__square_cy, 2))
        tmp_path = self.__groupPoints(self.__path, distance + .2)
        distance = self.__findCommonDistanceBetweenPoints(tmp_path)
        self.__path = self.__groupPoints(tmp_path, distance + .2)
        self.__sortPoints(self.__path)
        self.__findWrongPoints()
        # self.__sorted_path = self.__groupPoints(self.__sorted_path, distance)
        
        self.__calculateYaw()

    def __newWayOfDetect(self) -> None:
        self.__createMasks()

        array_of_points_to_check = list()
        height, width = self.__img_color.shape[:2]

        square_a = width / self.__w_count
        square_b = height / self.__h_count
        print(square_a, square_b)

        self.__diff_square = np.abs(square_a - square_b)

        if self.__diff_square > 1.0:
            return

        # width and height to center of square
        self.__square_cx = square_a / 2.0
        self.__square_cy = square_b / 2.0

        # create points to check
        for h in range(int(self.__square_cy), int(height), self.__HOW_OFTEN * int(self.__square_cy)):
            for w in range(int(self.__square_cx), int(width), self.__HOW_OFTEN * int(self.__square_cx)):
                array_of_points_to_check.append([h, w])
                point = [h, w]
                color = self.__witchColor(point)
                if str(color) == "[0, 0, 0]":
                    continue
                self.__img_color[h, w] = (0, 0, 0)
                point3d = Point3D(w, h, color)
                if DEBUG_MODE:
                    point3d.showPoint()
                self.__path.append(point3d.getPoint3DArray())

        

        # print("trajectory len: ", len(self.__path))
        # print(array_of_points_to_check)
        # print("len: ", len(array_of_points_to_check))
        # print("real len: ", w_count * h_count)
        # del self.__path[::10]
        print("trajectory len: ", len(self.__path))
        # cv2.imwrite("res.png", self.__img_color)

    def __isBlue(self, point) -> bool:
        if str(self.__blue_mask[point[0], point[1]]) != "[0 0 0]":
            if DEBUG_MODE:
                print("blue: ", self.__blue_mask[point[0], point[1]])
            return True
        else:
            return False

    def __isRed(self, point) -> bool:
        if str(self.__red_mask[point[0], point[1]]) != "[0 0 0]":
            if DEBUG_MODE:
                print("red: ", self.__red_mask[point[0], point[1]])
            return True
        else:
            return False

    def __isGreen(self, point) -> bool:
        if str(self.__green_mask[point[0], point[1]]) != "[0 0 0]":
            if DEBUG_MODE:
                print("green: ", self.__green_mask[point[0], point[1]])
            return True
        else:
            return False

    def __isYellow(self, point) -> bool:
        if str(self.__yellow_mask[point[0], point[1]]) != "[0 0 0]":
            if DEBUG_MODE:
                print("yellow: ", self.__yellow_mask[point[0], point[1]])
            return True
        else:
            return False

    def __witchColor(self, point) -> list:
        if self.__isBlue(point):
            return [0, 0, 255]
        elif self.__isGreen(point):
            return [0, 255, 0]
        elif self.__isRed(point):
            return [255, 0, 0]
        elif self.__isYellow(point):
            return [255, 255, 0]
        else:
            return [0, 0, 0]

    def __createRedColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__red_lower_0, self.__red_upper_0)
        mask_1 = cv2.inRange(self.__img_hsv, self.__red_lower_1, self.__red_upper_1)
        mask_2 = cv2.inRange(self.__img_hsv, self.__red_lower_2, self.__red_upper_2)
        mask = mask_0 + mask_1 + mask_2
        self.__red_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("RED", self.__red_mask)

    def __createBlueColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__blue_lower, self.__blue_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__blue_lower_1, self.__blue_upper_1)
        mask = mask_0 + mask_1
        self.__blue_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("BLUE", self.__blue_mask)

    def __createGreenColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__green_lower, self.__green_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__green_lower_1, self.__green_upper_1)
        mask = mask_0 + mask_1
        self.__green_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("GREEN", self.__green_mask)

    def __createYellowColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__yellow_lower, self.__yellow_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__yellow_lower_1, self.__yellow_upper_1)
        mask = mask_0 + mask_1
        self.__yellow_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("YELLOW", self.__yellow_mask)

    def __createMasks(self) -> None:
        self.__createYellowColorMask()
        self.__createRedColorMask()
        self.__createBlueColorMask()
        self.__createGreenColorMask()

    def __normalize(self, arr, t_min, t_max):
        norm_arr = []
        diff = t_max - t_min
        diff_arr = max(arr) - min(arr)
        for i in arr:
            temp = (((i - min(arr)) * diff) / diff_arr) + t_min
            norm_arr.append(int(temp))
        return norm_arr

    def __normalizeArray(self, array_1d: list, ax: str) -> list():
        # gives range starting from 1 and ending at 3
        range_to_normalize = (0, 0)
        if ax == "x":
            range_to_normalize = (self.__CLASSROOM_MIN_X, self.__CLASSROOM_MAX_X)
        elif ax == "y":
            range_to_normalize = (self.__CLASSROOM_MIN_Y, self.__CLASSROOM_MAX_Y)
            
        normalized_array_1d = self.__normalize(array_1d,
                                        range_to_normalize[0],
                                        range_to_normalize[1])
        if DEBUG_MODE:
            # display original and normalized array
            print("Original Array = ", array_1d)
            print("Normalized Array = ", normalized_array_1d)
        
        return normalized_array_1d

    def getArrays(self):
        x_a = list()
        y_a = list()
        z_a = list()
        pitch_a = list()
        roll_a = list()
        yaw_a = list()
        is_visited_a = list()

        for point3d in self.__sorted_path:
            x_a.append(point3d[0])
            y_a.append(point3d[1])
            z_a.append(point3d[2])
            pitch_a.append(point3d[3])
            roll_a.append(point3d[4])
            yaw_a.append(point3d[5])
            is_visited_a.append(point3d[6])

        # normalize x and y
        x_a = self.__normalizeArray(x_a, "x")
        y_a = self.__normalizeArray(y_a, "y")
        
        return x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a
