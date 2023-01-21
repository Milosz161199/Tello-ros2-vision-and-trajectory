import cv2
import numpy as np

from tqdm import tqdm
from Point3D import *

DEBUG_MODE = False

class PathDetector:
    def __init__(self, image):
        
        self.__img_color = self.__trimImage(image) 
        self.__img_hsv = cv2.cvtColor(self.__img_color, cv2.COLOR_BGR2HSV)

        # count of squares
        self.__w_count = 55 * 4 # 55
        self.__h_count = 38 * 4 # 38
        self.__diff_square = 0.0
        self.__square_cx = 0.0
        self.__square_cy = 0.0

        # Image shape
        self.__IMAGE_MIN_X = 0
        self.__IMAGE_MAX_X = self.__img_color.shape[0]
        self.__IMAGE_MIN_Y = 0
        self.__IMAGE_MAX_Y = self.__img_color.shape[1]

        # Classroom shape
        self.__CLASSROOM_MIN_X = -1.5 
        self.__CLASSROOM_MAX_X =  1.5 
        self.__CLASSROOM_MIN_Y = -1.0
        self.__CLASSROOM_MAX_Y =  1.0 

        self.__HOW_OFTEN = 2 # 2 

        self.__start_point = Point3D(self.__IMAGE_MIN_X + 20.0, self.__IMAGE_MAX_Y + 20.0, [0, 0, 0])
        
        self.__path = []
        self.__sorted_path = [self.__start_point.getPoint3DArray()]
        self.__sorted_path2 = list()

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

    def __trimImage(self, image):
        height, width = image.shape[:2]
        x_trim = int(width * 0.025)
        y_trim = int(height * 0.025)
        img_res = image[y_trim: height-y_trim, x_trim:width-x_trim]
        
        return img_res

    def __groupPoints(self, path, distance):  
        print("Starting group points in path...")     
        result_array = path
        
        for i, res_point in enumerate(result_array):
            for j, point in enumerate(path):
                dist = np.sqrt((res_point[0] - point[0]) ** 2 + 
                               (res_point[1] - point[1]) ** 2 + 
                               (res_point[2] - point[2]) ** 2)
                if dist <= distance:
                    if point in result_array:
                        result_array.remove(point)

        return result_array

    def getPath(self):
        return self.__sorted_path

    def __sortPoints(self, path :list) -> None:
        print("First sorting path...")
        while path:
            path.sort(key=lambda point3d: np.sqrt((point3d[0] - self.__sorted_path[-1][0]) ** 2 + (point3d[1] - self.__sorted_path[-1][1]) ** 2 + (point3d[2] - self.__sorted_path[-1][2]) ** 2))
            self.__sorted_path.append(path[0])
            path.pop(0)

    def __sortPoints2(self, path :list) -> None:
        print("Second sorting path...")
        while path:
            path.sort(key=lambda point3d: np.sqrt((point3d[0] - self.__sorted_path2[-1][0]) ** 2 + (point3d[1] - self.__sorted_path2[-1][1]) ** 2 + (point3d[2] - self.__sorted_path2[-1][2]) ** 2))
            self.__sorted_path2.append(path[0])
            path.pop(0)

    def __correctStartPointToPath(self) -> None:
        print("Correcting path...")
        start_point = self.__sorted_path[0]
        self.__sorted_path.pop(0)
        
        arr = np.zeros((len(self.__sorted_path), len(self.__sorted_path)))
        
        for i, res_point in enumerate(self.__sorted_path):
            for j, point in enumerate(self.__sorted_path):
                dist = np.sqrt((res_point[0] - point[0]) ** 2 + 
                               (res_point[1] - point[1]) ** 2 + 
                               (res_point[2] - point[2]) ** 2)
                arr[i][j] = dist
                
        index = np.unravel_index(arr.argmax(), arr.shape)

        self.__sorted_path2 = [self.__sorted_path[index[0]]]
        self.__sortPoints2(self.__sorted_path)
        self.__sorted_path = self.__sorted_path2

    def __findWrongPoints(self):
        print("Removing wrong points...")
        path_tmp = [self.__start_point.getPoint3DArray()]
        for i in range(len(self.__sorted_path) - 1):
            distance = np.sqrt((self.__sorted_path[i][0] - self.__sorted_path[i+1][0]) ** 2 + (self.__sorted_path[i][1] - self.__sorted_path[-1][1]) ** 2 )
            if distance <= (self.__IMAGE_MAX_X * 0.8):
                path_tmp.append(self.__sorted_path[i])
        self.__sorted_path.clear()
        self.__sorted_path = path_tmp    

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
        print("Preparing path...")
        self.__wayOfDetect()
        distance = (self.__IMAGE_MAX_X / self.__w_count) * self.__HOW_OFTEN * 3.0
        self.__path = self.__groupPoints(self.__path, distance + .2)
        distance = self.__findCommonDistanceBetweenPoints(self.__path) 
        self.__path = self.__groupPoints(self.__path, distance + .2)
        self.__sortPoints(self.__path)
        self.__correctStartPointToPath()
        self.__findWrongPoints() 
        self.__calculateYaw()
        print("Finished!")

    def __wayOfDetect(self) -> None:
        self.__createMasks()
        height, width = self.__img_color.shape[:2]

        square_a = width / self.__w_count
        square_b = height / self.__h_count

        self.__diff_square = np.abs(square_a - square_b)

        if self.__diff_square > 1.0:
            return

        # width and height to center of square
        self.__square_cx = square_a / 2.0
        self.__square_cy = square_b / 2.0

        # create points to check
        
        for h in tqdm(range(int(self.__square_cy), int(height), int(self.__HOW_OFTEN * self.__square_cy))):
            for w in range(int(self.__square_cx), int(width), int(self.__HOW_OFTEN * self.__square_cx)):
                point = [h, w]
                color = self.__witchColor(point)
                if str(color) == "[0, 0, 0]":
                    continue
                self.__img_color[h, w] = (0, 0, 0)
                point3d = Point3D(w, h, color)
                if DEBUG_MODE:
                    point3d.showPoint()
                self.__path.append(point3d.getPoint3DArray())
        cv2.imwrite('roi2.png', self.__img_color)
        if DEBUG_MODE:
            print("trajectory len: ", len(self.__path))
            cv2.imwrite("res.png", self.__img_color)

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
        
        if DEBUG_MODE:
            cv2.imshow("RED", self.__red_mask)

    def __createBlueColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__blue_lower, self.__blue_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__blue_lower_1, self.__blue_upper_1)
        mask = mask_0 + mask_1
        self.__blue_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        
        if DEBUG_MODE:
            cv2.imshow("BLUE", self.__blue_mask)

    def __createGreenColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__green_lower, self.__green_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__green_lower_1, self.__green_upper_1)
        mask = mask_0 + mask_1
        self.__green_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        
        if DEBUG_MODE:
            cv2.imshow("GREEN", self.__green_mask)

    def __createYellowColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__yellow_lower, self.__yellow_upper)
        mask_1 = cv2.inRange(self.__img_hsv, self.__yellow_lower_1, self.__yellow_upper_1)
        mask = mask_0 + mask_1
        self.__yellow_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        
        if DEBUG_MODE:
            cv2.imshow("YELLOW", self.__yellow_mask)

    def __createMasks(self) -> None:
        self.__createYellowColorMask()
        self.__createRedColorMask()
        self.__createBlueColorMask()
        self.__createGreenColorMask()

    def __normalize(self, arr, t_min, t_max):
        norm_arr = []
        diff = t_max - t_min
        diff_arr = max(arr) - min(arr)
        
        if diff_arr == 0.0:
            for i in arr:
                norm_arr.append(float(0.0))
        else:
            for i in arr:
                temp = (((i - min(arr)) * diff) / diff_arr) + t_min
                norm_arr.append(float(temp))
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
            x_a.append(float(point3d[0]))
            y_a.append(float(point3d[1]))
            z_a.append(float(point3d[2]))
            pitch_a.append(float(point3d[3]))
            roll_a.append(float(point3d[4]))
            yaw_a.append(float(point3d[5]))
            is_visited_a.append(point3d[6])

        # normalize x and y
        x_a = self.__normalizeArray(x_a, "x")
        y_a = self.__normalizeArray(y_a, "y")
        
        return x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a
