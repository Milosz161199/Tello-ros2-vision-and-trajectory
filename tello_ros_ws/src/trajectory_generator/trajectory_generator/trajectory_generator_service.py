from trajectory_interfaces_msgs.srv import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
# from PathDetector import PathDetector
import numpy as np
import time


class PathGeneratorService(Node):

    def __init__(self):
        super().__init__('path_generator_service')
        self.__cv2_bridge = CvBridge()
        self.__path_detector = None

        self.srv = self.create_service(Path, 'path_generator', self.generator_callback)

        self.__image = []
        self.__image_sub_node = None
        self.__image_topic = 'video_frames'

    def generator_callback(self, request, response):
        if request.start:
            self.__image_sub_node = rclpy.create_node('image_subscriber')
            subscription = self.__image_sub_node.create_subscription(Image, self.__image_topic, self.__listener_callback, 10)
            subscription  # prevent unused variable warning
            rclpy.spin_once(self.__image_sub_node)

            while not(len(self.__image)):
                print("No image! Retry...")
                subscription = self.__image_sub_node.create_subscription(Image, self.__image_topic, self.__listener_callback, 10)
                subscription  # prevent unused variable warning
                rclpy.spin_once(self.__image_sub_node)
                time.sleep(1000)
            
            self.__image_sub_node.destroy_node()

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

            # # Display image
            # cv2.imshow("Image", self.__image)
            # cv2.waitKey(0)

            response.number_of_points = len(x_a)
            response.x = list(x_a)
            response.y = list(y_a)
            response.z = list(z_a)
            response.pitch = list(pitch_a)
            response.roll = list(roll_a)
            response.yaw = list(yaw_a)
            response.is_visited = list(is_visited_a)

            return response

    def __validateData(self, x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a) -> bool:
        if len(x_a) == len(y_a) == len(z_a) == len(pitch_a) == len(roll_a) == len(yaw_a) == len(is_visited_a):
            return True
        else:
            return False

    def __listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.__image_sub_node.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        self.__image = self.__cv2_bridge.imgmsg_to_cv2(data)
        

def main(args=None):
    rclpy.init(args=args)

    path_generator_service = PathGeneratorService()

    rclpy.spin(path_generator_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


class Colors:
    def __init__(self):
        self.red_color = [255, 0, 0]
        self.green_color = [0, 255, 0]
        self.blue_color = [0, 0, 255]
        self.black_color = [0, 0, 0]
        self.yellow_color = [255, 255, 0]

        self.START_Z = 100

        self.if_red_value = self.START_Z + 20
        self.if_green_value = self.START_Z + 30
        self.if_blue_value = self.START_Z + 40
        self.if_black_value = self.START_Z + 50
        self.if_yellow_value = self.START_Z + 60

        self.COLORS_ARRAY = [(self.red_color, self.if_red_value),
                             (self.green_color, self.if_green_value),
                             (self.blue_color, self.if_blue_value),
                             (self.black_color, self.if_black_value),
                             (self.yellow_color, self.if_yellow_value)]

    def getColors(self):
        return self.COLORS_ARRAY


class PathDetector:
    def __init__(self, image):
        self.__img_color = image
        self.__img_hsv = cv2.cvtColor(self.__img_color, cv2.COLOR_BGR2HSV)
        # self.__img_color = cv2.cvtColor(self.__img_color, cv2.COLOR_BGR2RGB)

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
        self.__CLASSROOM_MIN_Y = 0.0
        self.__CLASSROOM_MAX_Y = 400.0
        self.__CLASSROOM_MIN_X = 0.0
        self.__CLASSROOM_MAX_X = 800.0

        self.__start_point = Point3D(0, 0, [0, 0, 0])

        self.__path = []
        self.__sorted_path = [self.__start_point.getPoint3DArray()]

        self.__blue_lower = np.array([94, 80, 2])
        self.__blue_upper = np.array([120, 255, 255])

        self.__red_lower_0 = np.array([0, 50, 50])
        self.__red_upper_0 = np.array([10, 255, 255])
        self.__red_lower_1 = np.array([170, 50, 50])
        self.__red_upper_1 = np.array([180, 255, 255])

        self.__green_lower = np.array([36, 25, 25])
        self.__green_upper = np.array([70, 255, 255])

        self.__yellow_lower = np.array([19, 107, 89])
        self.__yellow_upper = np.array([35, 255, 255])

        self.__green_mask = None
        self.__yellow_mask = None
        self.__red_mask = None
        self.__blue_mask = None

    def scaleValueIntoWorldPoint(self, value, ax) -> int:
        if str(ax) == "x":
            n_value = self.remapValue(value,
                                      self.__IMAGE_MIN_X,
                                      self.__IMAGE_MAX_X,
                                      self.__PAPER_MIN_X,
                                      self.__PAPER_MAX_X)
            return self.remapValue(n_value,
                                   self.__PAPER_MIN_X,
                                   self.__PAPER_MAX_X,
                                   self.__CLASSROOM_MIN_X,
                                   self.__CLASSROOM_MAX_X)
        elif str(ax) == "y":
            n_value = self.remapValue(value,
                                      self.__IMAGE_MIN_Y,
                                      self.__IMAGE_MAX_Y,
                                      self.__PAPER_MIN_Y,
                                      self.__PAPER_MAX_Y)
            return self.remapValue(n_value,
                                   self.__PAPER_MIN_Y,
                                   self.__PAPER_MAX_Y,
                                   self.__CLASSROOM_MIN_Y,
                                   self.__CLASSROOM_MAX_Y)
        else:
            print("Wrong ax!")
            return 0

    def remapValue(self, x, oMin, oMax, nMin, nMax) -> int:
        # range check
        if oMin == oMax:
            print("Warning: Zero input range")
            return 0

        if nMin == nMax:
            print("Warning: Zero output range")
            return 0

        # check reversed input range
        reverse_input = False
        old_min = min(oMin, oMax)
        old_max = max(oMin, oMax)
        if not old_min == oMin:
            reverse_input = True

        # check reversed output range
        reverse_output = False
        new_min = min(nMin, nMax)
        new_max = max(nMin, nMax)
        if not new_min == nMin:
            reverse_output = True

        portion = (x - old_min) * (new_max - new_min) / (old_max - old_min)
        if reverse_input:
            portion = (old_max - x) * (new_max - new_min) / (old_max - old_min)

        result = portion + new_min
        if reverse_output:
            result = new_max - portion

        return int(result)

    def getPath(self):
        return self.__sorted_path

    def sortPoints(self) -> None:
        while self.__path:
            self.__path.sort(key=lambda point3d: np.sqrt(
                (point3d[0] - self.__sorted_path[-1][0]) ** 2 + (
                        point3d[1] - self.__sorted_path[-1][1]) ** 2 + (
                        point3d[2] - self.__sorted_path[-1][2]) ** 2))
            self.__sorted_path.append(self.__path[0])
            self.__path.pop(0)

    def calculateYaw(self) -> None:
        for i in range(1, len(self.__sorted_path)):
            diff_x = self.__sorted_path[i][0] - self.__sorted_path[i - 1][0]
            diff_y = self.__sorted_path[i][1] - self.__sorted_path[i - 1][1]
            # print("diff_x: ", diff_x, " diff_y: ", diff_y)
            self.__sorted_path[i][5] = float(np.round(np.arctan2(diff_y, diff_x) * 180 / np.pi, 2))
            # print(self.sorted_trajectory[i][5])

    def preparePath(self) -> None:
        self.newWayOfDetect()

        self.__replace_to_close_points()

        self.sortPoints()
        self.calculateYaw()

    def newWayOfDetect(self):
        self.createMasks()

        array_of_points_to_check = list()
        height, width = self.__img_color.shape[:2]
        w_count = 55
        h_count = 38

        cube_a = width / w_count
        cube_b = height / h_count
        print(cube_a, cube_b)

        if abs(cube_a - cube_b) > 1.0:
            return

        # width and height to center of cube
        cube_cx = cube_a / 2.0
        cube_cy = cube_b / 2.0

        # create points to check
        for h in range(int(cube_cy), int(height), 2 * int(cube_cy)):
            for w in range(int(cube_cx), int(width), 2 * int(cube_cx)):
                array_of_points_to_check.append([h, w])
                # print(self.img_color[h, w])
                point = [h, w]
                color = self.witchColor(point)
                print(color)
                if str(color) == "[0, 0, 0]":
                    continue
                self.__img_color[h, w] = (0, 0, 0)
                point3d = Point3D(self.scaleValueIntoWorldPoint(w, "x"),
                                  self.scaleValueIntoWorldPoint(h, "y"),
                                  color)
                self.__path.append(point3d.getPoint3DArray())
        print("trajectory len: ", len(self.__path))
        print(array_of_points_to_check)
        print("len: ", len(array_of_points_to_check))
        print("real len: ", w_count * h_count)
        # del self.__path[::10]
        print("trajectory len: ", len(self.__path))
        # cv2.imwrite("res.png", self.__img_color)

    def isBlue(self, point) -> bool:
        if str(self.__blue_mask[point[0], point[1]]) != "[0 0 0]":
            print("blue: ", self.__blue_mask[point[0], point[1]])
            return True
        else:
            return False

    def isRed(self, point) -> bool:
        if str(self.__red_mask[point[0], point[1]]) != "[0 0 0]":
            print("red: ", self.__red_mask[point[0], point[1]])
            return True
        else:
            return False

    def isGreen(self, point) -> bool:
        if str(self.__green_mask[point[0], point[1]]) != "[0 0 0]":
            print("green: ", self.__green_mask[point[0], point[1]])
            return True
        else:
            return False

    def isYellow(self, point) -> bool:
        if str(self.__yellow_mask[point[0], point[1]]) != "[0 0 0]":
            print("yellow: ", self.__yellow_mask[point[0], point[1]])
            return True
        else:
            return False

    def witchColor(self, point) -> list:
        if self.isBlue(point):
            return [0, 0, 255]
        elif self.isGreen(point):
            return [0, 255, 0]
        elif self.isRed(point):
            return [255, 0, 0]
        elif self.isYellow(point):
            return [255, 255, 0]
        else:
            return [0, 0, 0]

    def createRedColorMask(self) -> None:
        mask_0 = cv2.inRange(self.__img_hsv, self.__red_lower_0, self.__red_upper_0)
        mask_1 = cv2.inRange(self.__img_hsv, self.__red_lower_1, self.__red_upper_1)
        mask = mask_0 + mask_1
        self.__red_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("RED", self.__red_mask)

    def createBlueColorMask(self) -> None:
        mask = cv2.inRange(self.__img_hsv, self.__blue_lower, self.__blue_upper)
        self.__blue_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("BLUE", self.__blue_mask)

    def createGreenColorMask(self) -> None:
        mask = cv2.inRange(self.__img_hsv, self.__green_lower, self.__green_upper)
        self.__green_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("GREEN", self.__green_mask)

    def createYellowColorMask(self) -> None:
        mask = cv2.inRange(self.__img_hsv, self.__yellow_lower, self.__yellow_upper)
        self.__yellow_mask = cv2.bitwise_and(self.__img_color, self.__img_color, mask=mask)
        # cv2.imshow("YELLOW", self.__yellow_mask)

    def createMasks(self) -> None:
        self.createYellowColorMask()
        self.createRedColorMask()
        self.createBlueColorMask()
        self.createGreenColorMask()

    def __replace_to_close_points(self):
        pass

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

        return x_a, y_a, z_a, pitch_a, roll_a, yaw_a, is_visited_a


class Point3D:
    colors = Colors()

    def __init__(self, x, y, color, pitch=0.0, roll=0.0, yaw=0.0, is_visited=False):
        self.x = x
        self.y = y

        self.color = color
        self.z = self.calculateZ()

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.is_visited = is_visited

    def calculateZ(self):
        for c, z in Point3D.colors.getColors():
            if str(c) == str(self.color):
                return z

    def showPoint(self):
        print(self.x, self.y, self.z)

    def getPoint3D(self):
        return {"x": self.x, "y": self.y, "z": self.z, "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
                "is_visited": self.is_visited}

    def getPoint3DArray(self):
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.is_visited]

