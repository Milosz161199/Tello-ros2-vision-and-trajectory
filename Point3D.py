from Colors import *


class Point3D:
    colors = Colors()

    def __init__(self, x, y, color, pitch=0, roll=0, yaw=0, is_visited=False):
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
