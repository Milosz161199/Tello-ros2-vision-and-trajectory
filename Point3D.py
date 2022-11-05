from Colors import *
import collections


class Point3D:
    colors = Colors()

    def __init__(self, x, y, color):
        self.x = x
        self.y = y

        self.color = color

        self.z = self.calculateZ()
        self.is_visited = False

    def calculateZ(self):
        for color, z in Point3D.colors.getColors():
            if collections.Counter(color) == collections.Counter(self.color):
                return z

    def showPoint(self):
        print(self.x, self.y, self.z)

    def getPoint3D(self):
        return {"x": self.x, "y": self.y, "z": self.z, "is_visited": self.is_visited}
