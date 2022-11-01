from TrajectoryDetector import *
from Point3D import *


if __name__ == "__main__":

    point3D = Point3D(2, 3, [255, 0, 0])
    point3D.showPoint()

    img = cv2.imread('page.png')
    cv2.imshow("WIN", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
