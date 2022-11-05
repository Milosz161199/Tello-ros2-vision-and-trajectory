from TrajectoryDetector import *
from Point3D import *
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')

    point3D = Point3D(2, 3, [255, 0, 0])
    point3D.showPoint()

    img = cv2.imread('page.png')
    trajectoryDetector = TrajectoryDetector(img)
    trajectoryDetector.detectColors()
    print(trajectoryDetector.trajectory)

    x = list()
    y = list()
    z = list()

    for point3d in trajectoryDetector.getTrajectory():
        x.append(point3d['x'])
        y.append(point3d['y'])
        z.append(point3d['z'])

    # ax.plot3D(x, y, z, 'gray')

    # cv2.imshow("WIN", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    plt.show()
