from TrajectoryDetector import *
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    img = cv2.imread('Grid_trajectory.png')
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

    ax.scatter(x, y, z, 'gray')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    cv2.imshow("WIN", img)
    # cv2.waitKey(0)
    plt.show()
    cv2.destroyAllWindows()
