from TrajectoryDetector import *
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig = plt.figure()
    # fig1 = plt.figure()
    ax = plt.axes(projection='3d')
    # ax1 = plt.axes()

    img = cv2.imread('test02.png')
    scale_percent = 50  # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    trajectoryDetector = TrajectoryDetector(img)
    trajectoryDetector.prepareTrajectory()
    print(trajectoryDetector.getTrajectory())

    x = list()
    y = list()
    z = list()

    for point3d in trajectoryDetector.getTrajectory():
        x.append(point3d[0])
        y.append(point3d[1])
        z.append(point3d[2])

    ax.plot3D(x, y, z, 'gray')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # ax1.plot(x, y)
    # ax1.set_xlabel('X-axis')
    # ax1.set_ylabel('Y-axis')

    cv2.imshow("WIN", img)
    cv2.waitKey(1000)
    plt.show()
    cv2.destroyAllWindows()
