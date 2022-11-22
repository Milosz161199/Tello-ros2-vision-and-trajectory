import cv2 as cv
import numpy as np
import time
# from itertools import combinations
import matplotlib.pyplot as plt
from imutils.perspective import order_points

def perspective_transformation(img, box):
  box = np.array(box, dtype="int")
  src_pts = order_points(box)

  # use Euclidean distance to get width & height
  width = int(np.linalg.norm(src_pts[0] - src_pts[1]))
  height = int(np.linalg.norm(src_pts[0] - src_pts[3]))

  dst_pts = np.array([[0,0], [width,0], [width,height], [0,height]], dtype=np.float32)

  M = cv.getPerspectiveTransform(src_pts, dst_pts)
  warped_img = cv.warpPerspective(img, M, (width, height))

  return warped_img

# Load the webcam    
cap = cv.VideoCapture(0)
_, img = cap.read()


# Add sliders for canny parameters
cv.namedWindow("Trackbars")
cv.createTrackbar("Min Threshold", "Trackbars", 30, 250, lambda x: None)
cv.createTrackbar("Max Threshold", "Trackbars", 50, 250, lambda x: None)



previous_contour = None
# start timer
start = time.time()
pts1 = np.float32([[360,50],[2122,470],[2264, 1616],[328,1820]])


while True:

    # Read trackbars
    min_threshold = cv.getTrackbarPos("Min Threshold", "Trackbars")
    max_threshold = cv.getTrackbarPos("Max Threshold", "Trackbars")

    # Read the webcam
    _, img = cap.read()
    # img = cv.imread("room_with_grid2.png")
    img = cv.resize(img, (0, 0), fx=0.4, fy=0.4)
    # Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray, (9, 9), 0)

    # blur = cv.morphologyEx(blur, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)

    canny = cv.Canny(blur, min_threshold, max_threshold)
    # canny = cv.morphologyEx(canny, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)
    
    contours, hierarchy = cv.findContours(canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    biggest = None
    roi = None

    max_area = 0
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 100:

            peri = cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, 0.02 * peri, True)
            if area > max_area and len(approx) == 4:
                # biggest = []
                bingo = approx.reshape(4,2)
                biggest = contour
                # print(approx)
                max_area = area
    
    

    if biggest is not None:
        cv.drawContours(img, biggest, -1, (0, 255, 0), 2)
        rect = cv.minAreaRect(biggest)
        box = cv.boxPoints(rect)
        print(box)
        box = np.int0(box)
        cv.drawContours(img, [box], 0, (0, 0, 255), 2)

    elapsed = time.time() - start
    # if elapsed time is greater than 1 second
    if elapsed > 4 and previous_contour is not None:
        # calculate the difference between the current contour and the previous contour
        difference = cv.matchShapes(box, previous_contour, 2, 0.0)
        print(difference)
        # reset timer
        if difference < 0.1:
            print("Paper detected")
        start = time.time()
        # set previous contour to current contour
        previous_contour = box

    if previous_contour is None and biggest is not None:

        previous_contour = box

    
    if biggest is not None:
        x, y, w, h = cv.boundingRect(box)
        roi = img[y:y+h, x:x+w]

    cv.imshow("Blur", blur)
    cv.imshow("Canny", canny)
    cv.imshow('img', img)
    if roi is not None:
        roi = perspective_transformation(img, bingo)
        cv.imshow('roi', roi)

    

    # thresh = cv.threshold(gray, min_threshold, 255, cv.THRESH_BINARY_INV)[1]
    # thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=1)
    
    # dst = cv.cornerHarris(thresh,4,3,0.04)

    # # x, y = img[dst>0.01*dst.max()]
    # coordinates = np.where(dst > 0.01 * dst.max())


    # # draw corners
    # corners = np.array(list(zip(coordinates[1], coordinates[0])))   

    # # match two points in corners
    # # if they are not on an approximately horizontal or vertical line, draw a line between them

    # # comb = list(combinations(corners, 2))
    # # for i in comb:
    # #     if abs(i[0][0] - i[1][0]) < 10 or abs(i[0][1] - i[1][1]) < 10:
    # #         cv.line(img, i[0], i[1], (0, 0, 255), 2)

    
    # for corner in corners:
    #     cv.circle(img, tuple(corner), 5, (0, 0, 255), -1)


    # rect = cv.minAreaRect(corners)
    # box = cv.boxPoints(rect)
    # box = np.int0(box)
    # cv.drawContours(img, [box], 0, (0, 0, 255), 2)

    # # calculate elapsed time
    # elapsed = time.time() - start

    # # if elapsed time is greater than 1 second
    # if elapsed > 4 and previous_contour is not None:
    #     # calculate the difference between the current contour and the previous contour
    #     difference = cv.matchShapes(box, previous_contour, 2, 0.0)
    #     print(difference)
    #     # reset timer
    #     if difference < 0.1:
    #         print("Paper detected")
    #     start = time.time()
    #     # set previous contour to current contour
    #     previous_contour = box

    # if previous_contour is None:
    #     previous_contour = box

    # x, y, w, h = cv.boundingRect(box)
    # roi = img[y:y+h, x:x+w]



    # cv.imshow("Corners", img)
    # cv.imshow("ROI", roi)
    # cv.imshow("Thresh", thresh)



    


    # Stop if escape key is pressed
    k = cv.waitKey(30) & 0xff
    if k==27:
        break
    


# Release the webcam
cap.release()

# Close all windows
cv.destroyAllWindows()