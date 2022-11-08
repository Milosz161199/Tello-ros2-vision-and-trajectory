import cv2 as cv
import numpy as np
import time
# from itertools import combinations

# Load the webcam
cap = cv.VideoCapture(0)

# Add sliders for canny parameters
cv.namedWindow("Trackbars")
cv.createTrackbar("Min Threshold", "Trackbars", 1, 250, lambda x: None)
cv.createTrackbar("Max Threshold", "Trackbars", 84, 250, lambda x: None)

previous_contour = None
# start timer
start = time.time()

while True:
    

    # Read trackbars
    min_threshold = cv.getTrackbarPos("Min Threshold", "Trackbars")
    max_threshold = cv.getTrackbarPos("Max Threshold", "Trackbars")

    # Read the webcam
    # _, img = cap.read()
    img = cv.imread("room_with_grid2.png")
    img = cv.resize(img, (0, 0), fx=0.2, fy=0.2)
    # Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray, (5, 5), 0)

    blur = cv.morphologyEx(blur, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)

    canny = cv.Canny(blur, min_threshold, max_threshold)
    canny = cv.morphologyEx(canny, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=2)
    
    contours, hierarchy = cv.findContours(canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    biggest = []
    max_area = 0
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 100:

            peri = cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, 0.02 * peri, True)
            if area > max_area and len(approx) == 4:
                biggest = []
                biggest.append(approx)
                print(approx)
                max_area = area
    
    cv.drawContours(img, biggest, -1, (0, 255, 0), 2)

    cv.imshow("Blur", blur)
    cv.imshow("Canny", canny)
    cv.imshow('img', img)

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