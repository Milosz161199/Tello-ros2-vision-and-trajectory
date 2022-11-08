import cv2 as cv
import numpy as np


# Load the webcam
cap = cv.VideoCapture(0)

# Add sliders for canny parameters
cv.namedWindow("Trackbars")
cv.createTrackbar("Min Threshold", "Trackbars", 0, 250, lambda x: None)
cv.createTrackbar("Max Threshold", "Trackbars", 84, 250, lambda x: None)

while True:
    # Read trackbars
    min_threshold = cv.getTrackbarPos("Min Threshold", "Trackbars")
    max_threshold = cv.getTrackbarPos("Max Threshold", "Trackbars")

    # Read the webcam
    # _, img = cap.read()
    img = cv.imread("room_with_grid.png")
    img = cv.resize(img, (0, 0), fx=0.2, fy=0.2)
    # Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    thresh = cv.threshold(gray, min_threshold, 255, cv.THRESH_BINARY_INV)[1]
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)), iterations=1)
    
    dst = cv.cornerHarris(thresh,4,3,0.04)

    # x, y = img[dst>0.01*dst.max()]
    coordinates = np.where(dst > 0.01 * dst.max())
    corners = np.array(list(zip(coordinates[1], coordinates[0])))

    rect = cv.minAreaRect(corners)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, (0, 0, 255), 2)

    x, y, w, h = cv.boundingRect(box)
    roi = img[y:y+h, x:x+w]

    cv.imshow("Corners", img)
    cv.imshow("ROI", roi)
    cv.imshow("Thresh", thresh)


    # Stop if escape key is pressed
    k = cv.waitKey(30) & 0xff
    if k==27:
        break

# Release the webcam
cap.release()

# Close all windows
cv.destroyAllWindows()