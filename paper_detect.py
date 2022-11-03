import cv2 as cv
import numpy as np
# open cv sheet of paper detection using contours on a webcam

# Load the webcam
cap = cv.VideoCapture(0)

# Add sliders for canny parameters
cv.namedWindow("Trackbars")
cv.createTrackbar("Min Threshold", "Trackbars", 60, 250, lambda x: None)
cv.createTrackbar("Max Threshold", "Trackbars", 200, 250, lambda x: None)

while True:
    # Read trackbars
    min_threshold = cv.getTrackbarPos("Min Threshold", "Trackbars")
    max_threshold = cv.getTrackbarPos("Max Threshold", "Trackbars")

    # Read the webcam
    _, img = cap.read()
    # Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Blur the image
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel, iterations=3)

    # mask = np.zeros(img.shape[:2],np.uint8)
    # bgdModel = np.zeros((1,65),np.float64)
    # fgdModel = np.zeros((1,65),np.float64)
    # rect = (20,20,img.shape[1]-20,img.shape[0]-20)
    # cv.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv.GC_INIT_WITH_RECT)
    # mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    # img = img*mask2[:,:,np.newaxis]

    # Apply Canny edge detection
    canny = cv.Canny(img, min_threshold, max_threshold)
    canny = cv.dilate(canny, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)), iterations=3)


    # Close the gaps between edges
    
    closed = cv.morphologyEx(canny, cv.MORPH_CLOSE, kernel, iterations=10)

    # Find contours
    contours, _ = cv.findContours(closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Find the biggest contour
    biggest = []
    max_area = 0
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 100:
            peri = cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, 0.05 * peri, True)
            if area > max_area and len(approx) == 4:
                biggest.append(contour)
                print(approx)
                max_area = area
    
    # Draw contours
    # cv.drawContours(img, contours, -1, (0, 255, 0), 2)

    # Draw the biggest contour
    cv.drawContours(img, biggest, -1, (0, 0, 255), 3)

    # Show the image
    cv.imshow("Image", img)
    cv.imshow("canny", canny)

    # Stop if escape key is pressed
    k = cv.waitKey(30) & 0xff
    if k==27:
        break

# Release the webcam
cap.release()

# Close all windows
cv.destroyAllWindows()