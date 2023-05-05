import cv2
import numpy as np


def nothing(x):
    pass

#HSV tracking
cv2.namedWindow("Tracking")
cv2.createTrackbar("Hmin", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Hmax", "Tracking", 255, 255, nothing)
cv2.createTrackbar("Smin", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Smax", "Tracking", 255, 255, nothing)
cv2.createTrackbar("Vmin", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Vmax", "Tracking", 255, 255, nothing)

while True:
    frame = cv2.imread('redlight.png', 1)
    font = cv2.FONT_HERSHEY_COMPLEX
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hmin = cv2.getTrackbarPos("Hmin", "Tracking")
    hmax = cv2.getTrackbarPos("Hmax", "Tracking")
    smin = cv2.getTrackbarPos("Smin", "Tracking")
    smax = cv2.getTrackbarPos("Smax", "Tracking")
    vmin = cv2.getTrackbarPos("Vmin", "Tracking")
    vmax = cv2.getTrackbarPos("Vmax", "Tracking")

    lowerbounds = np.array([hmin, smin, vmin])
    upperbounds = np.array([hmax, smax, vmax])

    mask = cv2.inRange(hsv, lowerbounds, upperbounds)

    res = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    trash, bin = cv2.threshold(gray, 227, 255, 1, cv2.THRESH_BINARY)

    # Detecting contours in image.
    contours, _ = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Going through every contours found in the image.
    for cnt in contours:

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

        # draws boundary of contours.
        cv2.drawContours(frame, [approx], 0, (0, 0, 255), 2)

        # Used to flatted the array containing
        # the co-ordinates of the vertices.
        n = approx.ravel()
        i = 0

        for j in n:
            if (i % 2 == 0):
                x = n[i]
                y = n[i + 1]

                # String containing the co-ordinates.
                string = str(x) + " " + str(y)

                if (i == 0):
                    # text on topmost co-ordinate.
                    cv2.putText(frame, "Arrow tip", (x, y),
                                font, 0.5, (255, 0, 0))
                else:
                    # text on remaining co-ordinates.
                    cv2.putText(frame, string, (x, y),
                                font, 0.5, (0, 255, 0))
            i += 1

    cv2.namedWindow('res', cv2.WINDOW_NORMAL)
    cv2.imshow("res", res)

    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()