import cv2
import numpy as np

import warnings

warnings.filterwarnings("ignore")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if ret == True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([54, 20, 20])
        upper_red = np.array([85, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        img = cv2.medianBlur(res, 5)
        ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=20, maxRadius=30)
        if circles is not None:
            print("circle is found")
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
        cv2.imshow('detected circles', cimg)
        cv2.imshow('res', res)
    else:
        print("Read Failed")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()