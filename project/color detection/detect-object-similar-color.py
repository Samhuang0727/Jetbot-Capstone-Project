
import cv2
import numpy as np

# HSV trackbar
def nothing(x):
    pass
cv2.namedWindow("Trackbar")
cv2.createTrackbar("Hmin","Trackbar",0,255,nothing)
cv2.createTrackbar("Hmax","Trackbar",255,255,nothing)
cv2.createTrackbar("Smin","Trackbar",0,255,nothing)
cv2.createTrackbar("Smax","Trackbar",255,255,nothing)
cv2.createTrackbar("Vmin","Trackbar",0,255,nothing)
cv2.createTrackbar("Vmax","Trackbar",255,255,nothing)

while True:
    img=cv2.imread('image.jpg',1)
    font=cv2.FONT_HERSHEY_COMPLEX
    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hmin=cv2.getTrackbarPos("Hmin","Trackbar")
    hmax=cv2.getTrackbarPos("Hmax","Trackbar")
    smin=cv2.getTrackbarPos("Smin","Trackbar")
    smax=cv2.getTrackbarPos("Smin","Trackbar")
    vmin=cv2.getTrackbarPos("Vmin","Trackbar")
    vmax=cv2.getTrackbarPos("Vmin","Trackbar")

    lower_bound=np.array([hmin,smin,vmin])
    upper_bound=np.array([hmin,smin,vmin])
    mask=cv2.inRange(hsv,lower_bound,upper_bound)
    
    res=cv2.bitwise_and(img,img,mask=mask)
    gray=cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    trash,bin=cv2.threshold(gray,227,255,1,cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Going through every contours found in the image.
    for cnt in contours:

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

        # draws boundary of contours.
        cv2.drawContours(img, [approx], 0, (0, 0, 255), 2)

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
                    cv2.putText(img, "Arrow tip", (x, y),
                                font, 0.5, (255, 0, 0))
                else:
                    # text on remaining co-ordinates.
                    cv2.putText(img, string, (x, y),
                                font, 0.5, (0, 255, 0))
            i += 1
    
    # Showing the output
    cv2.imshow("res", res)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
    
cv2.destroyAllWindows()
