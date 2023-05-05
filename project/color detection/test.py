import cv2
import numpy as np
from robot import Robot
import time

robot = Robot()

def gstreamer_pipeline(
    w=640,
    h=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                w,
                h,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

#HSV tracking
#cv2.namedWindow("Tracking")
#cv2.createTrackbar("Hmin", "Tracking", 0, 255, nothing)
#cv2.createTrackbar("Hmax", "Tracking", 255, 255, nothing)
#cv2.createTrackbar("Smin", "Tracking", 0, 255, nothing)
#cv2.createTrackbar("Smax", "Tracking", 255, 255, nothing)
#cv2.createTrackbar("Vmin", "Tracking", 0, 255, nothing)
#cv2.createTrackbar("Vmax", "Tracking", 255, 255, nothing)

def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("frame", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            ret, frame = cap.read()
            #cv2.imshow('frame', frame)
            
            font = cv2.FONT_ITALIC
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            #hmin = cv2.getTrackbarPos("Hmin", "Tracking")
            #hmax = cv2.getTrackbarPos("Hmax", "Tracking")
            #smin = cv2.getTrackbarPos("Smin", "Tracking")
            #smax = cv2.getTrackbarPos("Smax", "Tracking")
            #vmin = cv2.getTrackbarPos("Vmin", "Tracking")
            #vmax = cv2.getTrackbarPos("Vmax", "Tracking")

            #lowerbounds = np.array([hmin, smin, vmin])
            #upperbounds = np.array([hmax, smax, vmax])

            lower_bounds = np.array([54, 20, 0])
            upper_bounds = np.array([85, 255, 255])

            mask = cv2.inRange(hsv, lower_bounds, upper_bounds)
            colors = cv2.bitwise_and(frame, frame, mask=mask)
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 60, param1=50, param2=10, minRadius=8, maxRadius=14)
            
            if circles == True:

                circles = np.uint16(np.around(circles))

                for i in circles[0, :]:
                    cv2.circle(frame, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                    cv2.circle(mask, (i[0], i[1]), i[2]+130, (255, 255, 255), 2)
                    cv2.putText(frame, 'colors', (i[0], i[1]), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                
                robot.set_motors(0.2,0.2)
                
            else:
                robot.set_motors(0,0)
            cv2.imshow('frame', res)
            
            
        cv2.destroyAllWindows()

if __name__ == "__main__":
    show_camera()

