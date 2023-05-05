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
#hmin = cv2.getTrackbarPos("Hmin", "Tracking")
#hmax = cv2.getTrackbarPos("Hmax", "Tracking")
#smin = cv2.getTrackbarPos("Smin", "Tracking")
#smax = cv2.getTrackbarPos("Smax", "Tracking")
#vmin = cv2.getTrackbarPos("Vmin", "Tracking")
#vmax = cv2.getTrackbarPos("Vmax", "Tracking")
#lower_bound = np.array([hmin, smin, vmin])
#upper_bound = np.array([hmax, smax, vmax])

def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("frame", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            ret, frame = cap.read()
            if not ret:
                print("No detected frame")
                break
            cv2.imshow('frame', frame)

            hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_lower = np.array([25, 52, 72], np.uint8)
            green_upper = np.array([102, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
            kernel = np.ones((5, 5), "uint8")
            green_mask = cv2.dilate(green_mask, kernel)
            res_green = cv2.bitwise_and(frame, frame, mask = green_mask)
            contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            print(contours)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
            
            print(area)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contours)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # if area is not None:
            #     #robot.set_motors(0.2,0.2)
            # else:
            #     #robot.stop()

            cv2.imshow('Result', frame)
            
            
        cv2.destroyAllWindows()

if __name__ == "__main__":
    show_camera()

