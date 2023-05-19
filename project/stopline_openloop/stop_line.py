import cv2
import numpy as np
import time
import timeit
from robot import Robot

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

def stop_line_detect(frame):
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower = np.array([51, 107, 139])
        upper = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        mask = mask[450:480, :]
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (320, 4))
        mask = cv2.dilate(mask,None, iterations=2)
#         cv2.imshow('mask', mask)
        mask = cv2.erode(mask, kernel, iterations=1)
        
        num = np.transpose(np.nonzero(mask))
        if len(num) > 800:
            detect = True
        else:
            detect = False
        return detect


def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('demo.avi', fourcc, 30.0, (640, 480))
    print(camera.isOpened())
    if camera.isOpened():
        window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("frame", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            ret ,frame = camera.read()
            cv2.imshow('frame', frame)
            detect = stop_line_detect(frame)
            if detect == True:
                robot.set_motors(0.2, 0.192)
                time.sleep(0.5)
                robot.set_motors(0,0)
                print('stop!!!!!!!')
                break
            else:
                robot.set_motors(0.2,0.192)
                print('go straight....')
        out.release()
        camera.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == "__main__":
    show_camera()