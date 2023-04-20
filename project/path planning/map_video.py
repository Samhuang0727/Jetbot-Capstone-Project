import cv2
import numpy as np
import cv2.aruco as aruco
from math import sqrt, pow, atan, acos, pi, atan2
from numpy.linalg import inv, norm
with np.load('calibration.npz') as file:
    mtx, dist, rvecs, tvecs = [file[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

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

def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('demo.avi', fourcc, 30.0, (640, 480))
    if camera.isOpened():
        window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("frame", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            ret ,frame = camera.read()
            cv2.imshow('frame', frame)
            out.write(frame)
        out.release()
        camera.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == "__main__":
    show_camera()