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

def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray

def aruco_detect(gray):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #return corners , ids
    back = None
    if ids is not None:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        for i in range(len(ids)):
            x = tvec[i, 0, 0]
            y = tvec[i, 0, 1]
            z = tvec[i, 0, 2]
            R = cv2.Rodrigues(rvec[i])[0]
            # euler angle
#                 tx = atan2(R[2, 1], R[2, 2])
#                 ty = atan2(-R[2, 0], sqrt(pow(R[2, 1], 2) + pow(R[2, 2], 2)))
            tz = np.rad2deg(atan2(R[1, 0], R[0, 0]))
#                 angle = np.rad2deg(np.array([tx, ty, tz]))
#                 print(tz)
            dis = sqrt(pow(x,2)+pow(y,2)+pow(z,2))*100 + 3
            # modified distance
            dis = 0.0084*pow(dis, 2) + 0.5877*dis - 2.1246
#                 print('id = ', ids[i], dis, '(cm)')
            id = int(ids[i])
            if (0 <= id < 22) & (tz < 90):
                back = np.array([id, dis])
#                 distance[i, :] = np.array([id, dis, tz])
#         distance = np.round(distance, 1)
    return back



def show_camera():
    print(gstreamer_pipeline(flip_method=0))
    camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('demo.avi', fourcc, 30.0, (640, 480))
    print(camera.isOpened())
    if camera.isOpened():
        window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        while cv2.getWindowProperty("frame", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            ret ,frame = camera.read()
            cv2.imshow('frame', frame)
            gray = preprocess(frame)
            back = aruco_detect(gray)
            print(back)
        out.release()
        camera.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == "__main__":
    show_camera()



   