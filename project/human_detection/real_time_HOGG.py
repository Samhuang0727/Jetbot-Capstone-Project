import cv2
import numpy as np
import time
import timeit
from robot import Robot

robot = Robot()
""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    window_title = "CSI Camera"
    
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    # the output will be written to output.avi
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('demo.avi', fourcc, 30.0, (640, 480))
    leave = False
    if cap.isOpened():
        window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
        # initialize the HOG descriptor/person detector
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Capture frame-by-frame
            ret, frame = cap.read()
            print("detecting")
            # resizing for faster detection
            frame = cv2.resize(frame, (320, 240))
            # using a greyscale picture, also for faster detection
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # # detect people in the image
            # # returns the bounding boxes for the detected objects
            boxes, weights = hog.detectMultiScale(gray, winStride=(8,8))
    
            boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

            for (xA, yA, xB, yB) in boxes:
                # display the detected boxes in the colour picture
                cv2.rectangle(frame, (xA, yA), (xB, yB),(0, 255, 0), 2)
                print(abs(xA-xB))
                if  abs(xA-xB) > 75:
                    leave = True
                    break
            if leave == True:
                break        
            # Display the resulting frame
            #frame = cv2.resize(frame, (640, 480))                    
            cv2.imshow('CSI Camera',frame)

        # When everything is done, release the capture
        cap.release()
        # and release the output
        # out.release()
        # finally, close the window
        cv2.destroyAllWindows()

        if leave == True:    
            robot.set_motors(0.16,0.25)
            time.sleep(1)
            robot.stop()
            robot.set_motors(0.3,0.1)
            time.sleep(1.7)
            robot.set_motors(0,0)
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()



# cv2.startWindowThread()





