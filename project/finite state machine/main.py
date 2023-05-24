from transitions import Machine
import time
import cv2
import numpy as np
from function import detection
from lane_detect import lane
from A_star_class import AStar
import pid_1 as Pv
import pid_2 as PID
import cubic_spline
import timeit
from robot import Robot
import random
from numpy.linalg import inv, norm
from math import acos, pi, atan
import yaml

with open('calibration.yaml', 'r') as f:
    data = yaml.load(f, Loader=yaml.CLoader)
    cameraMatrix = data['camera_matrix']
    dist = data['dist_coeff']
    camera_matrix = np.array(cameraMatrix)
    dist_coeff = np.array(dist)

class FSM(object):
    # define states
    states = ['path_planning', 'traffic_light', 'openloop_motion', 'lane_detection', 'avoid_people']
    # main code
    def __init__(self, robot):
        self.robot = robot
        self.machine = Machine(model=self, states=FSM.states, initial='path_planning')
        self.machine.add_transition(trigger='find_path', source='path_planning', dest='traffic_light', before='stop_jetbot')
        self.machine.add_transition(trigger='green_light', source='traffic_light', dest='openloop_motion', before='stop_jetbot')
        self.machine.add_transition(trigger='pass_intersection', source='openloop_motion', dest='lane_detection')
        self.machine.add_transition(trigger='stop_line', source='lane_detection', dest='path_planning', after='stop_jetbot')
        self.machine.add_transition(trigger='detect_people', source='*', dest='avoid_people')
        self.machine.add_transition(trigger='avoid_success', source='avoid_people', dest='lane_detection', after='stop_jetbot')
        

    def stop_jetbot(self):
        r = self.robot
        r.set_motors(0, 0)
        time.sleep(1)

    def stopline_jetbot(self):
        r = self.robot
        r.set_motors(0.2, 0.192)
        time.sleep(0.5)
        r.set_motors(0,0)
        time.sleep(1)
        print('stopline_stop')

def show_state(fsm):
    print('current state: ', fsm.state)

robot = Robot()
camera = cv2.VideoCapture(detection.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
file1path = '/home/jetbot/Capstone-Project/project/path planning/aruco_marker/aruco_param.npz'
filepath =  '/home/jetbot/Capstone-Project/project/lane_following/calibration.npz'
file = np.load(filepath)
file1 = np.load(file1path)
fsm = FSM(robot)
# define parameter
direction = None
green_light = True
current_id = 8
id_distance = None
detect_people = None
people = 0
traffic_time = 0
window_handle = cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
#variables for lane following
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (640, 480), 1, (640, 480))
roi_lane = np.array([[0, 480], [640, 480], [640, 250], [0, 250]])
dst_pts = np.float32([[0, 400], [400, 400], [400, 0], [0, 0]])
M = cv2.getPerspectiveTransform(np.float32(roi_lane), dst_pts)
Minv = inv(M)

while cv2.getWindowProperty("frame", 0) >= 0:
    ret, frame = camera.read()
    d = detection(frame, file)
    state = fsm.state

    # detect aruco
    undistorted_image = d.undistortion(frame)
    gray = d.preprocess(frame)
    parameter = d.aruco_detect(gray)
#     print('parameter: ', parameter)
    if parameter is not None:
        current_id = int(parameter[0])
        id_distance = parameter[1]
    
    
    # detect people
    if people == 0:
        detect_people = False
        detect_people = d.HOG_detect()
        print('detect people: ', detect_people)
        if detect_people == True:
            fsm.detect_people()
            people += 1
    
    booll = [True, False]
    n = random.randint(0, 1)
    # show current FSM state
    show_state(fsm)

    # define 
    if state == 'path_planning':
        print('-----------------------')
        print('start path planning....')
        smooth = True

        #     digital map preprocessing
        img = cv2.flip(cv2.imread("map_v5.jpg"), 0)
        img[img > 128] = 255
        img[img <= 128] = 0
        m = np.asarray(img)
        m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
        m = m.astype(float) / 255.
        m = 1 - cv2.dilate(1 - m, np.ones((20, 20)))
        img = img.astype(float) / 255.

        #     Aruco marker input
        parameter = file1['parameter']
        print('current_id: ', current_id)
        id = current_id 
        face = parameter[:, 1]
        coordinate = parameter[:, 2:4]
        face1 = face[id]
        coor = coordinate[id]
        start = (coor[0], coor[1])
        goal = (570, 20)

        a = timeit.default_timer()
        astar = AStar(m)
        path = astar.planning(start=start, goal=goal, img=img, inter=22)
        print('path[0]: ',path[0])
        print('path[1]: ',path[1])

        # determine Jetbot direction
        p_0 = path[0]
        p_1 = path[1]

        planning, dir = astar.decide_direction(p_0, p_1, face1)
        print('direction:', dir)

        b = timeit.default_timer()
        print("Time: ", b - a)

        cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
        cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)

        # Extract Path
        if not smooth:
            for i in range(len(path) - 1):
                cv2.line(img, path[i], path[i + 1], (1, 0, 0), 2)
        else:
            path = np.array(cubic_spline.cubic_spline_2d(path, interval=1))
            for i in range(len(path) - 1):
                cv2.line(img, cubic_spline.pos_int(path[i]), cubic_spline.pos_int(path[i + 1]), (1, 0, 0), 1)

        img_ = cv2.flip(img, 0)
        fsm.find_path()

    elif state == 'traffic_light':
        print('-----------------------')
        print('start traffic light detection....')
        if traffic_time == 0:
            green_light = d.traffic_light_detect()
            if green_light == True:
                traffic_time += 1
        else:
            green_light = True
            
        if green_light == True:
            fsm.green_light()
        print('green light: ', green_light)

    elif state == 'openloop_motion':
        print('-----------------------')
        print('start open loop motion....')
        if dir == 'left':
            robot.set_motors(0.17,0.24)
            time.sleep(3.5)
            robot.set_motors(0, 0)
            print('turning left....')
        elif dir == 'right':
            robot.set_motors(0.19,0.24)
            time.sleep(0.5)
            robot.set_motors(0.3,0.13)
            time.sleep(1.7)
            robot.set_motors(0.19,0.26)
            time.sleep(0.7)
            robot.set_motors(0, 0)
            print('turning right....')
        else:
            robot.set_motors(0.2,0.192)
            time.sleep(3)
            robot.set_motors(0, 0)
            print('go straight....')
        fsm.pass_intersection()

    elif state == 'avoid_people':
        print('-----------------------')
        print('start avoid people....')
        d.avoid_people(robot)
        fsm.avoid_success()
    else:
        print('-----------------------')
        print('start lane detection....')
        stop = False
        stop = d.stop_line_detect(800)
#         if id_distance is not None:
#             if id_distance < 50:
#                 stop = True
        print('detect stop line: ', stop)
        print('id idstance: ', id_distance)
        if stop == True:
            fsm.stop_line()
        else:
            # lane tracking
            l = lane(frame)
            dst = cv2.undistort(frame, camera_matrix, dist_coeff, None, newCameraMatrix)
            x, y, w, h = roi
            dst = dst[y:y + h, x:x + w]
            dst = cv2.resize(dst, (640, 480), interpolation=cv2.INTER_NEAREST)
            perspective_img = cv2.warpPerspective(dst, M, (400, 400))
            pp_img = l.preprocess(perspective_img)
            loc_mid, end_mid = l.find_line(pp_img)
            p_loc, p_end = l.back_perspective(Minv, loc_mid, end_mid)
            loc_line, angle = l.detect_angle(p_loc, p_end, frame)

            if angle > 0:
                angle = 90 - angle
            else:
                angle = -90 - angle
            
            attitude_ctrl = PID.pid(1.5,0,0.2)
            # attitude control
            attitude_ctrl.cur = angle
            # attitude_ctrl.desire = 0
            attitude_ctrl.cal_err()
            r_mcd = attitude_ctrl.output()
            RPSR, RPSL = Pv.ctrl_mixer(r_mcd)
            Pv.motor_ctrl(RPSR,RPSL)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
cv2.destroyAllWindows()