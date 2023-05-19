import cv2
import numpy as np
import time
import timeit
from robot import Robot

robot = Robot()

while 1:
    robot.stop()
    dir =int(input())
    if dir == 1: #turn left
        robot.set_motors(0.17,0.24)
        time.sleep(3.5)
        #robot.set_motors(0.101,0.108)
        #time.sleep(2)
        robot.set_motors(0, 0)
        print('turning left....')
    elif dir == 2: #turn right
        robot.set_motors(0.19,0.24)
        time.sleep(0.5)
        robot.set_motors(0.3,0.13)
        time.sleep(1.7)
        robot.set_motors(0.19,0.26)
        time.sleep(0.7)
        robot.set_motors(0, 0)
        print('turning right....')
    else: #go straight
        robot.set_motors(0.2,0.192)
        time.sleep(83)
        robot.set_motors(0, 0)
        print('go straight....')
