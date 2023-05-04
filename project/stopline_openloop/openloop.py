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
        robot.set_motors(0.11,0.2)
        time.sleep(2.7)
        #robot.set_motors(0.101,0.108)
        #time.sleep(2)
        robot.set_motors(0, 0)
        print('turning left....')
    elif dir == 2: #turn right
        robot.set_motors(0.24,0.11)
        time.sleep(2.7)
        robot.set_motors(0, 0)
        print('turning right....')
    else: #go straight
        robot.set_motors(0.155,0.153)
        time.sleep(3)
        robot.set_motors(0, 0)
        print('go straight....')
