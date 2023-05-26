# Capstone Project - Jetbot

## Overview

This is a project developed from the Aerospace Engineering Practice Course of the Department of Aeronautic and Astronautic Engineering at National Cheng Kung University. Our goal is to build a self-driving car, which includes applications such as:

- **traffic light detection**
- **huamn detection**
  - HOG
- **lane following**
  - sliding windows method
  - camera calibration
  - PID control
- **path planning**
  - Astar
  - AruCo marker detection
- **stopline recognition**

Environment required:

- OpenCV 4
- Ubuntu 18.04
- Python 3

## Lane Following

Using the window sliding method to detect the lane lines and applying PID control parameters to ensure the vehicle stays within the lane.

### Camera Calibration

Firstly, we need to use `calibration.py` to perform camera calibration, obtain parameters such as ret, mtx, dist, rvecs, tvecs and store these parameters in `calibration.yaml`.

## Traffic Light Detection

Using HSV to determine the color of traffic lights.

## Human Detection and Avoidance

Using HOG to detect the human body and draw a bounding box on the screen to allow the car to perform obstacle avoidance.

## Path Planning

After recognizing the AruCO marker, determine the current relative position of the car on the map and use the A star algorithm to calculate the shortest path to the destination.

## Stopline Recognition

Using the size of the pixels and color recognition of the stop line on the frame to perform stop line recognition, and ensuring that the car stops before the stop line.
