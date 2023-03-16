import cv2 as cv
import numpy as np
import glob

# Load previously saved data
##################################TO DO##################################
with np.load('calibration.npz') as file:
    mtx, dist, rvecs, tvecs = [ file[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs') ]
##################################TO DO##################################

def draw(img, corners, imgpts):
    corners = corners.astype(int)
    imgpts = imgpts.astype(int)
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel( )), (0,0,255), 5)
    return img

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

#for image in glob.glob('1108 hw/*.jpg'):
img = cv.imread('img3.jpg') #load your photo
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
ret, corners = cv.findChessboardCorners(gray, (9,6),None)


if ret == True:

    corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)

##################################TO DO##################################

#use opencv function slovePnP to get objectpoints, chessboardcorner, camera matrix and distortion coefficient

    ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
    print("rvecs:",rvecs)
    print("tvecs",tvecs)
    # (1) Find the rotation and translation vectors.

#use opencv function projectPoints to get axis, rvecs , tvecs, mtx, dist

    # (2) Project 3D points to image plane
    imgpts,jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
    print(imgpts)

    ##################################TO DO##################################

    img = draw(img,corners2,imgpts)

    height, width = img.shape[:2]
    img = cv.resize(img, (width, height), interpolation = cv.INTER_CUBIC)
    cv.imshow('result',img)

    k = cv.waitKey(0) & 0xFF
    if k == ord('s'):
        cv.imwrite('result', img)

    cv.waitKey(0)
    cv.destroyAllWindows()