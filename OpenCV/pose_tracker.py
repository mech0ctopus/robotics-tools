# -*- coding: utf-8 -*-
"""
Pose tracking of chessboard on live video stream.
"""
import cv2 as cv
import numpy as np
import time

def draw_axes(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def draw_cube(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    img = cv.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img
 
def pose_tracker(ret=True, plot_cube=False, mirror=True):
    '''Runs pose tracking on live webcam video stream'''
    cam = cv.VideoCapture(0)
    
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    if plot_cube:
        axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                           [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3]])
    else:
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        
    while True:
        start=time.time()
        ret_val, img = cam.read()
        if mirror: 
            img = cv.flip(img, 1)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners)
            
            # Draw and display the corners
            cv.drawChessboardCorners(img, (9,6), corners2, ret)
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            
            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
            # project 3D points to image plane
            imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

            if plot_cube:
                #Draw Cube
                img = draw_cube(img,corners2,imgpts)
            else:
                #Draw coordinate frame axes
                img = draw_axes(img,corners2,imgpts)
            cv.imshow('Pose Tracking', img)
        
        #Estimate instantaneous frames per second
        end=time.time()
        fps=round(1/(end-start),2)        
        print(f'FPS: {fps}')
        
        if cv.waitKey(1) == 27: 
            break  # ESC to quit
        
    cv.destroyAllWindows()
    
if __name__=='__main__':
    pose_tracker(ret=True, plot_cube=False, mirror=True)
