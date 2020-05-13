# -*- coding: utf-8 -*-
"""
Testing OpenCV functionality

Sources:
draw_circles: https://www.geeksforgeeks.org/circle-detection-using-opencv-python/
show_webcam: https://gist.github.com/tedmiston/6060034
watershed: https://gist.github.com/saurabhpal97/6d47c2d6e96bd6de8ce8238764f0b05b#file-watershed-algorithm-py-L20
detect_edges: https://gist.github.com/saurabhpal97/279c3849b729e7f67c5cd57d54ace6fe#file-edge-detection-py
"""

import cv2 
import numpy as np
import matplotlib.pyplot as plt

def draw_circles(img): 
    '''Detects circles in image and plots them.'''
    # Read image. 
    #img = cv2.imread(filepath, cv2.IMREAD_COLOR) 
      
    # Convert to grayscale. 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
      
    # Blur using 3 * 3 kernel. 
    gray_blurred = cv2.blur(gray, (3, 3)) 
      
    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(gray_blurred,  
                       cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
                   param2 = 30, minRadius = 1, maxRadius = 40) 
      
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
      
            # Draw the circumference of the circle. 
            cv2.circle(img, (a, b), r, (0, 255, 0), 2) 
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3) 
            #cv2.imshow("Detected Circle", img) 
            #cv2.waitKey(0) 
    return img

def watershed(image):
    '''Segments image with watershed algorithm.'''
    #converting image to grayscale format
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #apply thresholding
    ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    #get a kernel
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations = 2)
    #extract the background from image
    sure_bg = cv2.dilate(opening,kernel,iterations = 3)
    
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret,sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
    
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_bg)
    
    ret,markers = cv2.connectedComponents(sure_fg)
    
    markers = markers+1
    
    markers[unknown==255] = 0
    
    markers = cv2.watershed(image,markers)
    image[markers==-1] = [255,0,0]
    
    return sure_fg

def detect_edges(img):
    '''Detects edges in image.'''
    edges = cv2.Canny(img,10,20) #100, 200
    return edges

def show_webcam(mirror=True,detect_circles=False,segment=False,edges=False):
    '''Shows live webcam video stream'''
    cam = cv2.VideoCapture(0)
    while True:
        ret_val, img = cam.read()
        if mirror: 
            img = cv2.flip(img, 1)
        if detect_circles:
            img=draw_circles(img)
        if segment:
            img=watershed(img)
        if edges:
            img=detect_edges(img)
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()

def depth(left_img,right_img):
    stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=11) #16, 15
    disparity = stereo.compute(left,right)
    plt.imshow(disparity,'plasma')#'gray')
    plt.show()
    
if __name__ == '__main__':
    # filepath=r"C:\Users\Craig\Pictures\tv.jpg"
    # # Read image. 
    # img = cv2.imread(filepath, cv2.IMREAD_COLOR) 
    # cv2.imshow('rgb',img)
    # img=detect_edges(img)
    # cv2.imshow('edges',img)
    #show_webcam(mirror=True,detect_circles=True,edges=True)
    
    # left=r"C:\Users\Craig\Pictures\Left1.JPG"
    # right=r"C:\Users\Craig\Pictures\Right1.JPG"

    left=r"G:\Documents\KITTI\data\val\X\2011_09_30_drive_0018_sync_0000000134.png"
    right=r"G:\Documents\KITTI\data\val\X\2011_09_30_drive_0018_sync_0000000135.png"
    
    left=cv2.imread(left,0)
    right=cv2.imread(right,0)
    depth(left,right)
    
