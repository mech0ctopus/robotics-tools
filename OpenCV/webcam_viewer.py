# -*- coding: utf-8 -*-
"""
Webcam viewer with OpenCV
"""
import cv2
    
def video_stream(mirror=True):
    '''Display live webcam video stream'''
    cam = cv2.VideoCapture(0)
    
    while True:
        ret_val, img = cam.read()
        if mirror: 
            img = cv2.flip(img, 1)
        cv2.imshow('Webcam', img)	

        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()
    
if __name__=='__main__':
    video_stream(mirror=False)