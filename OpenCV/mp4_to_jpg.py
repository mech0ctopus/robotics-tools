'''
Using OpenCV takes a mp4 video and produces a number of images.
Requirements
----
You require OpenCV 3.2 to be installed.
Run
----
Place file in same directory as filenames.
Open the mp4_to_jpg.py and edit student name and filenames. Then run:
$ cd <file_location>
$ python mp4_to_jpg.py
Which will produce a folder based on student name containing all images for all videos.
'''
import cv2
import numpy as np
import os

student='cmiller'
filenames=['IMG_4216.mov']
fps = 30

def mp4_to_jpegs(filename,fps):
    # Playing video from file:
    cap = cv2.VideoCapture(filename)
    cap.set(cv2.CAP_PROP_FPS, fps)
    title=filename.split('.')[0]

    try:
        if not os.path.exists(student):
            os.makedirs(student)
    except OSError:
        print ('Error: Creating directory')

    currentFrame = 0
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            break

        # Saves image of the current frame in jpg file
        name = './'+str(student)+'/video_'+str(title)+'_frame_' + str(currentFrame) + '.jpg'
        print ('Creating...' + name)
        cv2.imwrite(name, frame)

        # To stop duplicate images
        currentFrame += 1

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def mp4s_to_jpegs(filenames,fps):
    for filename in filenames:
        mp4_to_jpegs(filename,fps)

if __name__ == '__main__':
    mp4s_to_jpegs(filenames,fps)