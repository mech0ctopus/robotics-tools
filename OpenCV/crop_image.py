# -*- coding: utf-8 -*-
"""
Crop image
"""

def crop_image(img,x=640,y=192,mode='middle'):
    '''Crops images starting at 'top', 'middle', or 'bottom'.'''
   
    if img.shape[0] != y:
        #Crop vertically
        if mode=='top':
            y_top, y_bottom = 0, y
            img=img[y_top:y_bottom,:]
        elif mode=='middle':
            y_mid=img.shape[0]/2
            y_top, y_bottom = int(y_mid-y/2), int(y_mid+y/2)
            img=img[y_top:y_bottom,:]
        elif mode=='bottom':
            y_top, y_bottom = img.shape[0]-y, img.shape[0]
            img=img[y_top:y_bottom,:]
        else:
            print('Unknown crop mode.')
            img=None
    
    if img.shape[1] != x:
        #Crop horizontally in the middle of image
        x_mid=img.shape[1]/2
        x_left, x_right = int(x_mid-x/2), int(x_mid+x/2)
        img=img[:,x_left:x_right]
        
    return img