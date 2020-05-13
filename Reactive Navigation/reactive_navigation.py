# -*- coding: utf-8 -*-
"""
Craig Miller
cdmiller@wpi.edu
Advanced Robot Navigation, RBE595
Homework #1: Simple Reactive Navigation

Runs simulation of 2D robot with reactivate obstacle avoidance.
"""

import numpy as np
import matplotlib.pyplot as plt
from glob import glob
from os import remove
import cv2
from robot import *

def simulate(robot,environment,bounds,
             threshold=5,timesteps=20,
             plot_path=False,log=True,
             save_mp4=True):
    '''Runs simulation of 2D robot with reactivate obstacle avoidance.'''
    #Initialize plot
    plt.axes()
    plt.grid(which='both')
    
    #Plot obstacles
    for item in env.obstacles:
        plt.gca().add_patch(item)
    
    #Plot robot position
    for t in range(timesteps):
        if t==0:
            print('time, x, y, yaw (radians), closest_obstacle\n')
        else:
            robot_line.remove()

        #Update robot position
        robot_outline=plt.Circle((r.position),
                             radius=r.radius,
                             fc='white',ec='black')
        #Update robot orientation marker position
        robot_line=plt.Line2D([r.position[0],r.position[0]+r.radius*np.cos(r.orientation)], 
                              [r.position[1],r.position[1]+r.radius*np.sin(r.orientation)], 
                              color='black')
        
        #Add dot at previous robot position
        if plot_path:
            plt.gca().scatter(r.position[0],r.position[1],fc='white',ec='black',marker=".")
        
        #Update plot
        plt.gca().add_patch(robot_outline)
        plt.gca().add_line(robot_line)
        #Set plot properties & labels
        plt.axis(bounds) #[xmin,xmax,ymin,ymax]
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Robot Simulation: Reactive Control')
        plt.show()
        
        if save_mp4:
            plt.savefig(f'{t}.PNG')
        plt.pause(0.05)
        
        #Find nearest obstacles around robot in 45 deg intervals
        nearest_obstacles=obstacle_distances(robot,environment) 
        
        #Update logs
        if log==True:
            robot_logger(robot,t,nearest_obstacles[0],print_to_screen=True,save_to_file=True)
              
        #Check if robot collided
        if environment.occupancy_grid[int(robot.position[0])][int(robot.position[1])]:
            print('Robot Collided!')
        #Check if obstacle is closer to robot than we want
        elif nearest_obstacles[0]<threshold:
            r.rotate(cw=True,angle=5)
        #Update robot position at constant velocity
        else:
            r.update_position()
    
    if save_mp4:
        print('Saving Video of Simulation')
        #Build list of png files
        png_list = glob(r'*.PNG')
        sort_nicely(png_list)
        img_array = []
        #Convert PNG files to MP4
        for png in png_list:
            img = cv2.imread(png)
            height, width, layers = img.shape
            size = (width,height)
            img_array.append(img)
        out = cv2.VideoWriter('2D_Simulation_CraigMiller.mp4',
                              cv2.VideoWriter_fourcc(*'DIVX'), 5, size)
        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()
        
        #Delete all PNG files
        _ = [remove(png) for png in png_list]
        
if __name__=='__main__':
    #Instantiate robot & environment
    r=Robot(initial_position=[15,10])
    bounds=[0,50,0,50]
    env=create_env(bounds=bounds)
    #Run simulation
    simulate(r,env,bounds=bounds,threshold=5,timesteps=100,plot_path=True,log=True,
             save_mp4=True)