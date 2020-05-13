# -*- coding: utf-8 -*-
"""
Craig Miller
cdmiller@wpi.edu
Advanced Robot Navigation, RBE595
Homework #1: Simple Reactive Navigation

Contains Robot and Environment Classes.
Contains utility functions.
"""
import numpy as np
import matplotlib.pyplot as plt
import re
from math import radians, sqrt, inf

logs=[]
created_log=False

def compute_distance(point_1,point_2):
    '''Computes Euclidean distance between two points in 2D space.'''
    dy=point_2[1]-point_1[1]
    dx=point_2[0]-point_1[0]
    distance=sqrt(dx**2+dy**2)
    return distance

def obstacle_distance(robot,environment,orientation_offset=0):
    '''Returns the distance of the closest obstacle in front of robot.'''
    
    x,y=robot.position #initial position
    closest_obstacle=inf
    test_points=[]
    #Draw line from robot position to end of occupancy grid
    while x<environment.occupancy_grid.shape[0] and y<environment.occupancy_grid.shape[1] and x>=0 and y>=0:
        test_points.append([x,y])
        x+=np.cos(robot.orientation+radians(orientation_offset))
        y+=np.sin(robot.orientation+radians(orientation_offset))        
    for test_point in test_points:
        if environment.occupancy_grid[int(test_point[0])][int(test_point[1])]:
            obstacle=compute_distance(robot.position,test_point)
            if obstacle<closest_obstacle:
                closest_obstacle=obstacle
        
    return closest_obstacle

def obstacle_distances(robot,environment):
    '''Returns the distances of the closest obstacle in each direction around
    the robot.'''
    
    x,y=robot.position #initial position
    closest_obstacles={0:inf,45:inf,90:inf,135:inf,180:inf,225:inf,270:inf,315:inf}
    for offset, closet_obstacle in closest_obstacles.items():
        closest_obstacles[offset]=obstacle_distance(robot,environment,orientation_offset=offset)
        closest_obstacles[offset]=round(closest_obstacles[offset],2)
    return closest_obstacles

def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    l.sort(key=alphanum_key)
    
def create_env(bounds=[0,100,0,100]):
    '''Creates instance of Environment class with many obstacle shapes.'''
    env=Environment(bounds)
    
    env.add_circle((5,5),3)
    env.add_circle((25,25),1)
    env.add_circle((25,10),5)
    env.add_circle((45,30),4)
    env.add_circle((12,18),2.5)
    
    env.add_rectangle((0,20),4,2)
    env.add_rectangle((10,0),3,5)
    env.add_rectangle((35,10),4,2)
    env.add_rectangle((30,20),3,5)
    env.add_rectangle((40,40),4,8)
    
    env.add_triangle([(10,35),(15,40),(20,37)])
    env.add_triangle([(6,12),(10,12),(5,14)])
    env.add_triangle([(16,22),(20,22),(19,24)])
    
    return env

def robot_logger(robot,time,distance,print_to_screen=True,save_to_file=True):
    '''Logs robot data [time,x,y,yaw,closest_obstacle].'''
    global logs, created_log
    
    if len(logs) != 0:
        #Get idx of last logged position
        idx=len(logs)-1
        
        current_log=[time,
                     robot.position[0], #x
                     robot.position[1], #y
                     robot.orientation, #yaw
                     robot.position[0]-logs[idx][1], #dx
                     robot.position[1]-logs[idx][2], #dy
                     robot.orientation-logs[idx][3], #dyaw
                     distance]
    else:
        current_log=[time,
             robot.position[0], #x
             robot.position[1], #y
             robot.orientation, #yaw
             0, #dx
             0, #dy
             0, #dyaw
             distance]
    for idx, item in enumerate(current_log):
        if idx>0:
            current_log[idx]=round(current_log[idx],3)
        
    #Append to logs variable
    logs.append(current_log)    
    
    if print_to_screen:        
        print(current_log)
        
    #Update text file
    if save_to_file:
        #Create logs and write header if necessary
        if created_log==False:
            log_file=open('simulation_logs.txt',mode='w')
            log_file.write('time, x, y, yaw (rad), dx, dy, dyaw (rad), closest_obstacle\n')
            log_file.close()
            created_log=True
            
        with open('simulation_logs.txt', mode='a') as log_file:
            for idx, item in enumerate(current_log):
                if idx==(len(current_log)-1):
                    log_file.write(f'{item}')
                else:
                    log_file.write(f'{item}, ')
            log_file.write('\n')

class Robot():
    '''Defines circular 2D robot that can change position and orientation.'''
    def __init__(self,initial_position):
        #Initialize robot parameters
        self.position=initial_position
        #Define possible directions
        directions_deg=np.arange(0,360,45)
        self.directions_rad=np.array([radians(direction) for direction in directions_deg])
        self.orientation=self.directions_rad[0]
        self.velocity=1
        self.radius=0.5
        
    def __repr__(self):
        return f'Robot: Position {self.position}, Orientation: {self.orientation}'
        
    def update_position(self,dt=1):
        current_x=self.position[0]
        current_y=self.position[1]
        
        #Update position
        self.position[0]=current_x+self.velocity*np.cos(self.orientation)*dt
        self.position[1]=current_y+self.velocity*np.sin(self.orientation)*dt
        
    def rotate(self,cw=True,angle=45):
        if cw:
            self.orientation=self.orientation+radians(angle)
        else:
            self.orientation=self.orientation-radians(angle)
        
class Environment():
    '''Defines 2D environment that can hold shapes as obstacles.
    Obstacles are shown at exact locations and are represented in an
    occupancy grid.'''
    def __init__(self,bounds=[0,30,0,30],resolution=1):
        self.obstacles=[]
        self.bounds=bounds
        self.occupancy_grid=np.zeros((int((bounds[1]-bounds[0])/resolution),
                                      int((bounds[3]-bounds[2])/resolution)))
        self.occupancy_grid[int(bounds[0]/resolution),:]=1
        self.occupancy_grid[int(bounds[1]/resolution)-1,:]=1
        self.occupancy_grid[:,int(bounds[2]/resolution)]=1
        self.occupancy_grid[:,int(bounds[3]/resolution)-1]=1
        
    def add_circle(self,position,radius):
        #Add circle to obstacle list
        circle=plt.Circle(position,radius=radius,fc='r',ec='black')
        self.obstacles.append(circle)
        #Update occupancy grid
        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):
                if ((i - position[0])**2 + (j - position[1])**2) <= radius**2:
                    self.occupancy_grid[i][j]=1
        
    def add_rectangle(self,position,width,height,angle=0):
        #Add rectangle to obstacle list
        rectangle=plt.Rectangle(position,width,height,fc='r',ec='black')
        self.obstacles.append(rectangle)
        #Update occupancy grid
        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):
                if i>=position[0] and i<=(position[0]+width) and j>=position[1] and j<=(position[1]+height):
                    self.occupancy_grid[i][j]=1
    
    def add_triangle(self,vertices,closed=True):
        #Add polygon to obstacle list
        triangle=plt.Polygon(vertices,fc='r',ec='black')
        self.obstacles.append(triangle)
        
        def isInside(vertices, x, y):
            '''Checks if point is within triangle formed by vertices.'''
            x1=vertices[0][0]
            y1=vertices[0][1]
            x2=vertices[1][0]
            y2=vertices[1][1]
            x3=vertices[2][0]
            y3=vertices[2][1]
            #https://www.geeksforgeeks.org/check-whether-a-given-point-lies-inside-a-triangle-or-not/
            def area(x1, y1, x2, y2, x3, y3): 
                return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)
            # Calculate area of triangle ABC 
            A = area (x1, y1, x2, y2, x3, y3) 
            # Calculate area of triangle PBC  
            A1 = area (x, y, x2, y2, x3, y3) 
            # Calculate area of triangle PAC  
            A2 = area (x1, y1, x, y, x3, y3) 
            # Calculate area of triangle PAB  
            A3 = area (x1, y1, x2, y2, x, y) 
            # Check if sum of A1, A2 and A3  
            # is same as A 
            if(A == A1 + A2 + A3): 
                return True
            else: 
                return False
        #Update occupancy grid
        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):
                if isInside(vertices,i,j):
                    self.occupancy_grid[i][j]=1