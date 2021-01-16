# -*- coding: utf-8 -*-
"""
sin_cmd_vel

Generate and publish sinusoidal cmd_vel to test wheel speed controller.
"""

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

def get_cmd_vel(t,min_vel,max_vel,period):
    '''Plots sine wave.'''
    mean_vel=(max_vel+min_vel)/2
    ampl=max_vel-mean_vel
    cmd_vel=mean_vel+ampl*np.sin(2*np.pi*t/period)
    return cmd_vel
    
def plot_sine(min_vel,max_vel,period,num_sec):
    '''Plots sine wave.'''
    mean_vel=(max_vel+min_vel)/2
    ampl=max_vel-mean_vel
    
    t=np.linspace(start=0,stop=num_sec,num=num_sec*5)
    y=mean_vel+ampl*np.sin(2*np.pi*t/period)
    
    plt.figure()
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity (m/s) vs. Time (s)')
    plt.grid(True)
    plt.plot(t,y)
    plt.show()
    
def sin_cmd_vel():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('sin_cmd_vel_node', anonymous=True)
    hz=10
    rate = rospy.Rate(hz)
    t=0.0
    while not rospy.is_shutdown():
        #Calculate time period
        period=1.0/hz
        #Get velocity command
        cmd_vel=get_cmd_vel(t,min_vel=0.6,max_vel=0.9,period=10)
        
        #Build actual message
        twist=Twist()
        twist.linear.x=cmd_vel
        #Print command to terminal
        rospy.loginfo('t: '+ str(t) + ' cmd_vel: ' + str(cmd_vel))
        #Publish cmd_vel
        pub.publish(twist)
        #Update current time and sleep
        t+=period
        rate.sleep()


if __name__=='__main__':
    #plot_sine(min_vel=0.6,max_vel=0.9,period=10,num_sec=60)
    try:
        sin_cmd_vel()
    except rospy.ROSInterruptException:
        pass