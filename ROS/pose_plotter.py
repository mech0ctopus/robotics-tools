#!/usr/bin/env python

import roslib
import rospy
from nav_msgs.msg import Odometry
import csv
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

def odometryCb(msg):
    '''Creates CSV of Odometry data from ROS.'''
    filename=r'pose_data.csv'
    t=msg.header.stamp
    x_pos=msg.pose.pose.position.x
    y_pos=msg.pose.pose.position.y
    z_pos=msg.pose.pose.position.z

    x_or=msg.pose.pose.orientation.x
    y_or=msg.pose.pose.orientation.y
    z_or=msg.pose.pose.orientation.z
    w_or=msg.pose.pose.orientation.w
    orientation_list=[x_or,y_or,z_or,w_or]
    
    #Convert quaternion to RPY
    (roll,pitch,yaw)=euler_from_quaternion(orientation_list)

    with open(filename,mode='a') as csv_file:
        writer=csv.writer(csv_file,delimiter=',')
        writer.writerow([t, x_pos, y_pos, yaw])

def animate(i):
    '''Plots live pose data.'''
    graph_data = open('pose_data.csv','r').read()
    lines = graph_data.split('\n')
    ts = []
    xs = []
    ys = []
    yaws = []
    for line in lines:
        if len(line) > 1:
            t, x, y, yaw = line.split(',')
            ts.append(float(t))
            xs.append(float(x))
            ys.append(float(y))
            yaws.append(float(yaw))
    ax1.clear()
    ax1.plot(ts, xs,label='x')
    ax1.plot(ts, ys,label='y')
    ax1.plot(ts, yaws,label='yaw')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m) and Angle (rad)')
    plt.title('Robot Pose vs. Time')
    plt.grid(True)

if __name__ == "__main__":
    rospy.init_node('pose_plotter', anonymous=True)
    rospy.Subscriber('/odom',Odometry,odometryCb)
    style.use('fivethirtyeight')
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
    rospy.spin()
