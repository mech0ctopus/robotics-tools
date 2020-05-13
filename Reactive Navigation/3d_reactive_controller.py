#!/usr/bin/env python
'''
Craig Miller
cdmiller@wpi.edu
Advanced Robot Navigation, RBE595
Homework #1: Simple Reactive Navigation

3D Reactive Controller for use with Clearpath Robotics Jackal
simulation in Gazebo.  

Installation:
	https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html

Usage:
	Terminal 1: roslaunch jackal_gazebo jackal_world.launch config:=front_laser
	Terminal 2: python 3d_reactive_controller.py
'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import time, sleep
from math import radians

class Soda:
	'''Simple Object Detection and Avoidance (SODA) controller.'''
	def lidar_cb(self, data):
		'''LIDAR Callback.  Parse and print LIDAR data.'''
		range_data=data.ranges # meters
		
		zones={0:range_data[359],
			   45:range_data[469],
			   90:range_data[559],
			   135:range_data[649],
			   180:range_data[0],
			   225:range_data[89],
			   270:range_data[189],
			   315:range_data[279]}
		
		#Find closest object in each zone
		for name, distance in zones.items():
		    print(0,zones[0])
		    obstacle, distance, zone_name=self.obstacle_detected(zones[0],0,detection_radius=4)

	def obstacle_detected(self,distance,zone_name,detection_radius=1):
		'''Returns obstacle location if obstacle is found in LIDAR data.'''
		#distance=min(data)
		found_obstacle=False
		if distance<=detection_radius:
			found_obstacle=True
		self.react(found_obstacle,zone_name)
		distance=round(distance,2)
		return found_obstacle, distance, zone_name

	def react(self, obstacle_present,zone_name,rotation_z_deg=10,rotation_time_interval=0.1):
		'''Stops rover and commands reactionary motion if obstacle is detected.'''
		def rotate(angle_deg,direction,time_interval):
			'''Rotates robot about z-axis'''
			#Stop linear motion
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			#Rotate about z (vertical axis)
			vel_msg.angular.z = direction*0.2 #Radians, CCW is positive
			velocity_publisher.publish(vel_msg)
			print 'Rover turning due to obstacle.'

		vel_msg = Twist()

		if obstacle_present==True:
			direction=1 #CCW
			rotate(rotation_z_deg,direction,rotation_time_interval)
		else:
			#Stop linear motion
			vel_msg.linear.x = 0.2
			velocity_publisher.publish(vel_msg)

def sensor_listener():
    '''Create listener node for incoming sensor data.'''
    rospy.init_node('sensor_listener', anonymous=True)
    #Instantiate controller
    soda=Soda()
    #Define loop rate
    rate=rospy.Rate(2) #input in Hz
    #Subscribe to sensor data
    rospy.Subscriber("/front/scan", LaserScan, soda.lidar_cb)
    rospy.spin()

if __name__=='__main__':
    try:
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sensor_listener()
    except rospy.ROSInterruptException:
        pass