#!/usr/bin/env python
from pymavlink import mavutil
from time import sleep
import rospy
from geometry_msgs.msg import Twist
from pubpub.msg import encoder
from std_msgs.msg import Int32, Float32
from math import cos, sin, pi
from pid import PID
from collections import deque
from scipy.signal import butter, lfilter
import numpy as np

pwm_left_pid = rospy.Publisher('/pwm_left_pid', Int32, queue_size=1)
pwm_right_pid = rospy.Publisher('/pwm_right_pid', Int32, queue_size=1)
vel_desired_left = rospy.Publisher('/vel_desired_left', Float32, queue_size=1)
vel_desired_right = rospy.Publisher('/vel_desired_right', Float32, queue_size=1)
vel_measured_left = rospy.Publisher('/vel_measured_left', Float32, queue_size=1)
vel_measured_right = rospy.Publisher('/vel_measured_right', Float32, queue_size=1)
vel_mfiltered_left = rospy.Publisher('/vel_mfiltered_left', Float32, queue_size=1)
vel_mfiltered_right = rospy.Publisher('/vel_mfiltered_right', Float32, queue_size=1)

def butter_lowpass(cutoff, fs, order=5):
	nyq=0.5*float(fs)
	normal_cutoff=float(cutoff/nyq)
	b, a = butter(order, normal_cutoff, btype='low', analog=False)
	return b,a

def butter_lowpass_filter(data, cutoff, fs, order=5):
	b,a=butter_lowpass(cutoff,fs,order=order)
	y=lfilter(b,a,data)
	return y

class Rover:
	'''Defines Rover class for using pymavlink.'''
	def __init__(self):
		'''Defines constructor for Rover class.'''
		#Connect to PixHawk
		self.master=mavutil.mavlink_connection('udpin:0.0.0.0:14550')
		self.master.wait_heartbeat()
		#Initialize cmd_vel and Rover parameters
		self.cmd_vel={'vx':0,'wz':0,'vl':0,'vr':0}
		self.max_vel={'vx':1.9,'wz':-3.14,'vl':1.9,'vr':1.9} #2.68
		self.min_vel={'vx':-1.6,'wz':3.14,'vl':-1.6,'vr':-1.6} #-2.68
		self.max_pwm={'vx':1900,'wz':1900,'vl':1900,'vr':1900}
		self.neutral_pwm={'vx':1500,'wz':1500,'vl':1500,'vr':1500}
		self.min_pwm={'vx':1100,'wz':1100,'vl':1100,'vr':1100}
		self.pos_m_vx=(self.max_pwm['vx']-self.neutral_pwm['vx'])/(self.max_vel['vx']-0)
		self.neg_m_vx=(self.neutral_pwm['vx']-self.min_pwm['vx'])/(0-self.min_vel['vx'])
		self.pos_m_wz=(self.max_pwm['wz']-self.neutral_pwm['wz'])/(self.max_vel['wz']-0)
		self.neg_m_wz=(self.neutral_pwm['wz']-self.min_pwm['wz'])/(0-self.min_vel['wz'])
		self.pos_m_vl=(self.max_pwm['vl']-self.neutral_pwm['vl'])/(self.max_vel['vl']-0)
		self.neg_m_vl=(self.neutral_pwm['vl']-self.min_pwm['vl'])/(0-self.min_vel['vl'])
		self.pos_m_vr=(self.max_pwm['vr']-self.neutral_pwm['vr'])/(self.max_vel['vr']-0)
		self.neg_m_vr=(self.neutral_pwm['vr']-self.min_pwm['vr'])/(0-self.min_vel['vr'])

		#initialize encoder info.
		self.counts={'left':0,'right':0}
		self.prev_counts={'left':0,'right':0}
		self.past_vels={'left':deque([],maxlen=8),'right':deque([],maxlen=8)}
		self.current_vel={'left':0.0,'right':0.0}
		self.cpr=1120.0 #counts per revolution
		self.wheel_radius=0.12 #meters
		self.axle_length=0.46 #meters

		#Initialize independent PID controllers for left and wheel wheels
		self.left_controller=PID(P=7,I=35,D=0.0) #300,1200,0.0
		self.right_controller=PID(P=7,I=35,D=0.0) #7, 35, 0

		#Reset RC values so we can use them
		self.reset_rc_pwm()

	def arm(self):
		'''Arms the Rover.'''
		print('Arming Rover')
		self.master.mav.command_long_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			1,0,0,0,0,0,0)

	def disarm(self):
		'''Disarms the Rover.'''
		print('Disarming Rover')
		#Reset RC PWM values to Neutral
		self.reset_rc_pwm()
		self.master.mav.command_long_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			0,0,0,0,0,0,0)

	def get_pwm(self,skid_steer=False):
		'''Calculates PWM levels to achieve current cmd_vel state.'''
		#vx: Forward: 1500-1900, Rev: 1100-1500
		#wz: CW: 1500-1900, CCW: 1100-1500.
		if skid_steer:
			if self.cmd_vel['vl']>=0:
				m_vl=self.pos_m_vl
			else:
				m_vl=self.neg_m_vl

			if self.cmd_vel['vr']>=0:
				m_vr=self.pos_m_vr
			else:
				m_vr=self.neg_m_vr
			
			vl_diff=m_vl*self.cmd_vel['vl']
			vr_diff=m_vr*self.cmd_vel['vr']
			vl_pwm=self.neutral_pwm['vl']+vl_diff
			vr_pwm=self.neutral_pwm['vr']+vr_diff
			
			if vl_pwm>self.max_pwm['vl']:
				vl_pwm=self.max_pwm['vl']
			elif vl_pwm<self.min_pwm['vl']:
				vl_pwm=self.min_pwm['vl']

			if vr_pwm>self.max_pwm['vr']:
				vr_pwm=self.max_pwm['vr']
			elif vr_pwm<self.min_pwm['vr']:
				vr_pwm=self.min_pwm['vr']
			
			return vl_pwm, vr_pwm

		else:
			if self.cmd_vel['vx']>=0:
				m_vx=self.pos_m_vx
			else:
				m_vx=self.neg_m_vx

			if self.cmd_vel['wz']>=0:
				m_wz=self.pos_m_wz
			else:
				m_wz=self.neg_m_wz
			
			vx_diff=m_vx*self.cmd_vel['vx']
			wz_diff=m_wz*self.cmd_vel['wz']
			vx_pwm=self.neutral_pwm['vx']+vx_diff
			wz_pwm=self.neutral_pwm['wz']+wz_diff

			if vx_pwm>self.max_pwm['vx']:
				vx_pwm=self.max_pwm['vx']
			elif vx_pwm<self.min_pwm['vx']:
				vx_pwm=self.min_pwm['vx']

			if wz_pwm>self.max_pwm['wz']:
				wz_pwm=self.max_pwm['wz']
			elif wz_pwm<self.min_pwm['wz']:
				wz_pwm=self.min_pwm['wz']

			return vx_pwm, wz_pwm

	def get_speed(self,side,counts,prev_counts,dt):
		'''Calculates wheel speed (m/s) from encoder data.'''
		revs=float((self.counts[side]-self.prev_counts[side])/self.cpr)
		lin_dist=revs*2.0*pi*self.wheel_radius
		speed=lin_dist/dt
		return speed

	def get_wheel_vel(self):
		'''Convert Vx and Wz to Vl and Vr'''
		#if self.cmd_vel['wz']<0.05:
		#	self.cmd_vel['vl']=self.cmd_vel['vx']
		#	self.cmd_vel['vr']=self.cmd_vel['vx']
		#else:
		self.cmd_vel['vl']=(self.cmd_vel['vx']-self.cmd_vel['wz']*self.axle_length/2.0)/self.wheel_radius
		self.cmd_vel['vr']=(self.cmd_vel['vx']+self.cmd_vel['wz']*self.axle_length/2.0)/self.wheel_radius
	
	def lookup_pwm(self,vl,vr):
		'''Calculates PWM levels to achieve current cmd_vel state.'''
		#vx: Forward: 1500-1900, Rev: 1100-1500
		#wz: CW: 1500-1900, CCW: 1100-1500.
		if vl>=0:
			m_vl=self.pos_m_vl
		else:
			m_vl=self.neg_m_vl

		if vr>=0:
			m_vr=self.pos_m_vr
		else:
			m_vr=self.neg_m_vr
		
		vl_diff=m_vl*vl
		vr_diff=m_vr*vr
		vl_pwm=self.neutral_pwm['vl']+vl_diff
		vr_pwm=self.neutral_pwm['vr']+vr_diff
		print(vl_pwm,vr_pwm)
		if vl_pwm>self.max_pwm['vl']:
			vl_pwm=self.max_pwm['vl']
		elif vl_pwm<self.min_pwm['vl']:
			vl_pwm=self.min_pwm['vl']

		if vr_pwm>self.max_pwm['vr']:
			vr_pwm=self.max_pwm['vr']
		elif vr_pwm<self.min_pwm['vr']:
			vr_pwm=self.min_pwm['vr']

		return vl_pwm, vr_pwm

	def reset_pixhawk(self):
		'''Reboots Pixhawk flight controller board.'''
		print('Rebooting PixHawk')
		self.master.reboot_autopilot()
		#Sleep to allow pixhawk to comeback
		sleep(5)

	def reset_rc_pwm(self):
	    """ Reset RC channel PWM values"""
	    rc_channel_values = [1500 for _ in range(9)] #1500 is neutral
	    self.master.mav.rc_channels_override_send(
			self.master.target_system,
			self.master.target_component,
			*rc_channel_values) # RC channel list, in microseconds.

	def set_cmd_vel(self,vx,wz):
		'''Sets current cmd_vel state for Rover.'''
		self.cmd_vel['vx']=vx
		self.cmd_vel['wz']=wz
		#Convert to left/right velocities
		self.get_wheel_vel()

	def set_counts(self,left_counts,right_counts):
		'''Sets current encoder states for Rover.'''
		self.prev_counts['left']=self.counts['left']
		self.prev_counts['right']=self.counts['right']

		self.counts['left']=left_counts
		self.counts['right']=right_counts

	def set_rc_pwm(self, channel_id, pwm):
		""" Set RC channel PWM value.
			Args:
				channel_id (TYPE): Channel ID (3:Throttle, 1:Yaw)
				pwm (int, optional): Channel pwm value 1100-1900"""
		if channel_id < 1 or channel_id > 9:
			print("Channel does not exist.")
			return
		#Ignore values that are 65535 (keep previous cmd values)
		rc_channel_values = [65535 for _ in range(9)]
		rc_channel_values[channel_id - 1] = pwm
		self.master.mav.rc_channels_override_send(
			self.master.target_system,
			self.master.target_component,
			*rc_channel_values) # RC channel list, in microseconds.

	def set_skid_v(self,vl_pwm,vr_pwm):
		'''Simultaneously set left and right wheel velocities.'''
		rc_channel_values = [65535 for _ in range(9)]
		rc_channel_values[1-1]=vl_pwm
		rc_channel_values[3-1]=vr_pwm
		self.master.mav.rc_channels_override_send(
			self.master.target_system,
			self.master.target_component,
			*rc_channel_values) # RC channel list, in microseconds.

	def set_vx(self,vx_pwm):
		'''Sets RC channel to current cmd_vel for vx.'''
		self.set_rc_pwm(3,vx_pwm)

	def set_wz(self,wz_pwm):
		'''Sets RC channel to current cmd_vel for wz.'''
		self.set_rc_pwm(1,wz_pwm)

	def update_current_vel(self,left_counts,right_counts,dt):
		'''Updates current velocity state based on encoder data.'''
		self.set_counts(left_counts,right_counts)
		v_l=self.get_speed('left',self.counts['left'], self.prev_counts['left'],dt)
		v_r=self.get_speed('right',self.counts['right'], self.prev_counts['right'],dt)
		self.current_vel={'left':v_l,'right':v_r}
		self.past_vels['left'].append(v_l)
		self.past_vels['right'].append(v_r)
		#rospy.loginfo("V_l_act: " + str(round(self.current_vel['left'],3)) + str(', V_r_act: ') + str(round(self.current_vel['right'],3)))

	def update(self,vx,wz,skid_steer=False):
		'''Updates controller based on cmd_vel and sends PWM values via mavlink.'''
		self.set_cmd_vel(vx,wz) #Also updates vl/vr
		if skid_steer:	
			#Update setpoints to current command velocities
			self.left_controller.setPoint(self.cmd_vel['vl'])
			self.right_controller.setPoint(self.cmd_vel['vr'])
			vel_desired_left.publish(Float32(self.cmd_vel['vl']))
			vel_desired_right.publish(Float32(self.cmd_vel['vr']))
			#rospy.loginfo("Des. Left: " + str(self.cmd_vel['vl']) + str(', Des. Right: ') + str(self.cmd_vel['vr']))
			#rospy.loginfo("V. Left: " + str(self.current_vel['left']) + str(', V. Right: ') + str(self.current_vel['right']))

			#Get filtered velocity measurements
			left_vel_filtered=np.average(self.past_vels['left'])
			right_vel_filtered=np.average(self.past_vels['right'])
			vel_mfiltered_left.publish(Float32(left_vel_filtered))
			vel_mfiltered_right.publish(Float32(right_vel_filtered))
			rospy.loginfo("Filt. Left: " + str(left_vel_filtered) + str(', Filt. Right: ') + str(right_vel_filtered))
			# left_vel_filtered=butter_lowpass_filter(self.past_vels['left'],cutoff=0.5,fs=13.0,order=3.0)
			# right_vel_filtered=butter_lowpass_filter(self.past_vels['right'],cutoff=0.5,fs=13.0,order=3.0)
			# vel_mfiltered_left.publish(Float32(left_vel_filtered[-1]))
			# vel_mfiltered_right.publish(Float32(right_vel_filtered[-1]))

			#Get output PWM values given current velocities (from encoders)
			vl_pwm=self.neutral_pwm['vl']+self.left_controller.update(left_vel_filtered)
			vr_pwm=self.neutral_pwm['vr']+self.right_controller.update(right_vel_filtered)
			#rospy.loginfo("Left: " + str(vl_pwm) + str(', Right: ') + str(vr_pwm))

			#TODO: ADD TRY/EXCEPT CATCH FOR INVALID PWM
			if vl_pwm>self.max_pwm['vl']:
				vl_pwm=self.max_pwm['vl']
			if vr_pwm>self.max_pwm['vr']:
				vr_pwm=self.max_pwm['vr']

			self.set_skid_v(vl_pwm,vr_pwm)

			pwm_left_pid.publish(Int32(vl_pwm))
			pwm_right_pid.publish(Int32(vr_pwm))

			#FIX THESE
			vel_measured_left.publish(Float32(self.current_vel['left']))
			vel_measured_right.publish(Float32(self.current_vel['right']))

		else:
			vx_pwm, wz_pwm=self.get_pwm()
			self.set_vx(vx_pwm)
			self.set_wz(wz_pwm)

def callback(msg):
	'''cmd_vel callback to update Rover state.'''
	rover.update(msg.linear.x,msg.angular.z,skid_steer=True)
	#rospy.loginfo("Vx_cmd: " + str(rover.cmd_vel['vx']) + str(', Wz_cmd: ') + str(rover.cmd_vel['wz']))

def wheel_callback(msg):
	'''Encoder callback to update Rover state.'''
	rover.update_current_vel(msg.left_counts,msg.right_counts,dt=0.05)
	#rospy.loginfo("Left: " + str(rover.counts['left']) + str(', Right: ') + str(rover.counts['right']))

def listener():
	'''cmd_vel listener.'''
	rospy.init_node('velocity_interpreter', anonymous=True)
	rospy.Subscriber("cmd_vel_out", Twist, callback)
	rospy.Subscriber("wheel_counts", encoder, wheel_callback)
	rospy.spin()

if __name__=='__main__':
	rover=Rover()
	rover.arm()

	#rover.set_skid_v(1750,1750) 
	#sleep(20)

	try:
		while not rospy.is_shutdown():
			listener()
	except KeyboardInterrupt:
		rospy.loginfo('Exiting Controller')

	for _ in range(9):
		rover.disarm()
