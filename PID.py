#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import PoseStamped, Twist, Pose

def quaternion_to_euler(w, x, y, z):
	ysqr = y * y
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = np.arctan2(t0, t1)
	t2 = +2.0 * (w * y - z * x)
	t2 = np.where(t2 > +1.0, +1.0, t2)
	#t2 = +1.0 if t2 > +1.0 else t2
	t2 = np.where(t2 < -1.0, -1.0, t2)
	#t2 = -1.0 if t2 < -1.0 else t2
	Y = np.arcsin(t2)
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = np.arctan2(t3, t4)
	Z = float(Z)
	return Z

class PIDNode:
	def __init__(self):
		# Initialize the ROS node
		rospy.init_node('pid_node')

		# Create a parameter and set its initial value

		# Set up the subscriber for the TF messages
		self.pose_sub = rospy.Subscriber("/tf", TFMessage, self.callback_pose)
		
		# Set up the subscriber for the PIDreferenca messages
		self.pid_sub_x = rospy.Subscriber("/pid_x", Float64, self.callback_pid_x)
		
		# Set up the subscriber for the PIDreferenca messages
		self.pid_sub_y = rospy.Subscriber("/pid_y", Float64, self.callback_pid_y)
		
		# Set up the subscriber for the PIDreferenca messages
		self.pid_sub_yaw = rospy.Subscriber("/pid_yaw", Float64, self.callback_pid_yaw)	
		
		# Set up the subscriber for the PIDreferenca messages
		self.pid_sub_yaw = rospy.Subscriber("/pid_enable", String, self.callback_pid_enable)
		
		# Set up publisher for /cmd_vel
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		# Set the loop rate in Hz
		self.rate = rospy.Rate(50)

		self.current_pose = [0.0,0.0,0.0]
		
		self.referenca = [0.0,0.0,0.0]
		self.pid_enable = 'False'
		self.pid_flag = 'False'
		
		self.PID_Kr_z = 2.55  # Kkr = 5.1
		self.PID_Kr_y = 1.5  # Kkr = 5.3
		self.PID_Kr_x = 1  # Kkr = 2.9
		
		self.PID_Ki_y = 0.2
		self.PID_Ki_x = 0.2
		
		self.integral_greska_y = 0
		self.integral_greska_x = 0
		
		self.PID_T = 0.02
		
		
	def callback_pose(self, data):
		x = data.transforms[0].transform.translation.x
		y = data.transforms[0].transform.translation.y
		z = data.transforms[0].transform.rotation.z
		w = data.transforms[0].transform.rotation.w
		or_x = data.transforms[0].transform.rotation.x
		or_y = data.transforms[0].transform.rotation.y
		
		z_euler = quaternion_to_euler(w, or_x, or_y, z)

		self.current_pose = [x,y,z_euler]

	def callback_pid_x(self, data):
		self.referenca[0] = data.data
	def callback_pid_y(self, data):
		self.referenca[1] = data.data
	def callback_pid_yaw(self, data):
		self.referenca[2] = data.data
	def callback_pid_enable(self, data):
		self.pid_enable = data.data

	def run(self):
		while not rospy.is_shutdown():
			
			move = Twist()
			if self.pid_enable == 'True':	
				
				# PID Z ORIJENTACIJA
				e_t_z = self.referenca[2] - self.current_pose[2]
				Up_z = self.PID_Kr_z*e_t_z
					
				pid_z = Up_z 
				
				#PID X os
				e_t_x = self.referenca[0] - self.current_pose[0]
				Up_x = self.PID_Kr_x*e_t_x
				self.integral_greska_x += e_t_x*self.PID_T
				self.integral_greska_x = max(min(self.integral_greska_x,0.1),-0.1)
				Ui_x = self.PID_Ki_x * self.integral_greska_x
				
				pid_x = Up_x + Ui_x
				
				if abs(pid_x) < 0.02:
					pid_x=0		
								
				#PID Y os
				e_t_y = self.referenca[1] - self.current_pose[1]
				Up_y = self.PID_Kr_y*e_t_y
				self.integral_greska_y += e_t_y*self.PID_T
				self.integral_greska_y = max(min(self.integral_greska_y,0.1),-0.1)
				Ui_y = self.PID_Ki_y * self.integral_greska_y
					
					
				pid_y = Up_y + Ui_y
				if abs(pid_y) < 0.018:
					pid_y=0
				
				move.linear.x = pid_x
				move.linear.y = pid_y
				move.angular.z = pid_z
				self.pub.publish(move)
				self.pid_flag = 'True'
			else:
				if self.pid_flag == 'True':
					self.integral_greska_y = 0
					self.integral_greska_x = 0
					move.linear.x = 0.0
					move.linear.y = 0.0
					move.angular.z = 0.0
					self.pub.publish(move)
					self.pid_flag = 'False'

			# Sleep to maintain the loop rate
			self.rate.sleep()

if __name__ == '__main__':
	try:
		pid_node = PIDNode()
		pid_node.run()
	except rospy.ROSInterruptException:
		pass
	

