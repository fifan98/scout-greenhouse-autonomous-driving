#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
import copy


from time import sleep
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from std_msgs.msg import Float64, Bool, String

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

class MASTERNode:
	def __init__(self):
		# Initialize the ROS node
		rospy.init_node('master_node')

		# Create a parameter and set its initial value
		rospy.set_param('PID_reference', [0.0, 0.0, 0.0])

		# Set up the subscriber for the PoseSatmped messages
		self.sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback_sub)
		
		# Set up subscriber for /odom
		self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom) 
		
		# Set up subscriber for current position /tf
		self.sub_pose = rospy.Subscriber("/tf", TFMessage, self.callback_pose)
		
		# Set up the subscriber for the /mjeh_status 
		self.sub_mjeh = rospy.Subscriber("/mjeh_status", String, self.callback_mjeh)
		
		# Set up publisher for /cmd_vel
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		# Set up publisher for /pid_referenca
		self.pub_pid_x = rospy.Publisher('/pid_x', Float64, queue_size=10)
		
		# Set up publisher for /pid_referenca
		self.pub_pid_y = rospy.Publisher('/pid_y', Float64, queue_size=10)
		
		# Set up publisher for /pid_referenca
		self.pub_pid_yaw = rospy.Publisher('/pid_yaw', Float64, queue_size=10)
		
		# Set up publisher for /pid_referenca
		self.pub_pid_enable = rospy.Publisher('/pid_enable', String, queue_size=10)
		
		# Set up publisher for /mjeh
		self.pub_mjeh = rospy.Publisher('/mjeh', String, queue_size=10)
		
		
		self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.goal = MoveBaseGoal()
		
		self.box_counter = int(0)
		self.tocke = []
		self.tocke_z = []
		self.tocka_robot = []
		self.pre_node = []
		self.pre_node_kontra = []
		self.tocke_m = []
		self.tocke_dummy = []
		self.box = int(1)
		self.current_vel = [0.0,0.0,0.0]
		self.current_pose = [0.0,0.0,0.0]
		self.sekvenca_enable = False
		self.delta = 2
		self.prvi_put = False
		self.mjeh_status = 'Zauzeto'
		self.mjeh_sekvenca = ['Gore','Gore','Gore']
		self.mjeh_counter = int(0)
		
	def callback_sub(self, data):
		print("Pre node, točka ili pre node kontra( P ,T, K, M ) ili sekvenca S ili I")
		provjera = input()
		if provjera == "P":
			self.pre_node.append(data)
		elif provjera == "M":
			self.tocke_m.append(data)
		elif provjera == "K":
			self.pre_node_kontra.append(data)
		elif provjera == "Z":
			self.tocke_z.append(data)
		elif provjera == "R":
			self.tocka_robot.append(data)
		elif provjera == "S":
			self.sekvenca_enable = True
			self.prvi_put = True			
					
	def callback_odom(self, msg):
		self.current_vel[0] = msg.twist.twist.linear.x
		self.current_vel[1] = msg.twist.twist.linear.y
		self.current_vel[2] = msg.twist.twist.angular.z
		
	def callback_pose(self, data):
		x = data.transforms[0].transform.translation.x
		y = data.transforms[0].transform.translation.y
		z = data.transforms[0].transform.rotation.z
		w = data.transforms[0].transform.rotation.w
		or_x = data.transforms[0].transform.rotation.x
		or_y = data.transforms[0].transform.rotation.y
		
		z_euler = quaternion_to_euler(w, or_x, or_y, z)

		self.current_pose = [x,y,z_euler]
		
	def callback_mjeh(self, data):
		self.mjeh_status = data.data

	def run(self):
		while not rospy.is_shutdown():	
			if self.sekvenca_enable == True:
				print("Unesite u koji box želite ( 1 ili 2 )")
				self.box = int(input())
				if self.prvi_put == True:
					print("Izracun tocaka!")
					for i in range(len(self.pre_node)):
						delta_x = abs(self.pre_node[i].pose.position.x - self.tocke_m[i].pose.position.x)
						delta_y = abs(self.pre_node[i].pose.position.y - self.tocke_m[i].pose.position.y)
						delta_delta_x = delta_x/self.delta
						delta_delta_y = delta_y/self.delta
						for l in range(self.delta):
							self.tocke.append(copy.deepcopy(self.pre_node[0]))
						for l in range(self.delta):
							self.tocke[l+self.delta*i].pose.position.x = copy.copy(self.pre_node[i].pose.position.x) - delta_delta_x*l
						if (copy.copy(self.pre_node[i].pose.position.y) - copy.copy(self.tocke_m[i].pose.position.y)) > 0:
							for l in range(self.delta):
								self.tocke[l+self.delta*i].pose.position.y = copy.copy(self.pre_node[i].pose.position.y) - delta_delta_y*l
						else:
							for l in range(self.delta):
								self.tocke[l+self.delta*i].pose.position.y = copy.copy(self.pre_node[i].pose.position.y) + delta_delta_y*l
					self.prvi_put = False
					print("Izracun gotov!")
				print("Testna sekvenca pokrenuta!")
				print("Šaljem u KONTRA node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node_kontra[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node_kontra[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				print("Šaljem u pre node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				while self.box_counter < (self.delta + 1):
					if self.box_counter < self.delta :
						print("Krecem ulazit u box!")
						pose_x = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.position.x
						pose_y = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.position.y
						
						z = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.z
						w = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.w
						x = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.x
						y = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.y
						
						z_euler = quaternion_to_euler(w, x, y, z)
						
						pid_x = Float64()
						pid_y = Float64()
						pid_yaw = Float64()
						pid_enable = String()
						pid_x = pose_x
						pid_y = pose_y
						pid_yaw = z_euler
						pid_enable = 'True'
						
						print("Palim PID!")
						self.pub_pid_x.publish(pid_x)
						self.pub_pid_y.publish(pid_y)
						self.pub_pid_yaw.publish(pid_yaw)
						self.pub_pid_enable.publish(pid_enable)
						
						print("x pozicija" + str(abs(self.current_pose[0]-pose_x)))
						print("y pozicija" + str(abs(self.current_pose[1]-pose_y)))
						while True:
							print(abs(self.current_pose[0]-pose_x))
							print(abs(self.current_pose[1]-pose_y))
							if  (abs(self.current_pose[0]-pose_x)<0.06):
								break
						print("Gasim PID")
						pid_enable = 'False'
						self.pub_pid_enable.publish(pid_enable)
						self.box_counter += int(1)
					else:
						print("Krecem ulazit u box!")
						pose_x = self.tocke_m[int(self.box)-int(1)].pose.position.x
						pose_y = self.tocke_m[int(self.box)-int(1)].pose.position.y
						
						z = self.tocke_m[int(self.box)-int(1)].pose.orientation.z
						w = self.tocke_m[int(self.box)-int(1)].pose.orientation.w
						x = self.tocke_m[int(self.box)-int(1)].pose.orientation.x
						y = self.tocke_m[int(self.box)-int(1)].pose.orientation.y
						
						z_euler = quaternion_to_euler(w, x, y, z)
						
						pid_x = Float64()
						pid_y = Float64()
						pid_yaw = Float64()
						pid_enable = String()
						pid_x = pose_x
						pid_y = pose_y
						pid_yaw = z_euler
						pid_enable = 'True'
						
						print("Palim PID!")
						self.pub_pid_x.publish(pid_x)
						self.pub_pid_y.publish(pid_y)
						self.pub_pid_yaw.publish(pid_yaw)
						self.pub_pid_enable.publish(pid_enable)
						
						print("x pozicija" + str(abs(self.current_pose[0]-pose_x)))
						print("y pozicija" + str(abs(self.current_pose[1]-pose_y)))
						while True:
							print(abs(self.current_pose[0]-pose_x))
							print(abs(self.current_pose[1]-pose_y))
							if  (abs(self.current_pose[0]-pose_x)<0.06):
								break
						print("Gasim PID")
						pid_enable = 'False'
						self.pub_pid_enable.publish(pid_enable)
						self.box_counter += int(1)
				self.box_counter = int(0)
				
				print("KRECE SEKVENCA NOSACA!")
				mjeh_slanje = String()
				mjeh_slanje = 'Gore'
				self.pub_mjeh.publish(mjeh_slanje)
				sleep(1)
				while self.mjeh_status == 'Zauzeto':
					sleep(0.5)
						
				
				print("Šaljem u pre node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				print("Šaljem u KONTRA node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node_kontra[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node_kontra[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				print("Krecem do robotske ruke!")
				
				for i in range(len(self.tocke_z)):
					self.goal.target_pose.header.frame_id = 'map' 
					self.goal.target_pose.pose.position.x = self.tocke_z[i].pose.position.x
					self.goal.target_pose.pose.position.y = self.tocke_z[i].pose.position.y
					self.goal.target_pose.pose.orientation.z = self.tocke_z[i].pose.orientation.z
					self.goal.target_pose.pose.orientation.w = self.tocke_z[i].pose.orientation.w
					self.goal.target_pose.pose.orientation.x = self.tocke_z[i].pose.orientation.x
					self.goal.target_pose.pose.orientation.y = self.tocke_z[i].pose.orientation.y
					self.client.send_goal(self.goal)
					self.client.wait_for_result()
				
				print("Šaljem u KONTRA node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node_kontra[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node_kontra[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				print("Šaljem u pre node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				while self.box_counter < (self.delta + 1):
					if self.box_counter < self.delta :
						print("Krecem ulazit u box!")
						pose_x = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.position.x
						pose_y = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.position.y
						
						z = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.z
						w = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.w
						x = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.x
						y = self.tocke[(int(self.box)-int(1))*self.delta + self.box_counter].pose.orientation.y
						
						z_euler = quaternion_to_euler(w, x, y, z)
						
						pid_x = Float64()
						pid_y = Float64()
						pid_yaw = Float64()
						pid_enable = String()
						pid_x = pose_x
						pid_y = pose_y
						pid_yaw = z_euler
						pid_enable = 'True'
						
						print("Palim PID!")
						self.pub_pid_x.publish(pid_x)
						self.pub_pid_y.publish(pid_y)
						self.pub_pid_yaw.publish(pid_yaw)
						self.pub_pid_enable.publish(pid_enable)
						
						print("x pozicija" + str(abs(self.current_pose[0]-pose_x)))
						print("y pozicija" + str(abs(self.current_pose[1]-pose_y)))
						while True:
							print(abs(self.current_pose[0]-pose_x))
							print(abs(self.current_pose[1]-pose_y))
							if  (abs(self.current_pose[0]-pose_x)<0.06):
								break
						print("Gasim PID")
						pid_enable = 'False'
						self.pub_pid_enable.publish(pid_enable)
						self.box_counter += int(1)
					else:
						print("Krecem ulazit u box!")
						pose_x = self.tocke_m[int(self.box)-int(1)].pose.position.x
						pose_y = self.tocke_m[int(self.box)-int(1)].pose.position.y
						
						z = self.tocke_m[int(self.box)-int(1)].pose.orientation.z
						w = self.tocke_m[int(self.box)-int(1)].pose.orientation.w
						x = self.tocke_m[int(self.box)-int(1)].pose.orientation.x
						y = self.tocke_m[int(self.box)-int(1)].pose.orientation.y
						
						z_euler = quaternion_to_euler(w, x, y, z)
						
						pid_x = Float64()
						pid_y = Float64()
						pid_yaw = Float64()
						pid_enable = String()
						pid_x = pose_x
						pid_y = pose_y
						pid_yaw = z_euler
						pid_enable = 'True'
						
						print("Palim PID!")
						self.pub_pid_x.publish(pid_x)
						self.pub_pid_y.publish(pid_y)
						self.pub_pid_yaw.publish(pid_yaw)
						self.pub_pid_enable.publish(pid_enable)
						
						print("x pozicija" + str(abs(self.current_pose[0]-pose_x)))
						print("y pozicija" + str(abs(self.current_pose[1]-pose_y)))
						while True:
							print(abs(self.current_pose[0]-pose_x))
							print(abs(self.current_pose[1]-pose_y))
							if  (abs(self.current_pose[0]-pose_x)<0.06):
								break
						print("Gasim PID")
						pid_enable = 'False'
						self.pub_pid_enable.publish(pid_enable)
						self.box_counter += int(1)
				self.box_counter = int(0)
				
				print("KRECE SEKVENCA NOSACA!")
				mjeh_slanje = String()
				mjeh_slanje = 'Dole'
				self.pub_mjeh.publish(mjeh_slanje)
				sleep(1)
				while self.mjeh_status == 'Zauzeto':
					sleep(0.5)
					
				print("Šaljem u pre node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()
				
				print("Šaljem u KONTRA node: " + str(self.box))
				
				self.goal.target_pose.header.frame_id = 'map' 
				self.goal.target_pose.pose.position.x = self.pre_node_kontra[int(self.box)-int(1)].pose.position.x
				self.goal.target_pose.pose.position.y = self.pre_node_kontra[int(self.box)-int(1)].pose.position.y
				self.goal.target_pose.pose.orientation.z = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.z
				self.goal.target_pose.pose.orientation.w = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.w
				self.goal.target_pose.pose.orientation.x = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.x
				self.goal.target_pose.pose.orientation.y = self.pre_node_kontra[int(self.box)-int(1)].pose.orientation.y
				self.client.send_goal(self.goal)
				self.client.wait_for_result()

if __name__ == '__main__':
	try:
		master = MASTERNode()
		master.run()
	except rospy.ROSInterruptException:
		pass
