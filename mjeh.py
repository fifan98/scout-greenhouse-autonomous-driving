#!/usr/bin/env python3

import rospy
import serial

from time import sleep
from std_msgs.msg import Float64, Bool, String

class MJEHNode:
	def __init__(self):
		rospy.init_node('mjeh_node')
		
		# Set up the subscriber for the mjeh 
		self.sub_mjeh = rospy.Subscriber("/mjeh", String, self.callback_mjeh)
		
		# Set up publisher for /mjeh_status
		self.pub_mjeh = rospy.Publisher('/mjeh_status', String, queue_size=10)
		
	def callback_mjeh(self, data):
		mjeh_status = String()
		if data.data == 'Gore':
			mjeh_status = 'Zauzeto'
			self.pub_mjeh.publish(mjeh_status)
			print("Podizem nosac!")
			ser = serial.Serial('/dev/ttyUSB0', 9600)
			sleep(2)
			br = '1'
			ser.write(br.encode())
			print("Poslana komanda za podizanje mjeha!")
			line = ser.readline().decode().strip()
			line = int(line)
			while line < 400:
				line = ser.readline().decode().strip()
				line = int(line)
			print("Nosac podignut!")
			ser.close()
			mjeh_status = 'Slobodno'
			self.pub_mjeh.publish(mjeh_status)
		elif data.data == 'Dole':
			mjeh_status = 'Zauzeto'
			self.pub_mjeh.publish(mjeh_status)
			print("Spustam nosac!")
			ser = serial.Serial('/dev/ttyUSB0', 9600)
			sleep(2)
			br = '0'
			ser.write(br.encode())
			print("Poslana komanda za spustanje mjeha!")
			line = ser.readline().decode().strip()
			line = int(line)
			while line > 50:
				line = ser.readline().decode().strip()
				line = int(line)
			print("Nosac spusten!")
			ser.close()
			mjeh_status = 'Slobodno'
			self.pub_mjeh.publish(mjeh_status)
if __name__ == '__main__':
	try:
		mjeh = MJEHNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

