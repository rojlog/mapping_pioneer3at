#!/usr/bin/env python3

import rospy
import tf2_ros

import cv2

import numpy as np
import matplotlib.pyplot as plt
import math as mt
from time import sleep

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import astartmod as astart


#class Pioneer_3AT():
class Pioneer3AT():
	def __init__(self):
		
		# twist()
		self.move_cmd = Twist()
		
		# linear velocity
		self.linear_speed = 0.2
		
		# angular velocity and move
		self.angular_speed = 0.5 # radians for second
		
		# publisher
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		
		# time sleep one second
		self.move_time = 1.0
		
		# features of the sensor (lidar)
		self.ratio = 1.5    # [meters]
		self.angle = 0.5    # []
		#self.origin_lidar = 
		
		
		# posicion del robot
		
		
		
	def move_forward(self):
		self.move_cmd.linear.x = self.linear_speed
		self.cmd_pub.publish(self.move_cmd)
		
		sleep(self.move_time)
		
		#Stop Pioneer
		self.move_cmd.linear.x = 0.0
		self.cmd_pub.publish(self.move_cmd)
	
	def move_angular(self, direction, angle):
    		
		# Tiempo para girar (ajustar para cambiar el ángulo)
		turn_time = angle / (self.angular_speed * 57.2958)  # 1 rad = 180/pi = 57.2958 Convertir ángulo a radianes y calcular el tiempo
                
		# Crear un mensaje de tipo Twist
		print(turn_time)
		self.move_cmd.angular.z = self.angular_speed if direction == 'left' else -self.angular_speed

		# Publicar el comando de giro
		self.cmd_pub.publish(self.move_cmd)
					
		# Esperar el tiempo especificado
		sleep(turn_time)
					
		# stop Pioneer
		self.move_cmd.angular.z = 0
		self.cmd_pub.publish(self.move_cmd)	


	#def geometry_laser(self):
	
	
#def position_gazebo_to_world():





def Map(msg):
	data = np.array(msg.data)
	width = msg.info.width
	height = msg.info.height
	
	grid = data.reshape((height, width))
	num_free = np.count_nonzero(grid == 0)
	
	num_unknown = np.count_nonzero(grid == -1)
	num_occupied = np.count_nonzero(grid == 100)
	print(f"Unknown: {num_unknown}, Free: {num_free}, Occupied: {num_occupied}")
	
	entropy = (num_occupied+num_free) / (width * height)
	
	print('Entropy = ', entropy)
    	
    	
    
	

rospy.init_node('move_Pioneer3AT')




if __name__=='__main__':
	rospy.init_node('Frontier_Approch')

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	#print(tfBuffer)
	try:
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			trans = tfBuffer.lookup_transform('odom','base_link', rospy.Time(0), rospy.Duration(1.0)) # Tener una funcion o ver como hay informacion en el buffer
			# rospy.Subscriber('/map', OccupancyGrid, Map, trans)
			print(trans)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass