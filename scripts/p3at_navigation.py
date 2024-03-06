#!/usr/bin/env python3

import rospy
import cv2
import tf2_ros

import numpy as np
import matplotlib.pyplot as plt
import math as mt

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

import astartmod as astart


class Pioneer_3AT():
	



if __name__=='__main__':
	rospy.init_node('Frontier_Approch')

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	try:
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			trans = tfBuffer.lookup_transform('odom','base_link', rospy.Time(0), rospy.Duration(1.0)) # Tener una funcion o ver como hay informacion en el buffer
			rospy.Subscriber('/map', OccupancyGrid, Map, trans)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass