#!/usr/bin/env python

import rospy
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import String

if __name__ == '__main__' :
	# Node initialization
	rospy.init_node('distance_calculator', anonymous=True)
	
	# Define the publisher
	pub = rospy.Publisher('talker', String, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		
		# Get one data sample from the /odom topic.
		msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
		
		# Extract the information you want from the message
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		
		# compute the 2D distance from origin (0, 0)
		distance = math.sqrt(x**2 + y**2)
		
		# Publish the desired message
		publishString = "2D distance from the origin: {}".format(distance)
		pub.publish(publishString)		
		
		rate.sleep()
		 
	
