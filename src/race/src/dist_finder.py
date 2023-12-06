#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np

# Some useful variable declarations.
angle_range = 240  # Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 2.5  # distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9  # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15  # this vel variable is not really used here.
error = 0.0  # initialize the error
car_length = 0.50  # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher("error", pid_input, queue_size=10)


def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.

	#see how far input angle is from min_angle

	#get that index

	#angle_max - angle_min

	#(our angle - min) / increment
	print(data.angle_min)
	print(data.angle_max)
	k = int(math.floor((angle - data.angle_min) / data.angle_increment))
	if math.isnan(data.ranges[k]):
		if math.isnan(data.ranges[k-1]) or math.isnan(data.ranges[k+1]):
			if math.isnan(data.ranges[k-2] or math.isnan(data.ranges[k+2])):
				return float('nan')
			else:
				return (data.ranges[k-2]+data.ranges[k+2])/2
		else:
			return (data.ranges[k-1]+data.ranges[k+1])/2
	dist = data.ranges[k]
	return dist

    #TODO: implement



def callback(data):
	global pub
	global forward_projection
	global desired_distance
	global vel
	global error

	theta = math.radians(60) # you need to try different values for theta
	a = getRange(data,math.radians(-30)) # obtain the ray distance for theta
	b = getRange(data,math.radians(-90))	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	
	if math.isnan(a):
		error = float('nan')
	
	else:
		## Your code goes here to determine the projected error as per the alrorithm
		# Compute Alpha, AB, and CD..and finally the error.

		#compute alpha -- tan -1((alpha cos(theta)-b) /alpha sin(theta))
		alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))

		#compute AB -- b cos(alpha)
		AB = b*(math.cos(alpha))

		#compute CD -- AB + AC sin(alpha) -- AC is forward prjection (reasonable value)
		AC = forward_projection
		CD = AB + AC*(math.sin(alpha))

		#compute error -- desired_distance - CD

		error = CD - desired_distance

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	# print(error)
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_4/scan",LaserScan,callback)
	rospy.spin()