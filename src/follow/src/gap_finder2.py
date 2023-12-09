#!/usr/bin/env python
from __future__ import division
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
import copy


# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher("error", pid_input, queue_size=10)
laser_pub = rospy.Publisher("/car_4/scan", LaserScan, queue_size = 10)


def angle_to_radians(angle):
    return math.radians(angle)

def radians_to_angle(radians):
    return math.degrees(radians)

def preprocess_lidar(data):
    ranges = data.ranges
    left = int(angle_to_radians(44) / data.angle_increment)
    right = len(ranges) - int(angle_to_radians(30) / data.angle_increment)

    subarray = ranges[left : right + 1]
    # data.angle_min += left * data.angle_increment
    # data.angle_max = data.angle_min + ((right - left) * data.angle_increment)
    # print(data.angle_min, data.angle_max)

    nans = []
    for i in range(len(subarray)):
        if math.isnan(subarray[i]):
            nans.append(i)
    
    return subarray

def find_gap(data):
    processed_data = preprocess_lidar(data)

def callback(data):
    global pub
    global vel
    global error

    processed_data = preprocess_lidar(data)

    gap = find_gap(processed_data)

    
    


if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("gap_finder", anonymous=True)
    # TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
    rospy.Subscriber("/car_4/scan", LaserScan, callback)

    rospy.spin()
