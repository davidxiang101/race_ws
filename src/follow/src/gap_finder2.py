#!/usr/bin/env python
from __future__ import division
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
import copy
import tf


steering_marker_pub = rospy.Publisher(
    "/car_4/steering_angle_marker", Marker, queue_size=1
)
command_pub = rospy.Publisher(
    "/car_4/offboard/command", AckermannDrive, queue_size=1
)
laser_pub = rospy.Publisher("/car_4/scan", LaserScan, queue_size = 10)


def angle_to_radians(angle):
    return math.radians(angle)


def radians_to_angle(radians):
    return math.degrees(radians)


def preprocess(data):
    left = int(angle_to_radians(44) / data.angle_increment)
    right = len(data.ranges) - int(angle_to_radians(30) / data.angle_increment)

    subarray = data.ranges[left : right + 1]

    nans = []
    for i in range(len(subarray)):
        if math.isnan(subarray[i]):
            nans.append(i)
    
    return subarray


def find_gap(processed_data):
    return 30.0


def transform_steering(steering_angle):
    if steering_angle > 30:
        steering_angle = 30
        print("\n\n\n\n\n\n\n\n\nEXCEED TURNING\n\n\n\n\n\n\n")
    if steering_angle < -30:
        steering_angle = -30
        print("\n\n\n\n\n\n\n\n\nEXCEED TURNING\n\n\n\n\n\n\n")
    
    command_angle = steering_angle * (10.0 / 3.0)
    print("command angle: ", command_angle)
    return command_angle


def dynamic_speed(command_angle):
    max_speed = 30
    min_speed = 10

    error = 1 - (abs(command_angle) / 100)
    dynamic_speed = (error) * (max_speed - min_speed) + min_speed
    dynamic_speed = min(max_speed, dynamic_speed)
    dynamic_speed = max(min_speed, dynamic_speed)
    print("dynamic speed: ", dynamic_speed)
    
    return dynamic_speed


def callback(data):
    print(type(data))
    processed_data = preprocess(data)

    gap_dir = find_gap(processed_data)

    command = AckermannDrive()
    command.steering_angle = transform_steering(gap_dir)
    command.speed = dynamic_speed(command.steering_angle)
    command_pub.publish(command)


    # visualization stuff
    steering_marker = Marker()
    steering_marker.header.frame_id = "car_4_laser"
    steering_marker.header.stamp = rospy.Time.now()
    steering_marker.ns = "steering_angle_marker"
    steering_marker.type = Marker.ARROW
    steering_marker.scale.x = 1.0
    steering_marker.scale.y = 0.1
    steering_marker.scale.z = 0.1
    steering_marker.color.r = 0.0
    steering_marker.color.g = 0.0
    steering_marker.color.b = 1.0
    steering_marker.color.a = 1.0

    steering_marker.pose.position.x = 0.0
    steering_marker.pose.position.y = 0.0
    steering_marker.pose.position.z = 0.0

    steering_angle_rads = math.radians(gap_dir)
    quat = tf.transformations.quaternion_from_euler(0, 0, steering_angle_rads)

    steering_marker.pose.orientation.x = quat[0]
    steering_marker.pose.orientation.y = quat[1]
    steering_marker.pose.orientation.z = quat[2]
    steering_marker.pose.orientation.w = quat[3]

    steering_marker.color.a = 1.0
    steering_marker.color.g = 1.0

    steering_marker_pub.publish(steering_marker)



if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("gap_finder", anonymous=True)
    rospy.Subscriber("/car_4/scan", LaserScan, callback)
    rospy.spin()
