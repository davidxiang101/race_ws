#!/usr/bin/env python
from __future__ import division
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
import copy

steering_marker_pub = rospy.Publisher(
    "/car_4/steering_angle_marker", Marker, queue_size=1
)
command_pub = rospy.Publisher(
    "/car_4/offboard/command".format(car_name), AckermannDrive, queue_size=1
)
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

    nans = []
    for i in range(len(subarray)):
        if math.isnan(subarray[i]):
            nans.append(i)
    
    return subarray

def find_gap(data):
    processed_data = preprocess_lidar(data)


def transform_steering(steering_angle):
    if steering_angle > 30:
        steering_angle = 30
        print("\n\n\n\n\n\n\n\n\nEXCEED TURNING\n\n\n\n\n\n\n")
    if steering_angle < -30:
        steering_angle = -30
        print("\n\n\n\n\n\n\n\n\nEXCEED TURNING\n\n\n\n\n\n\n")
    
    command.steering_angle = steering_angle * (10.0 / 3.0)
    print("command angle: ", command.steering_angle)


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
    global pub
    global vel
    global error

    processed_data = preprocess_lidar(data)

    gap_dir = find_gap(processed_data)

    command.steering_angle = transform_steering(gap_dir)
    command.dynamic_speed = dynamic_speed(command.steering_angle)
    command_pub.publish(command)


    # visualization stuff
    steering_marker = Marker()
    steering_marker.header.frame_id = frame_id
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

    steering_marker.pose.position.x = data.pose.position.x
    steering_marker.pose.position.y = data.pose.position.y
    steering_marker.pose.position.z = 0.0

    heading = tf.transformations.euler_from_quaternion(
        (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        )
    )[2]
    steering_angle_rads = math.radians(gap_dir)
    yaw = heading + steering_angle_rads
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    steering_marker.pose.orientation.x = data.pose.orientation.x
    steering_marker.pose.orientation.y = data.pose.orientation.x
    steering_marker.pose.orientation.z = quat[2]
    steering_marker.pose.orientation.w = quat[3]

    steering_marker.color.a = 1.0
    steering_marker.color.g = 1.0

    steering_marker_pub.publish(steering_marker)



if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("gap_finder", anonymous=True)
    # TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
    rospy.Subscriber("/car_4/scan", LaserScan, callback)

    rospy.spin()
