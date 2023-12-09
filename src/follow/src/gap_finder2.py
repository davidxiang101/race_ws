#!/usr/bin/env python
from __future__ import division
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
import copy
import tf


steering_marker_pub = rospy.Publisher(
    "/car_4/steering_angle_marker", Marker, queue_size=1
)
command_pub = rospy.Publisher("/car_4/offboard/command", AckermannDrive, queue_size=1)
disparity_pub = rospy.Publisher(
    "/car_4/disparity_extension", MarkerArray, queue_size=10
)
laser_pub = rospy.Publisher("/car_4/scan", LaserScan, queue_size=10)


def angle_to_radians(angle):
    return math.radians(angle)


def radians_to_angle(radians):
    return math.degrees(radians)


def preprocess(data, max_distance=5.0):
    num_points = len(data.ranges)
    center_index = num_points // 2
    angle_range = data.angle_max - data.angle_min

    forward_angle_min = -math.pi / 2  # -90 degrees
    forward_angle_max = math.pi / 2  # 90 degrees
    start_index = int((forward_angle_min - data.angle_min) / angle_range * num_points)
    end_index = int((forward_angle_max - data.angle_min) / angle_range * num_points)

    forward_scan = data.ranges[start_index : end_index + 1]

    processed_scan = [
        min(max_distance, r) if not math.isnan(r) else max_distance
        for r in forward_scan
    ]
    print(len(data.ranges), len(processed_scan))

    return processed_scan


def disparity_extension(
    processed_data, angle_increment, car_width=0.20, clearance_threshold=0.04
):
    lidar = processed_data
    new_lidar = copy.deepcopy(lidar)
    for i, scan_dist in enumerate(lidar):
        if scan_dist < clearance_threshold:
            continue
        k = int(
            math.atan((car_width + clearance_threshold) / scan_dist) / angle_increment
        )
        for j in range(max(0, i - k), min(len(lidar), i + k + 1)):
            new_lidar[j] = min(new_lidar[j], lidar[i])

    return new_lidar


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


def publish_disparity_data(
    extended_data, angle_min, angle_max, angle_increment, frame_id="car_4_laser"
):
    marker_array = MarkerArray()
    num_points = len(extended_data)

    # Calculate start and end angles for the forward-facing scan
    forward_angle_min = -math.pi / 2  # -90 degrees
    forward_angle_max = math.pi / 2  # 90 degrees

    # Calculate the starting index based on the minimum forward angle
    start_index = int((forward_angle_min - angle_min) / angle_increment)
    end_index = start_index + num_points

    for i in range(num_points):
        angle = angle_min + (start_index + i) * angle_increment
        distance = extended_data[i]

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = distance * math.cos(angle)
        marker.pose.position.y = distance * math.sin(angle)
        marker.pose.position.z = 0
        marker.scale.x = 0.05  # Small sphere size
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Alpha must be non-zero
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)

    disparity_pub.publish(marker_array)


def publish_steering_marker(steering_angle, frame_id="car_4_laser"):
    steering_marker = Marker()
    steering_marker.header.frame_id = frame_id
    steering_marker.header.stamp = rospy.Time.now()
    steering_marker.ns = "steering_angle_marker"
    steering_marker.id = 0
    steering_marker.type = Marker.ARROW
    steering_marker.action = Marker.ADD
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

    steering_angle_rads = math.radians(steering_angle)
    quat = tf.transformations.quaternion_from_euler(0, 0, steering_angle_rads)

    steering_marker.pose.orientation.x = quat[0]
    steering_marker.pose.orientation.y = quat[1]
    steering_marker.pose.orientation.z = quat[2]
    steering_marker.pose.orientation.w = quat[3]

    steering_marker_pub.publish(steering_marker)


def callback(data):
    processed_data = preprocess(data)

    extended_data = disparity_extension(processed_data, data.angle_increment)

    publish_disparity_data(extended_data, -90, 90, data.angle_increment)

    gap_dir = find_gap(processed_data)

    command = AckermannDrive()
    command.steering_angle = transform_steering(gap_dir)
    command.speed = dynamic_speed(command.steering_angle)
    command_pub.publish(command)

    publish_steering_marker(command.steering_angle)


if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("gap_finder", anonymous=True)
    rospy.Subscriber("/car_4/scan", LaserScan, callback)
    rospy.spin()
