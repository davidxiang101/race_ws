#!/usr/bin/env python
from __future__ import division
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
import copy

# Some useful variable declarations.
angle_range = 240  # Hokuyo 4LX has 240 degrees FoV for scan
car_width = 0.08
threshold = 0.1
width_threshold = 0.10
disparity_threshold = 0.1
max_dist_threshold = 6.0
height_weight = 5.0
width_weight = 1.0
vel = 15  # this vel variable is not really used here.
error = 0.0  # initialize the error
car_length = 0.50  # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
previous_max_ind = 0
previous_max_area = 0

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher("error", pid_input, queue_size=10)


def find_best_gap2(free_space_ranges, inc):
    free_space_ranges = disparity_extender_dynamic(free_space_ranges, inc)
    stk = []  # height, startind
    max_area = 0
    max_area_ind = 0

    for i, height in enumerate(free_space_ranges):
        if not stk or height > stk[-1][0]:
            stk.append((height, i))
        else:
            earliest = i
            while stk and stk[-1][0] > height:
                prev = stk.pop()
                prev_index = prev[1]
                area = (height_weight * prev[0]) * (width_weight * (i - prev[1]))
                if area > max_area:
                    max_area = area
                    max_area_ind = (prev_index + i) >> 1
            stk.append((height, earliest))

    while stk:
        prev = stk.pop()
        prev_index = prev[1]
        area = (height_weight * prev[0]) * (width_weight * (i - prev[1]))
        if area > max_area:
            max_area = area
            max_area_ind = (prev_index + i) >> 1
    print("max_area", max_area, max_area_ind)
    previous_max_ind = max_area_ind
    previous_max_area = max_area
    return max_area_ind


def find_best_gap(free_space_ranges, inc):
    stk = []  # height, startind
    max_area = 0
    max_area_ind = 0

    for i, height in enumerate(free_space_ranges):
        if not stk or height > stk[-1][0]:
            stk.append((height, i))
        else:
            earliest = i
            while stk and stk[-1][0] > height:
                prev = stk.pop()
                prev_index = prev[1]
                k = 0
                if prev[0] != 0:
                    k = math.atan((car_width + width_threshold) / prev[0]) / inc
                if i - prev_index > k:
                    area = (height_weight * prev[0]) * (width_weight * (i - prev[1]))
                    if area > max_area:
                        max_area = area
                        max_area_ind = (prev_index + i) >> 1
            stk.append((height, earliest))

    while stk:
        prev = stk.pop()
        prev_index = prev[1]
        k = 0
        if prev[0] != 0:
            k = math.atan((car_width + width_threshold) / prev[0]) / inc
        if i - prev_index > k:
            area = (height_weight * prev[0]) * (width_weight * (i - prev[1]))
            if area > max_area:
                max_area = area
                max_area_ind = (prev_index + i) >> 1
    print("max_area1", max_area)
    previous_max_ind = max_area_ind
    previous_max_area = max_area
    return max_area_ind


def find_max_gap(free_space_ranges):
    max_length = max(free_space_ranges)
    max_gap_straightest_ind = 0
    closest_dist_to_mid = len(free_space_ranges)
    mid = len(free_space_ranges) >> 1

    for i, num in enumerate(free_space_ranges):
        if num == max_length:
            if abs(mid - i) < closest_dist_to_mid:
                closest_dist_to_mid = abs(mid - i)
                max_gap_straightest_ind = i
    return max_gap_straightest_ind


def preprocess_lidar(data):
    """Preprocess the LiDAR scan array. Expert implementation includes:
    1.Setting each value to the mean over some window
    2.Rejecting high values (eg. > 3m)
    """
    ranges = data.ranges
    # remove 15 degrees of lidar values from each side of ranges

    left = int(round((math.radians(44)) / data.angle_increment))
    right = len(ranges) - int(round((math.radians(30)) / data.angle_increment))

    subarray = list(ranges[left : right + 1])
    # reject high values
    for i in range(len(subarray)):
        if math.isnan(subarray[i]) or subarray[i] > max_dist_threshold:
            subarray[i] = max_dist_threshold        # this should workF

    # print(subarray)
    # hist = defaultdict(int)
    # for num in subarray:
    # 	hist[int(round(num))] += 1
    # print(hist)

    # print(subarray)
    return subarray


def disparity_extender_dynamic(lidar, inc):
    new_lidar = copy.deepcopy(lidar)
    for i, scan_dist in enumerate(lidar):
        if scan_dist < threshold:
            continue
        k = int(math.atan((car_width + threshold) / scan_dist) / inc)
        for j in range(max(0, i - k), min(len(lidar), i + k + 1)):
            new_lidar[j] = min(new_lidar[j], lidar[i])

    # hist = defaultdict(int)
    # for num in new_lidar:
    #     hist[num] += 1
    # print(hist)
    print("max", max(new_lidar))
    return new_lidar


def disparity_extender(lidar, k):
    dq = deque()
    left_extended = [0] * len(lidar)
    # loop through lidar and keep track of smallest within k range
    for i in range(len(lidar)):
        while dq and lidar[i] < lidar[dq[-1]]:
            dq.pop()
        dq.append(i)
        if i - dq[0] > k:
            dq.popleft()
        left_extended[i] = lidar[dq[0]]

    dq = deque()
    right_extended = [0] * len(lidar)
    # loop through lidar and keep track of smallest within k range
    for i in range(len(lidar) - 1, -1, -1):
        while dq and lidar[i] < lidar[dq[-1]]:
            dq.pop()
        dq.append(i)
        if dq[0] - i > k:
            dq.popleft()
        right_extended[i] = lidar[dq[0]]

    # combine left and right extended by taking min
    extended = [min(left_extended[i], right_extended[i]) for i in range(len(lidar))]
    return extended


def find_gap(data):
    # data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view

    lidar = preprocess_lidar(data)

    # naive k, change to be respective to the distance returned by the lidar later
    k = int(round(12 / 180 * len(lidar)))

    # extend the lidar data
    # adjusted_gaps = disparity_extender(lidar, k)
    # adjusted_gaps2 = disparity_extender_dynamic(lidar, data.angle_increment)

    # find the largest gap
    # gap_theta = find_max_gap(adjusted_gaps)
    # gap_theta2 = find_max_gap(adjusted_gaps2)
    # print(gap_theta, adjusted_gaps[gap_theta], max(adjusted_gaps))

    # normalize index to -90, 90
    # print("deepest gap: ", adjusted_gaps[gap_theta])
    gap_theta = find_best_gap(lidar, data.angle_increment)
    angle = (gap_theta / len(lidar)) * 180

    return round(angle) - 90


def callback(data):
    global pub
    global vel
    global error

    gap_theta = find_gap(data)
    error = gap_theta
    # print('Angle With Best Gap: ', gap_theta)
    # calculate error here
    # if gap_theta is not None:
    # 	# convert gap_theta (index of lidar) to angle in rad (0 index is -90 deg and max index is 90 deg)
    # 	# print(gap_theta * data.angle_increment)
    # 	# error = -1 * math.radians(gap_theta * data.angle_increment - 90)
    # 	index_error = gap_theta - len(data.ranges)/2
    # 	angle_error = index_error * data.angle_increment
    # 	print(math.degrees(angle_error))
    # 	d = data.ranges[gap_theta]
    # 	error = d*math.sin(angle_error)

    # else:
    # 	error = 0

    msg = pid_input()
    msg.pid_error = -1 * error * (10/3)
    print(-1 * error)
    msg.pid_vel = vel
    pub.publish(msg)


if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("gap_finder", anonymous=True)
    # TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
    rospy.Subscriber("/car_4/scan", LaserScan, callback)
    rospy.spin()
