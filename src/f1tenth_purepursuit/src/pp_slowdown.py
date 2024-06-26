#!/usr/bin/env python
from __future__ import division
import rospy
import math
import os
import sys
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
from collections import deque, defaultdict
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import copy
import tf
import heapq
from scipy.spatial import KDTree

rviz_angle = 0


steering_marker_pub = rospy.Publisher(
    "/car_4/steering_angle_marker2", Marker, queue_size=1
)
command_pub = rospy.Publisher("/car_4/offboard/command", AckermannDrive, queue_size=1
)
disparity_pub = rospy.Publisher(
    "/car_4/disparity_extension", MarkerArray, queue_size=10
)
polygon_pub = rospy.Publisher(
    "/car_4/purepursuit_control/visualize".format('car_4'),
    PolygonStamped,
    queue_size=1,
)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
    # raceline_pub.publish(raceline)

goal_pub = rospy.Publisher("/goal", Marker, queue_size=1)


pp_ang = False
angle_increment = 0
num_points = 0
plan = []
path_resolution = []
frame_id = "map"
raceline = None
obstacle_detected = False
trajectory_name = rospy.get_param("~arg1", "test_demoline2.csv")
car_name = rospy.get_param("~arg2", "car_4")
global wp_seq
global curr_polygon
max_speed = 20.0
min_speed = 10.0

speed_factors = []

wp_seq = 0
control_polygon = PolygonStamped()

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/path/raceline_final_smooth8c.csv"
    )
    # file_path = os.path.expanduser('~/map_ws/src/raceline.csv'.format(trajectory_name))

    global speed_factors

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        for waypoint in csv_reader:
            plan.append(waypoint[:4])  # x, y, z, w
            speed_factors.append(float(waypoint[4]))  # Speed factor

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
        dx = plan[index][0] - plan[index - 1][0]
        dy = plan[index][1] - plan[index - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    raceline_path = Path()
    raceline_path.header.frame_id = "map"

    global kd_tree
    coordinates = [
        (point[0], point[1]) for point in plan
    ]  # Assuming x, y coordinates are the first two elements
    kd_tree = KDTree(coordinates)

    for index, point in enumerate(plan):
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = point[0]
        waypoint.pose.position.y = point[1]
        raceline_path.poses.append(waypoint)

    global raceline
    raceline = raceline_path
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    # raceline_pub.publish(raceline_path)
    #     rate.sleep()

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN = 0.325

def purepursuit_control_node(data):
    start_time = rospy.get_time()
    # publish reference path for RVI
    global raceline
    global wp_seq
    global curr_polygon
    global obstacle_detected
    global gap_angle

    command = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    # print(plan)

    # Query the k-d tree for the nearest neighbor
    # uncomment when ready
    odom_position = (data.pose.position.x, data.pose.position.y)
    base_proj_idx = kd_tree.query(odom_position)[1]
    base_proj_pos = plan[base_proj_idx][:2]  # x, y
    pose_x, pose_y = base_proj_pos

    # base_proj_idx = 0
    # base_proj_dist = sys.maxsize
    # base_proj_pos = (0, 0)

    # for idx in range(len(plan)):
    #     x, y, z, w = plan[idx]
    #     dist_to_point = math.sqrt(math.pow(odom_x - x, 2) + math.pow(odom_y - y, 2))
    #     if base_proj_dist > dist_to_point:
    #         base_proj_dist = dist_to_point
    #         base_proj_pos = (x, y)  # only x, y
    #         base_proj_idx = idx
    # pose_x, pose_y = base_proj_pos

    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion(
        (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        )
    )[2]

    # print(base_proj_idx)
    # TODO 2: You need to tune the value of the lookahead_distance
    # lookahead_distance = 1.83

    # dynamic lookahead distance (needs to be tuned and tested)
    BASE_DISTANCE = 0.8
    MAX_DISTANCE = 2.2
    lookahead_distance = BASE_DISTANCE + (
        (speed_factors[base_proj_idx]) * (MAX_DISTANCE - BASE_DISTANCE)
    )

    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    curr_lookahead_dist = 0
    goal_idx = base_proj_idx

    while (
        math.sqrt(
            ((plan[goal_idx][0] - odom_x) ** 2) + ((plan[goal_idx][1] - odom_y) ** 2)
        )
        < lookahead_distance
    ):
        goal_idx += 1
        if goal_idx >= len(plan):
            goal_idx = 0

    # After the loop, make sure goal_idx is valid for 'plan'
    goal_idx = goal_idx % len(path_resolution)
    goal_pos = plan[goal_idx]  # gives x, y, z, w
    target_x, target_y = goal_pos[:2]
    # print(base_proj_idx, goal_idx)

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here

    # change to reference frame of the car
    translated_x = goal_pos[0] - odom_x
    translated_y = goal_pos[1] - odom_y

    rotated_x = translated_x * math.cos(-heading) - translated_y * math.sin(-heading)
    rotated_y = translated_x * math.sin(-heading) + translated_y * math.cos(-heading)

    # target_x, target_y = rotated_x, rotated_y
    y_t = rotated_y
    x_t = rotated_x
    dist_to_goal = math.sqrt(x_t * x_t + y_t * y_t)
    alpha = math.asin(y_t / dist_to_goal)
    steering_angle = math.degrees(
        math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / dist_to_goal)
    )
    # print("steering angle: ", steering_angle)

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here

    # -100 command correlates to 30 degrees right of the car
    # 100 command correlates to 30 degrees left of the car
    # 0 is straight (0 degrees)
    if steering_angle > 30:
        steering_angle = 30
        print("\nEXCEED TURNING\n")
    if steering_angle < -30:
        steering_angle = -30
        print("\nEXCEED TURNING\n")

    rviz_angle = steering_angle * (10.0 / 3.0)

    command.steering_angle = steering_angle * (10.0 / 3.0)
    # print("command angle: ", command.steering_angle)

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed

    global max_speed
    global min_speed

    # uncomment when ready
    current_speed_factor = speed_factors[base_proj_idx]
    dynamic_speed = current_speed_factor * (max_speed - min_speed) + min_speed
    command.speed = dynamic_speed

    # # if obstacle detected within 0.5 m directly ahead stop the car
    # if obstacle_detected == True:
    #     #command.steering_angle = transform_steering(gap_angle)
    #     command.speed = 0

    # error = 1 - (abs(command.steering_angle) / STEERING_RANGE)
    # print("error: ", error)
    # dynamic_speed = (error) * (max_speed - min_speed) + min_speed
    # command.speed = min(max_speed, dynamic_speed)
    # command.speed = max(min_speed, dynamic_speed)
    # command.speed = dynamic_speed
    # print("dynamic speed: ", command.speed)

    command_pub.publish(command)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point4

    # These are set to zero only so that the template code builds.

    base_link = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x = odom_x
    base_link.y = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    wp_seq = wp_seq + 1
    #polygon_pub.publish(control_polygon)

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

    steering_marker.pose.position.x = odom_x
    steering_marker.pose.position.y = odom_y
    steering_marker.pose.position.z = 0.0

    steering_angle_rads = steering_angle * (3.14159 / 180.0)
    yaw = heading + steering_angle_rads
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    steering_marker.pose.orientation.x = data.pose.orientation.x
    steering_marker.pose.orientation.y = data.pose.orientation.y
    steering_marker.pose.orientation.z = quat[2]
    steering_marker.pose.orientation.w = quat[3]

    steering_marker.color.a = 1.0
    steering_marker.color.g = 1.0

    steering_marker_pub.publish(steering_marker)

    print(
        "Elapsed: {:.2f} ms, Steering Angle: {:.2f}, Command Angle: {:.2f}, Speed: {:.2f}, Obstacle Detected: {}".format(
            (rospy.get_time() - start_time) * 1000,
            steering_angle,
            command.steering_angle,
            command.speed,
            obstacle_detected = None,
        )
    )



def preprocess(data, max_distance=3.0):
    # Convert data.ranges to a NumPy array for efficient processing
    scan_ranges = np.array(data.ranges)

    # Calculate necessary indices for slicing
    num_points = len(scan_ranges)
    angle_range = data.angle_max - data.angle_min
    forward_angle_min = -math.pi / 4  # -90 degrees
    forward_angle_max = math.pi / 4  # 90 degrees

    start_index = int((forward_angle_min - data.angle_min) / angle_range * num_points)
    end_index = int((forward_angle_max - data.angle_min) / angle_range * num_points) + 1

    # Extract the relevant scan segment
    forward_scan = scan_ranges[start_index:end_index]

    # Process the scan data
    # Replace NaNs with max_distance and cap values at max_distance
    processed_scan = np.where(np.isnan(forward_scan), max_distance, forward_scan)
    processed_scan = np.minimum(processed_scan, max_distance)


    global angle_increment
    angle_increment = data.angle_increment
    global num_points
    num_points = len(processed_scan)

    return processed_scan


def disparity_extension(
    processed_data, angle_increment, car_width=0.20, clearance_threshold=0.06
):
    # Create an array to hold the extended data
    new_lidar = processed_data.copy()

    # Vectorize the extension computation
    indices = np.arange(len(processed_data))
    for i, scan_dist in enumerate(processed_data):
        if scan_dist >= clearance_threshold:
            k = int(
                math.atan((car_width + clearance_threshold) / scan_dist)
                / angle_increment
            )
            start = max(0, i - k)
            end = min(len(processed_data), i + k + 1)
            new_lidar[start:end] = np.minimum(new_lidar[start:end], processed_data[i])

    return new_lidar


# def find_gap(extended_data, inc, height_weight=1, pp_weight=0.5):
#     global pp_ang  # this is acc an index
#     global num_points
#     max_depth = 0
#     max_ind = 0

#     for i, height in enumerate(extended_data):
#         pp_adjust = (1 + pp_weight * (1 - (abs(pp_ang - i) / num_points)))
#         if height * pp_adjust > max_depth:
#             max_depth = height * pp_adjust
#             max_ind = i

#     targ_ind = max_ind
#     return targ_ind, max_depth


def index_to_angle(index, angle_increment, num_points):
    angle_min = -math.pi / 4
    angle = math.degrees(angle_min + (index * angle_increment))
    return angle


def transform_steering(steering_angle):
    if steering_angle > 30:
        steering_angle = 30
    if steering_angle < -30:
        steering_angle = -30

    command_angle = steering_angle * (10.0 / 3.0)
    # print("command angle: ", command_angle)
    return command_angle


def dynamic_speed(command_angle):
    max_speed = 17
    min_speed = 12

    error = 1 - (abs(command_angle) / 100)
    dynamic_speed = (error) * (max_speed - min_speed) + min_speed
    dynamic_speed = min(max_speed, dynamic_speed)
    dynamic_speed = max(min_speed, dynamic_speed)
    # print("dynamic speed: ", dynamic_speed)

    return dynamic_speed


def publish_disparity_data(
    extended_data, angle_min, angle_max, angle_increment, frame_id="car_4_laser"
):
    marker_array = MarkerArray()
    num_points = len(extended_data)

    # Calculate start and end angles for the forward-facing scan
    forward_angle_min = -math.pi /4  # -90 degrees
    forward_angle_max = math.pi / 4  # 90 degrees

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
    steering_marker.color.r = 1.0
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

    publish_disparity_data(extended_data, -45, 45, data.angle_increment)

    # best_gap_index, max_area = find_gap(extended_data, data.angle_increment)

    # gap_angle = index_to_angle(best_gap_index, data.angle_increment, len(extended_data))

    command = AckermannDrive()
    # command.steering_angle = transform_steering(gap_angle)


    command.speed = dynamic_speed(command.steering_angle) # might delete
    threshold = 1.2
    if extended_data[pp_ang] < threshold or extended_data[len(extended_data) // 2] < (threshold - 0.3): #mayb + L&R of mid
        #command.speed = command.speed * -1
        # slow it down
        command.speed = dynamic_speed(command.steering_angle) * 0.3
    else:
        command.speed = dynamic_speed(command.steering_angle)
    command_pub.publish(command)

    global rviz_angle
    publish_steering_marker(rviz_angle)



def angle_to_index(steering_angle, angle_increment, num_points):
    angle_min = -math.pi / 4
    angle_rad = math.radians(steering_angle)
    index = int((angle_rad - angle_min) / angle_increment)
    return min(max(index, 0), num_points - 1)


def pp_callback(data):
    global pp_ang
    global angle_increment
    global num_points

    command_angle = data.steering_angle     #angle of goal projection
    command_angle = min(command_angle, 45)
    command_angle = max(command_angle, -45)
    ang = angle_increment
    num = num_points

    index = angle_to_index(command_angle, ang, num)
    # print('command_angle: ', command_angle)
    pp_ang = index


if __name__ == "__main__":
    print("Hokuyo LIDAR node started")
    rospy.init_node("pure_pursuit", anonymous=True)
    rospy.Subscriber("/car_4/scan", LaserScan, callback)
    #rospy.Subscriber("/pure_pursuit/angle", AckermannDrive, pp_callback)
    #print("gap finder started")
    rospy.spin()
