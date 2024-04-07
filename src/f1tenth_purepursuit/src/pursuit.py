#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf
from scipy.spatial import KDTree
import copy

# Global variables for storing the path, path resolution, frame ID, and car details
plan = []
path_resolution = []
frame_id = "map"

raceline = None
steering_marker_pub = rospy.Publisher(
    "/car_8/steering_angle_marker", Marker, queue_size=1
)
# Publishers for sending driving commands and visualizing the control polygon
command_pub = rospy.Publisher(
    "/pure_pursuit/command", AckermannDrive, queue_size=2
)
polygon_pub = rospy.Publisher(
    "/car_8/purepursuit_control/visualize",
    PolygonStamped,
    queue_size=1,
)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
goal_pub = rospy.Publisher("/goal", Marker, queue_size=1)
angle_pub = rospy.Publisher("/pure_pursuit/angle", AckermannDrive, queue_size=10)


# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

max_speed = 48.0
min_speed = 4.0

# Global variable to control speed halving
sandbag = False
sandbag_factor = 0.6

speed_factors = []

wp_seq = 0
control_polygon = PolygonStamped()


def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser("~/catkin_ws/src/f1tenth_purepursuit/path/demoline20_smooth.csv")

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

    print(path_resolution)


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
    global gap_angle

    command = AckermannDrive()
    # raceline_pub.publish(raceline)

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.

    # Query the k-d tree for the nearest neighbor
    # uncomment when ready
    odom_position = (data.pose.position.x, data.pose.position.y)
    base_proj_idx = kd_tree.query(odom_position)[1]
    base_proj_pos = plan[base_proj_idx][:2]  # x, y
    pose_x, pose_y = base_proj_pos

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
    BASE_DISTANCE = 0.6
    MAX_DISTANCE = 2.1
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

    target_angle = math.atan2(rotated_y, rotated_x)

    # Create and publish the angle message
    angle_msg = AckermannDrive()
    angle_msg.steering_angle = math.degrees(target_angle)
    global angle_pub
    angle_pub.publish(angle_msg)

    # target_x, target_y = rotated_x, rotated_y
    y_t = rotated_y
    x_t = rotated_x
    steering_offset = 6
    dist_to_goal = math.sqrt(x_t * x_t + y_t * y_t)
    alpha = math.asin(y_t / dist_to_goal)
    steering_angle = (
        math.degrees(math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / dist_to_goal))
        + steering_offset
    )

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here

    if steering_angle > 30:
        steering_angle = 30
        print("\nEXCEED TURNING\n")
    if steering_angle < -30:
        steering_angle = -30
        print("\nEXCEED TURNING\n")

    command.steering_angle = steering_angle * (10.0 / 3.0)

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed

    global max_speed
    global min_speed

    # uncomment when ready
    global sandbag_factor
    current_speed_factor = speed_factors[base_proj_idx]
    dynamic_speed = current_speed_factor * (max_speed - min_speed) + min_speed
    if sandbag and current_speed_factor > 0.9:
        dynamic_speed = dynamic_speed * sandbag_factor
    command.speed = dynamic_speed

    command_pub.publish(command)

    # Visualization code

    publish_steering_marker(command.steering_angle)
    end_time = rospy.get_time()

    # print(
    #     "Elapsed: {:.2f} ms, Steering Angle: {:.2f}, Command Angle: {:.2f}, Speed: {:.2f}".format(
    #         (end_time - start_time) * 1000,
    #         steering_angle,
    #         command.steering_angle,
    #         command.speed
    #     )
    # )


def publish_steering_marker(steering_angle, frame_id="car_8_laser"):
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


def toggle_speed_callback(event):
    global sandbag
    sandbag = not sandbag


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)
        rospy.Timer(rospy.Duration(60), toggle_speed_callback)

        if not plan:
            rospy.loginfo("obtaining trajectory")
            construct_path()

        rospy.Subscriber(
            "/car_8/particle_filter/viz/inferred_pose",
            PoseStamped,
            purepursuit_control_node,
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
