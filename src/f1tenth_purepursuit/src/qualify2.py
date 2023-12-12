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
obstacle_detected = False


disparity_pub = rospy.Publisher("/car_4/disparity_extension", MarkerArray, queue_size=1)
slow_down = False
disparity_pub = rospy.Publisher("/car_4/disparity_extension", MarkerArray, queue_size=1)
steering_marker_pub = rospy.Publisher(
    "/car_4/steering_angle_marker", Marker, queue_size=1
)
command_pub = rospy.Publisher(
    "/car_4/offboard/command".format(car_name), AckermannDrive, queue_size=1
)
polygon_pub = rospy.Publisher(
    "/car_4/purepursuit_control/visualize".format(car_name),
    PolygonStamped,
    queue_size=1,
)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
goal_pub = rospy.Publisher("/goal", Marker, queue_size=1)
# Global variables for waypoint sequence and current polygon


global wp_seq
global curr_polygon

max_speed = 40.0
min_speed = 30.0

speed_factors = []

wp_seq = 0
control_polygon = PolygonStamped()


def construct_path():
    file_path = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/path/raceline_final_smooth8b.csv".format(
            trajectory_name
        )
    )
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


STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325


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


def find_gap(extended_data, inc, height_weight=100000000, width_weight=1):
    max_depth = 0
    max_ind = 0

    for i, height in enumerate(extended_data):
        if height > max_depth:
            max_depth = height
            max_ind = i

    targ_ind = max_ind

    return targ_ind, max_depth


def index_to_angle(index, angle_increment, num_points):
    angle_min = -math.pi / 2
    angle = math.degrees(angle_min + (index * angle_increment))
    return angle


def transform_steering(steering_angle):
    if steering_angle > 30:
        steering_angle = 30
        print("\nEXCEED TURNING\n")
    if steering_angle < -30:
        steering_angle = -30
        print("\nEXCEED TURNING\n")

    command_angle = steering_angle * (10.0 / 3.0)
    return command_angle


def purepursuit_control_node(data):
    start_time = rospy.get_time()
    global raceline
    global wp_seq
    global curr_polygon
    global obstacle_detected
    global gap_angle
    global slow_down

    raceline_pub.publish(raceline)
    command = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # kd tree for base pos
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

    # dynamic lookahead dist
    BASE_DISTANCE = 1.2
    MAX_DISTANCE = 1.8
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

    command.steering_angle = steering_angle * (10.0 / 3.0)
    # print("command angle: ", command.steering_angle)

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed

    global max_speed
    global min_speed

    # uncomment when ready
    current_speed_factor = speed_factors[base_proj_idx]
    dynamic_speed = current_speed_factor * (max_speed - min_speed) + min_speed
    steering_offset = -8
    final_speed = max(-100, dynamic_speed + steering_offset)
    command.speed = final_speed

    # if nearing obstacle
    if slow_down == True:
        command.speed = final_speed * 0.6

    # if obstacle detected within 0.5 m directly ahead stop the car
    if obstacle_detected == True:
        # command.speed = 0
        command.speed = final_speed / 3  # slow down a ton
        command.steering_angle = transform_steering(
            gap_angle
        )  # find the gap and follow it
        # maybe publish steering angle to rviz here

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
    polygon_pub.publish(control_polygon)

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
            obstacle_detected,
        )
    )


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
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0

        marker_array.markers.append(marker)

    disparity_pub.publish(marker_array)


# callback for the laser scan, computes gap finder stuff and finds best gap if obstacle detected
def callback(data):
    global obstacle_detected
    global gap_angle
    global slow_down
    processed_data = preprocess(data)
    extended_data = disparity_extension(processed_data, data.angle_increment)
    publish_disparity_data(extended_data, -90, 90, data.angle_increment)
    print("middle length = ", extended_data[len(extended_data) // 2])

    # if there's something within 1m in any of the middle 3 indeces
    for datapoint in extended_data[
        len(extended_data) // 2 - 1 : len(extended_data) // 2 + 1
    ]:
        if datapoint < 1.0:
            obstacle_detected = True
            best_gap_index, max_depth = find_gap(extended_data, data.angle_increment)
            gap_angle = index_to_angle(
                best_gap_index, data.angle_increment, len(extended_data)
            )
            publish_steering_marker(gap_angle)
        elif datapoint < 2:
            slow_down = True

        else:
            obstacle_detected = False
            slow_down = False

    # print("Obstacle detected: ", obstacle_detected)
    print("slow down?: ", slow_down)


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)
        if not plan:
            rospy.loginfo("obtaining trajectory")
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter.
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber("/car_4/scan", LaserScan, callback, queue_size=1)
        rospy.Subscriber(
            "/car_4/particle_filter/viz/inferred_pose".format(car_name),
            PoseStamped,
            purepursuit_control_node,
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
