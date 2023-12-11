# Import necessary libraries
import rospy
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



#pub disparity data
def publish_disparity_data(
    extended_data, angle_min, angle_increment, frame_id="car_4_laser"
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

    disparity_pub = rospy.Publisher(
    "/car_4/disparity_extension", MarkerArray, queue_size=10
)

    disparity_pub.publish(marker_array)


def publish_steer_marker_data(data, steering_angle, heading, odom_x, odom_y, frame_id):

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

    steering_marker_pub = rospy.Publisher(
        "/car_4/steering_angle_marker", Marker, queue_size=1
    )
    
    steering_marker_pub.publish(steering_marker)

def create_goal_node():
    goal_pub = rospy.Publisher("/goal", Marker, queue_size=1)

def create_raceline_node():
    raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
def create_laser_node():
    laser_pub = rospy.Publisher("/car_4/scan", LaserScan, queue_size=10)

def publish_polygon_data(pose_x, pose_y, target_x, target_y, odom_x, odom_y, frame_id, car_name):
    global wp_seq
    wp_seq = 0
    global curr_polygon
    control_polygon = PolygonStamped()
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

    polygon_pub = rospy.Publisher(
    "/car_4/purepursuit_control/visualize".format(car_name),
    PolygonStamped,
    queue_size=1,
)
    polygon_pub.publish(control_polygon)

def callback(data,pose_x, pose_y, target_x, target_y, odom_x, odom_y, car_name, steering_angle, heading, extended_data, angle_min, angle_increment):
    publish_polygon_data(pose_x, pose_y, target_x, target_y, odom_x, odom_y, frame_id, car_name)
    create_laser_node()
    create_raceline_node()
    create_goal_node()
    publish_steer_marker_data(data, steering_angle, heading, odom_x, odom_y, frame_id = "car_4_laser")
    publish_disparity_data(extended_data, angle_min, angle_increment, frame_id="car_4_laser"
)