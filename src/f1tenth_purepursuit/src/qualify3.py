#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive


# Variables to store the latest data from both algorithms
gap_finder_data = None
pure_pursuit_data = None

# Additional logic variables
use_gap_finder = False  # Logic to decide which algorithm to use


def gap_finder_callback(data):
    global use_gap_finder
    global gap_finder_data
    gap_finder_data = data
    if gap_finder_data is not None and gap_finder_data.speed < 0:
        use_gap_finder = True
        gap_finder_data.speed = gap_finder_data.speed * -1.0
    else:
        use_gap_finder = False
    decide_and_publish()

def pure_pursuit_callback(data):
    global pure_pursuit_data
    pure_pursuit_data = data
    # print("pursue")
    decide_and_publish()


def decide_and_publish():
    global gap_finder_data
    global pure_pursuit_data
    global use_gap_finder
    
    # Use gap finder data if an obstacle is detected
    if use_gap_finder:
        command_pub.publish(gap_finder_data)
        print("gap finder")
    # Otherwise, use pure pursuit data
    elif pure_pursuit_data is not None:
        print("pure pursuit")
        command_pub.publish(pure_pursuit_data)
    # Reset data after use
    # print("NONE")



if __name__ == "__main__":
    rospy.init_node("qualify_control", anonymous=True)
    # print(";alskjdfa;lsdkjfas;lkfdja;slkdfj")

    # Subscribers to listen to the gap finder and pure pursuit nodes
    rospy.Subscriber(
        "/gap_finder/command", AckermannDrive, gap_finder_callback
    )
    rospy.Subscriber(
        "/pure_pursuit/command", AckermannDrive, pure_pursuit_callback
    )

    # Publisher to send final command to the car
    command_pub = rospy.Publisher(
        "/car_4/offboard/command", AckermannDrive, queue_size=1
    )
    
    rospy.spin()
