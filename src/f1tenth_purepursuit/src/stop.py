#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive


# Variables to store the latest data from both algorithms
pure_pursuit_data = None

# Additional logic variables
slowdown = False  # Logic to decide which algorithm to use


def gap_finder_callback(data):
    if data.steering_angle == -1:
        slowdown = True
    else:
        slowdown = False

def pure_pursuit_callback(data):
    global pure_pursuit_data
    pure_pursuit_data = data
    # print("pursue")
    decide_and_publish()


def decide_and_publish():
    global pure_pursuit_data
    global slowdown
    
    # Use gap finder data if an obstacle is detected
    if slowdown:
        pure_pursuit_data.speed = 0
        command_pub.publish(pure_pursuit_data)
        print("slowdown")
    # Otherwise, use pure pursuit data
    elif pure_pursuit_data is not None:
        print("pure pursuit")
        command_pub.publish(pure_pursuit_data)

    print("neither")



if __name__ == "__main__":
    rospy.init_node("stop", anonymous=True)
    print(";alskjdfa;lsdkjfas;lkfdja;slkdfj")

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
