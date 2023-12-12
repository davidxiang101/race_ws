#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive


class QualifyControl:
    def __init__(self):
        # Initialize node
        rospy.init_node("qualify_control", anonymous=True)

        # Subscribers to listen to the gap finder and pure pursuit nodes
        rospy.Subscriber(
            "/gap_finder/command", AckermannDrive, self.gap_finder_callback
        )
        rospy.Subscriber(
            "/pure_pursuit/command", AckermannDrive, self.pure_pursuit_callback
        )

        # Publisher to send final command to the car
        self.command_pub = rospy.Publisher(
            "/car_4/offboard/command", AckermannDrive, queue_size=1
        )

        # Variables to store the latest data from both algorithms
        self.gap_finder_data = None
        self.pure_pursuit_data = None

        # Additional logic variables
        self.use_gap_finder = False  # Logic to decide which algorithm to use

    def gap_finder_callback(self, data):
        self.gap_finder_data = data
        self.decide_and_publish()

    def pure_pursuit_callback(self, data):
        self.pure_pursuit_data = data
        self.decide_and_publish()

    def decide_and_publish(self):
        # Logic to decide which algorithm to use
        if self.use_gap_finder and self.gap_finder_data is not None:
            self.command_pub.publish(self.gap_finder_data)
        elif self.pure_pursuit_data is not None:
            self.command_pub.publish(self.pure_pursuit_data)

        # Reset data after use
        self.gap_finder_data = None
        self.pure_pursuit_data = None

    def run(self):
        # Keep node running
        rospy.spin()


if __name__ == "__main__":
    node = QualifyControl()
    node.run()
