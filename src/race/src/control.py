#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
import time

# PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
error_sum = 0.0
angle = 0.0
prev_time = time.time()


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 0.0	#default
min_speed = 20
max_speed = 45
min_error = -1.8  # replace with your actual value
max_error = 1.0  # replace with your actual value

# Publisher for moving the car.
# TODO: Use the coorect topic /car_4/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 1)

def control(data):
	global command_pub
	global prev_error
	global vel_input
	global kp
	global kd
	global angle 
	global error_sum
	global prev_time

	if  math.isnan(data.pid_error):
		return

	print("PID Control Node is Listening to error")

	current_time = time.time()
	dt = current_time - prev_time

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	# 1. Calculate the error between the desired and current state
	current_error = data.pid_error
	# Calculate the proportional term
	proportional = kp * current_error
	# 3. Calculate the integral term
	error_sum += current_error * dt
	integral = ki * (error_sum)
	# 4. Calculate the derivative term
	derivative = kd * (current_error - prev_error) / dt
	# 5. Calculate the steering angle
	angle = proportional + integral + derivative
	# 6. Update previous error
	prev_error = current_error
	prev_time = current_time


	# Calculate new speed based on current error
	normalized_error = 1 - ((abs(current_error)) / (max_error))
	new_speed = min_speed + normalized_error * (max_speed - min_speed)
    
    # Clamp speed within specified range
	new_speed = min(max_speed, max(min_speed, new_speed))

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	# check the steering angle is in bounds
	if angle > 100:
		angle = 100
	elif angle < -100:
		angle = -100
	command.steering_angle = angle

	vel_input = new_speed


	# TODO: Make sure the velocity is within bounds [0,100]
	# check the velocity is in bounds
	if vel_input > 50:
		vel_input = 50
	elif vel_input < -50:
		vel_input = -50
	command.speed = vel_input


	# Move the car autonomously
	command_pub.publish(command)
	#print(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
