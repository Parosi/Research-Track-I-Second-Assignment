#!/usr/bin/env python

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This script implements a server for the service RobotRandomPos.
In fact in its main function, service(), it runs a server. 
Latter's callback, service_callback, controls the mode and provides a response.
The function controlRegion permits to control if a random point is inside of 
one of the seven regions ( these can be visualized in the png image Regions.png).

In paricular there are four modes:
-manualPos (User inserts manually one of the six fixed positions)
-manualRandom (User inserts manually a random position)
-autoPos (User wants one of the six fixed positions randomly selected)
-autoRandom (User wants a random position randomly selected)
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import random
from final_assignment.srv import RobotRandomPos, RobotRandomPosResponse

# Callback function called when there is a new request
def service_callback(req):
	# Define the tuble of x fixed values
	x_values_ = (-4, -4, -4, 5, 5, 5)

	# Define the tuble of x fixed values
	y_values_ = (-3, 2, 7, -7, -3, 1)
 
	# Create a response
	response = RobotRandomPosResponse()

	# Control the mode
	if req.mode == "manualPos" :
		# Control if the poision is one of the possible six
		if req.user_x == -4 and (req.user_y == -3 or req.user_y == 2 or req.user_y == 7) :
			response.result = "ok"
			response.result_x = req.user_x
			response.result_y = req.user_y
		elif req.user_x == 5 and (req.user_y == -3 or req.user_y == 1 or req.user_y == -7) :
			response.result = "ok"
			response.result_x = req.user_x
			response.result_y = req.user_y
		else :
			response.result = "error1"
			response.result_x = req.user_x
			response.result_y = req.user_y
		
	elif req.mode == "manualRandom" :
		# Control if the x inserted is feasible
		if req.user_x <= 6.1 and req.user_x >= -6.1 :
			# Control if y inserted is feasible
			if req.user_y <= 9 and req.user_y >= -8.5 :
				# Control if the position inserted is feasible
				feasible_ = controlRegion(req.user_x, req.user_y)
				if feasible_ == True :
					response.result = "ok"
					response.result_x = req.user_x
					response.result_y = req.user_y					
				else :
					response.result = "error2"
					response.result_x = req.user_x
					response.result_y = req.user_y
			else :
				response.result = "error3"
				response.result_x = req.user_x
				response.result_y = req.user_y
		else :
			response.result = "error4"
			response.result_x = req.user_x
			response.result_y = req.user_y

	elif req.mode == "autoPos" :
		# Define variable for exiting from the while loop
		condition_ = False

		# Extract a random number between 0 and 5 and if the position extracted 
		# is the same as the current position of the robot, extract a new number
		while (condition_ == False) :
			n_ = random.randint(0, 5)
			if req.user_x != x_values_[n_] or req.user_y != y_values_[n_] :
				condition_ = True
				response.result = "ok"
				response.result_x = x_values_[n_]
				response.result_y = y_values_[n_]
		
	elif req.mode == "autoRandom" :
		# Define variable for exiting from the while loop
		condition_ = False

		# Extract a random number between 0 and 5 and if the position extracted 
		# is the same as the current position of the robot, extract a new number
		while (condition_ == False) :
			x_ = random.uniform(-5.9, 5.9)
			y_ = random.uniform(-8.3, 8.8)
			# Control if the position inserted is feasible
			feasible_ = controlRegion(x_, y_)
			if feasible_ == True and (req.user_x != x_ or req.user_y != y_)	:
				condition_ = True
				response.result = "ok"
				response.result_x = x_
				response.result_y = y_
	else :
		response.result = "error0"
		response.result_x = 0
		response.result_y = 0

	# Return the response
	return response


# Main function of the node
def service():
	# Initialize the node
	rospy.init_node('robot_random_pos_server')
	# Create the service
	rospy.Service('robot_random_pos', RobotRandomPos, service_callback)
	# Check continously if there are requests
	rospy.spin()

# Function that controls if a random point is inside one of the seven regions
def controlRegion(x, y) :
	# Define return variable
	result_ = False

	# Control if the goal is in region 1
	if x >= -0.3 and y <= -6.2 and y >= -8.3 :
		result_ = True
	# Control if the goal is in region 2
	elif x <= 2.3 and y <= -0.1 and y >= -5.5 :
		result_ = True
	# Control if the goal is in region 3
	elif x >= 3 and y <= -1.8 and y >= -5.5 :
		result_ = True
	# Control if the goal is in region 4
	elif x >= 3 and y <= 2.8 and y >= -1.5 :
		result_ = True
	# Control if the goal is in region 5
	elif x <= -2.2 and y <= 2.8 and y >= 0.6 :
		result_ = True
	# Control if the goal is in region 6
	elif x <= 0.3 and x >= -1.9 and y >= 0.3 :
		result_ = True
	# Control if the goal is in region 7
	elif x <= -1.9 and y <= 8.8 and y >= 3.1 :
		result_ = True

	# Return the result 
	return result_


# Be sure that service function executes only if we execute
# the script and not when we import it
if __name__ == '__main__':
	try:
		service()
	except rospy.ROSInterruptException:
		pass
