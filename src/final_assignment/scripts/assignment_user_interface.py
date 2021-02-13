#! /usr/bin/env python

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This script implements a user interface to control a robot in a 3d enviroment.

In paricular there are eight commands:
-manualPos (User inserts manually one of the six fixed positions) reach_mode_ = 1
-manualRandom (User inserts manually a random position) reach_mode_ = 2
-autoPos (User wants one of the six fixed positions randomly selected) reach_mode_ = 3
-autoRandom (User wants a random position randomly selected) reach_mode_ = 4
-stop (Stop the robot in the last position)
-change-algorithm (Change the algorithm)
-move-base (Select dijkstra's algorithm) algorithm_ = 1
-bug0 (Select bug0 algorithm) algorithm_ = 2
-wall-follow (Select wall_follow algorithm) algorithm_ = 3

It also creates a ros service for receiving the information about in 
which state are the bug0 and wall follow algorithm (only if they are active)
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import math
import sys
import select
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from final_assignment.srv import RobotRandomPos, FollowWallSwitch, AssignmentUserInterface, AssignmentUserInterfaceResponse
from tf import transformations

# Create the publisher on topic /move_base/goal
goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 10)

# Create a client to robot random pos service
robot_random_pos_client = rospy.ServiceProxy('/robot_random_pos', RobotRandomPos)

# Create a client to wall follower switch service
wall_follower_client = rospy.ServiceProxy('/wall_follower_switch', FollowWallSwitch)

# Create client to bug0 switch service
bug0_client = rospy.ServiceProxy('bug0_switch', SetBool)

# Create a variable for a subscriber on topic /odom
odom_sub = None
		
# Create a variable for a subscriber on topic /scan	
laser_sub = None

# Create a variable for a subscriber on topic /move_base/status	
move_base_status_sub = None

# Create the variable for the counter of odom messages, because we want to print 
# an update only every 2000 messages so every two seconds sharp
odom_count_ = 0

# Create the variable for the counter of scan messages, because we want to print 
# an update only every 76 messages so every two seconds sharp
scan_count_ = 0

# Create the variable for the counter of move_base messages, because we want to print 
# an update only every 10 messages so every two seconds sharp
move_base_count_ = 0

# Create the variable for the counter of bug0 messages, because we want to print 
# an update only every 50 messages so every two seconds sharp
bug0_count_ = 0

# Create the variable for the counter ofwall follow messages, because we want to print 
# an update only every 75 messages so every two seconds sharp
wall_follow_count_ = 0

# Create a variable to activate the log of the odom
active_odom_ = False

# Create a variable to activate the log of the distance from obstacle
active_distance_from_obstacle_ = False

# Create a variable to activate the log of the move_base
active_move_base_ = False

# Create the previous odom variables to check if the new one is equal
previous_odom_x_ = 0
previous_odom_y_ = 0
previous_odom_yaw_ = 0

# Create the previous move_base state to check if the last state was target reached
previous_move_base_state_ = 1

# Create the previous bug0 state to check if the last state was target reached
previous_bug0_state_ = 1

# Create current target variables
target_x_ = 0
target_y_ = 0


# Create variable to store the chosen algorithm
algorithm_ = 0

# Create variable to store the chosen reach mode
reach_mode_ = 0

# Create the previous scan regions variable to check if the new ones are equal
previous_regions_ = {
		'right':  0,
		'fright': 0,
		'front':  0,
		'fleft':  0,
		'left':   0,
	}

# Callback function that prints the current position of the robot
def print_odom(odometry) :
	global odom_count_, previous_odom_x_, previous_odom_y_, previous_odom_yaw_, active_odom_
	# Control if this log is active
	if active_odom_ == True :
		# Convert the quaternary to yaw
		quaternion_ = (
    	 		odometry.pose.pose.orientation.x, 
       			odometry.pose.pose.orientation.y, 
        	 	odometry.pose.pose.orientation.z, 
        		odometry.pose.pose.orientation.w
        	)
		euler_ = transformations.euler_from_quaternion(quaternion_)
		yaw_ = euler_[2]
    
		# Control if the odometry is different than the previous one
		if odometry.pose.pose.position.x != previous_odom_x_ or  \
	     		odometry.pose.pose.position.y != previous_odom_y_ or yaw_ != previous_odom_yaw_ :
			# Control if the odom counter has reached the max value
			if odom_count_ == 2000 :
				# Print the position 
				print 'ROBOT POSITION:\nx =', "{:.3f}".format(odometry.pose.pose.position.x), \
	         			'y =', "{:.3f}".format(odometry.pose.pose.position.y), 'yaw =', "{:.3f}".format(yaw_)
				sys.stdout.flush()
				# Reset the counter
				odom_count_ = 1
			else :
				# Increase odom counter
				odom_count_ += 1
		
# Callback function that prints the current distance of the obstacles
def print_obstacle_distance(scan) :
	global scan_count_, previous_regions_, active_distance_from_obstacle_
	# Control if this log is active
	if active_distance_from_obstacle_ == True :
		# Convert scan to regions
		regions_ = {
			'right':  min(min(scan.ranges[0:143]), 10),
			'fright': min(min(scan.ranges[144:287]), 10),
			'front':  min(min(scan.ranges[288:431]), 10),
			'fleft':  min(min(scan.ranges[432:575]), 10),
			'left':   min(min(scan.ranges[576:719]), 10),
		}
		# Control if the regions are different than the previous ones
		if previous_regions_['right'] != regions_['right'] or previous_regions_['fright'] != regions_['fright'] or \
 				previous_regions_['front'] != regions_['front'] or previous_regions_['fleft'] != regions_['fleft'] or \
   		 		previous_regions_['left'] != regions_['left'] :
			# Control if the scan counter has reached the max value
			if scan_count_ == 76 :
				# Print the distance from the obstacle
				print 'DISTANCE FROM OBSTACLES:\nright =', "{:.3f}".format(regions_['right']), 'front-right =', \
          				"{:.3f}".format(regions_['fright']), 'front =', "{:.3f}".format(regions_['front']), 'front-left =', \
          				"{:.3f}".format(regions_['fleft']), 'left =', "{:.3f}".format(regions_['left'])
				sys.stdout.flush()
				# Reset the counter
				scan_count_ = 1
			else :
				# Increase scan counter
				scan_count_ += 1


# Function that control the gui for change algorithm
def select_algorithm() :
	global algorithm_, wall_follower_client, active_distance_from_obstacle_, active_odom_
	# Define variable for exiting from the while loop
	condition_ = False

	# Extract a random number between 0 and 5 and if the position extracted 
	# is the same as the current position of the robot, extract a new number
	while condition_ == False :
		print('\nInsert the number of an algorithm among the following: \n1.move-base  2.bug0  3.wall-follow')
		sys.stdout.flush()
		new_alg_ = raw_input('')
	
		# Control the inserted number
		if new_alg_ == '1' :
			algorithm_ = 1
			condition_ = True
			select_command()
		elif new_alg_ == '2' :
			algorithm_ = 2
  			condition_ = True
			select_command()
		elif new_alg_ == '3' :
			algorithm_ = 3
			condition_ = True
			# Send a request to the wall follow service
			resp = wall_follower_client(True, False)
			print('WALL FOLLOW ALGORITHM STARTED')
			sys.stdout.flush()
			active_distance_from_obstacle_ = True
			active_odom_ = True
			condition_ = False
			print("Press q to stop the follow wall algorithm")
			sys.stdout.flush()
			# Print option to stop the algorithm
			while condition_ == False :	
				response_ = raw_input('')
				# Control the inserted response
				if response_ == "q" :
					condition_ = True
					stop_robot()
		else :
			print("Invalid number!")
			sys.stdout.flush()
   
   
# Function that control the gui for change mode   
def select_command() :
	global algorithm_, reach_mode_
	# Define variable for exiting from the while loop
	condition_ = False
	# Control in which algorithm we are
	if algorithm_ == 1 or algorithm_ == 2 :
		while condition_ == False :
			print("\nInsert the number of a command among the following: \n1.manualPos  " + 
         			"2.manualRandom  3.autoPos  4.autoRandom  5.stop  6.change-algorithm")
			sys.stdout.flush()
			new_command_ = raw_input('')
	
			# Control the inserted number
			if new_command_ == '1' :
				reach_mode_ = 1
				condition_ = True
				request_target()
			elif new_command_ == '2' :
				reach_mode_ = 2
				condition_ = True
				request_target()
			elif new_command_ == '3' :
				reach_mode_ = 3
				condition_ = True
				request_target()
			elif new_command_ == '4' :
				reach_mode_ = 4
				condition_ = True
				request_target()
			elif new_command_ == '5' :
				stop_robot()
				condition_ = True
			elif new_command_ == '6' :
				select_algorithm()
				condition_ = True
			else :
				print("Invalid number!")
				sys.stdout.flush()


# Function that stop the robot in the last position
def stop_robot() :
	global algorithm_, wall_follower_client, active_distance_from_obstacle_, active_odom_, \
		reach_mode_, previous_move_base_state_
	# Control in which algorithm we are
	if (algorithm_ == 1 or algorithm_ == 2) and (reach_mode_ == 3 or reach_mode_ == 4) :
		# Control if the robot reached the target
		if (previous_move_base_state_ == 3 and algorithm_ == 1) or (previous_bug0_state_ == 2 and algorithm_ == 2) :
			print('\nRobot stopped in the last target position')
			sys.stdout.flush()
			select_command()
		# Control if the robot aborted the target
		elif (previous_move_base_state_ == 4 and algorithm_ == 1) or (previous_bug0_state_ == 3 and algorithm_ == 2) :
			print('\nRobot stopped')
			sys.stdout.flush()
			select_command()
	elif algorithm_ == 1 or algorithm_ == 2 :
		print("\nRobot is already stopped")
		sys.stdout.flush()
		select_command()
	elif algorithm_ == 3 :
		# Send a request to the wall follow service
		resp = wall_follower_client(False, False)
		active_distance_from_obstacle_ = False
		active_odom_ = False
		print("\nRobot stopped")
		sys.stdout.flush()
  		print('Follow Wall stopped')
		sys.stdout.flush()
		select_algorithm()
 

# Callback function that prints the current state of move_base algorithm
def print_move_base_state(goalStatus) :
	global move_base_status_sub, move_base_count_, target_x_, target_y_, algorithm_, \
     		active_distance_from_obstacle_, active_move_base_, active_odom_, \
			previous_move_base_state_, reach_mode_
	# Control if the current algorithm is move_base
	if algorithm_ == 1 and active_move_base_ == True:
		# Control if the scan counter has reached the max value
		if move_base_count_ == 10 :
			# Control in which status is the move_base algorithm
			# status = 1 active       
			# status = 3 target achieved
			# status = 4 impossible to find a valid plan
			if goalStatus.status_list[0].status == 1 :
				# Print the status
				print'ROBOT STATUS: Move_Base - Reaching target x:', target_x_, 'y:', target_y_
				sys.stdout.flush()
				previous_move_base_state_ = 1
				# Reset the counter
				move_base_count_ = 1
			elif goalStatus.status_list[0].status == 3 :
				# Control if the the laste state is 3, this because when a target is reached the last 
				# remains in the move_base status for some messages
				if previous_move_base_state_ == 3:
					# Print the status
					print'ROBOT STATUS: Move_Base - Reaching target x:', target_x_, 'y:', target_y_
					sys.stdout.flush()
					# Reset the counter
					move_base_count_ = 1
				else:
					# Print the status
					print'ROBOT STATUS: Move_Base - Achieved target x:', target_x_, 'y:', target_y_
					sys.stdout.flush()
					# Reset the counter
					move_base_count_ = 1
					previous_move_base_state_ = 3
					active_distance_from_obstacle_ = False
					active_odom_ = False
					active_move_base_ = False
					if (reach_mode_ == 3 or reach_mode_ == 4) :
						# Ask if the user wants to stop the auto mode
						print('\nDo you want to stop the auto mode? (Y/n)')
						sys.stdout.flush()
						# User has 20 seconds to answer
						i, o, e = select.select( [sys.stdin], [], [], 20 )
						if i and sys.stdin.readline().strip() == 'Y' :
							stop_robot()
						else :
							request_target()
					else :
						select_command()
			elif goalStatus.status_list[0].status == 4 :	
				# Control if the the laste state is 4, this because when a target is canceled the last 
				# remains in the move_base status for some messages
				if previous_move_base_state_ == 4:
					# Print the status
					print'ROBOT STATUS: Move_Base - Reaching target x:', target_x_, 'y:', target_y_
					sys.stdout.flush()
					# Reset the counter
					move_base_count_ = 1
				else:
					# Print the status
					print'ROBOT STATUS: Move_Base - Aborted target x:', target_x_, 'y:', target_y_
					sys.stdout.flush()
					# Reset the counter
					move_base_count_ = 1
					previous_move_base_state_ = 4
					active_distance_from_obstacle_ = False
					active_odom_ = False
					active_move_base_ = False
					if (reach_mode_ == 3 or reach_mode_ == 4) :
						# Ask if the user wants to stop the auto mode
						print('\nDo you want to stop the auto mode? (Y/n)')
						sys.stdout.flush()
						# User has 20 seconds to answer
						i, o, e = select.select( [sys.stdin], [], [], 20 )
						if i and sys.stdin.readline().strip() == 'Y' :
							stop_robot()
						else :
							request_target()
					else :
						select_command()	
		else :
			# Increase move_base counter
			move_base_count_ += 1

# Callback function that prints the current state of the bug0 and wall follow algorithm
def print_bug0Wall_state(assignmentUserInterface) :
	global target_x_, target_y_, algorithm_, active_distance_from_obstacle_, active_odom_, \
		reach_mode_, previous_bug0_state_, bug0_count_, wall_follow_count_, bug0_client
	# Create a response
	response = AssignmentUserInterfaceResponse()
	# Control which is the current algorithm
	# Bug0
	if algorithm_ == 2 :
		# Control in which status is the bug0 algorithm, becuase these two states arrive only once
		# status = 2 target achieved
		# status = 3 timeout expired
		if assignmentUserInterface.mode == 2 and assignmentUserInterface.node == 'bug0' :
			# Print the status
			print'ROBOT STATUS: Bug0 - Achieved target x:', target_x_, 'y:', target_y_
			sys.stdout.flush()
			previous_bug0_state_ = 2
			bug0_count_ = 1
			active_distance_from_obstacle_ = False
			active_odom_ = False
			resp = bug0_client(False)
			if (reach_mode_ == 3 or reach_mode_ == 4) :
				# Ask if the user wants to stop the auto mode
				print('\nDo you want to stop the auto mode? (Y/n)')
				sys.stdout.flush()
				# User has 20 seconds to answer
				i, o, e = select.select( [sys.stdin], [], [], 20 )
				if i and sys.stdin.readline().strip() == 'Y' :
					stop_robot()
				else :
					request_target()
			else :
				select_command()
		elif assignmentUserInterface.mode == 3 and assignmentUserInterface.node == 'bug0' :
			# Print the status
			print'ROBOT STATUS: Bug0 - Timeout expired, Target x:', target_x_, 'y:', target_y_
			sys.stdout.flush()
			previous_bug0_state_ = 3
			bug0_count_ = 1
			active_distance_from_obstacle_ = False
			active_odom_ = False
			resp = bug0_client(False)
			if (reach_mode_ == 3 or reach_mode_ == 4) :
				# Ask if the user wants to stop the auto mode
				print('\nDo you want to stop the auto mode? (Y/n)')
				sys.stdout.flush()
				# User has 20 seconds to answer
				i, o, e = select.select( [sys.stdin], [], [], 20 )
				if i and sys.stdin.readline().strip() == 'Y' :
					stop_robot()
				else :
					request_target()
			else :
				select_command()
		# Control if the scan counter has reached the max value
		elif bug0_count_ == 50 :
			# Control in which status is the bug0 algorithm
			# status = 0 go to point
			# status = 1 follow wall
			if assignmentUserInterface.mode == 0 and assignmentUserInterface.node == 'bug0' :
				# Print the status
				print'ROBOT STATUS: Bug0 - Going to point, Target x:', target_x_, 'y:', target_y_
				sys.stdout.flush()
				previous_bug0_state_ = 0
				bug0_count_ = 1
			elif assignmentUserInterface.mode == 1 and assignmentUserInterface.node == 'bug0' :
				# Print the status
				print'ROBOT STATUS: Bug0 - Following wall, Target x:', target_x_, 'y:', target_y_
				sys.stdout.flush()
				previous_bug0_state_ = 1
				bug0_count_ = 1			
		else :
			# Increase move_base counter
			bug0_count_ += 1
 
	# Follow Wall
	elif algorithm_ == 3 :
		# Control if the scan counter has reached the max value
		if wall_follow_count_ == 75 :
			# Control in which status is the bug0 algorithm
			# status = 0 find the wall
			# status = 1 turn left
			# status = 2 follow the wall
			if assignmentUserInterface.mode == 0 and assignmentUserInterface.node == 'wall_follow' :
				wall_follow_count_ = 1
				# Print the status
				print('ROBOT STATUS: Follow Wall - Finding wall')
				sys.stdout.flush()
			elif assignmentUserInterface.mode == 1 and assignmentUserInterface.node == 'wall_follow' :
				wall_follow_count_ = 1
				# Print the status
				print('ROBOT STATUS: Follow Wall - Turning left')
				sys.stdout.flush()
			elif assignmentUserInterface.mode == 2 and assignmentUserInterface.node == 'wall_follow' :
				wall_follow_count_ = 1
				# Print the status
				print('ROBOT STATUS: Follow Wall - Following wall')
				sys.stdout.flush()
		else :
			# Increase move_base counter
			wall_follow_count_ += 1
	# Send a response
	response.result = 'ok'
	# Return the response
	return response

# Function that request a new target to robot random pos server  
def request_target() :
	global reach_mode_, algorithm_, robot_random_pos_client, target_x_, target_y_
	# Define variable for exiting from the while loop
	condition_ = False
	# Control which is the current reach mode
	if reach_mode_ == 1 :
		while condition_ == False :	
			print("Insert one of the following six positions: \n(-4,-3)  (-4,2)  (-4,7)  (5,-7)  (5,-3)  (5,1)")
			sys.stdout.flush()
			x_ = float(raw_input('x: '))
			y_ = float(raw_input('y: '))
			# Send a request to robot random pos server
			resp = robot_random_pos_client('manualPos', x_, y_)
			# Control the response
			if resp.result == 'ok' :
				target_x_ = x_
				target_y_ = y_
				condition_ = True
				send_new_target(x_, y_)
			elif resp.result == 'error1' :
				print('The inserted position is not one of the six positions')
				sys.stdout.flush()
	elif reach_mode_ == 2 :
		while condition_ == False :	
			print("Insert one random position in the interval x:[-5.9,5.9] y:[-8.3, 8.8]:")
			sys.stdout.flush()
			x_ = float(raw_input('x: '))
			y_ = float(raw_input('y: '))
			# Send a request to robot random pos server
			resp = robot_random_pos_client('manualRandom', x_, y_)
			# Control the response
			if resp.result == 'ok' :
				target_x_ = x_
				target_y_ = y_
				condition_ = True
				send_new_target(x_, y_)
			elif resp.result == 'error2' :
				print('The inserted position is not inside the house, try with a new position')
				sys.stdout.flush()
			elif resp.result == 'error3' :
				print('The inserted y is not in the required interval, try with a new position')
				sys.stdout.flush()
			elif resp.result == 'error4' :
				print('The inserted x is not in the required interval, try with a new position')
				sys.stdout.flush()
	elif reach_mode_ == 3 :
		# Send a request to robot random pos server
		resp = robot_random_pos_client('autoPos', target_x_, target_y_)
		# Control the response
		if resp.result == 'ok' :
			target_x_ = resp.result_x
			target_y_ = resp.result_y
			print'\nRANDOM POSITION: x:', resp.result_x, 'y:', resp.result_y
			sys.stdout.flush()
			send_new_target(resp.result_x, resp.result_y)
	elif reach_mode_ == 4 :
		# Send a request to robot random pos server
		resp = robot_random_pos_client('autoRandom', target_x_, target_y_)
		# Control the response
		if resp.result == 'ok' :
			target_x_ = resp.result_x
			target_y_ = resp.result_y
			print'\nRANDOM POSITION: x:', resp.result_x, 'y:', resp.result_y
			sys.stdout.flush()
			send_new_target(resp.result_x, resp.result_y)
		


# Function that sends a new target to move_base or bug0
def send_new_target(x, y) :
	global algorithm_, goal_pub, bug0_client, active_distance_from_obstacle_, \
		active_move_base_, active_odom_
 
	# Control which is the current algorithm
	if algorithm_ == 1 :
		# Initialize the message
		goal_ = MoveBaseActionGoal()
		goal_.goal.target_pose.header.frame_id = 'map'
		goal_.goal.target_pose.pose.orientation.w = 1
		goal_.goal.target_pose.pose.position.x = x
		goal_.goal.target_pose.pose.position.y = y
		goal_pub.publish(goal_)
		print('\nTARGET POSITON SET')
		sys.stdout.flush()
		print('ROBOT STATUS: Move_Base - Start moving\n')
		sys.stdout.flush()
		active_distance_from_obstacle_ = True
		active_move_base_ = True
		active_odom_ = True
  
	elif algorithm_ == 2 :
		rospy.set_param("des_pos_x", x)
  		rospy.set_param("des_pos_y", y)
		print('\nTARGET POSITON SET')
		sys.stdout.flush()
		resp = bug0_client(True)
		print('ROBOT STATUS: Bug0 - Start moving\n')
		sys.stdout.flush()
		active_distance_from_obstacle_ = True
		active_odom_ = True


def main():
	global odom_sub, laser_sub, move_base_status_sub

	rospy.init_node('assignment_user_interface')

	# Create the service
	rospy.Service('assignment_user_interface', AssignmentUserInterface, print_bug0Wall_state)
 	# Create a subscriber on topic /odom
	odom_sub = rospy.Subscriber('/odom', Odometry, print_odom)
	# Create a subscriber on topic /scan	
	laser_sub = rospy.Subscriber('/scan', LaserScan, print_obstacle_distance)
	# Create a subscriber on topic /move_base/status	
	move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, print_move_base_state)
	# Start to interact with the user
	select_algorithm()

	rospy.spin()


if __name__ == '__main__':
    main()
