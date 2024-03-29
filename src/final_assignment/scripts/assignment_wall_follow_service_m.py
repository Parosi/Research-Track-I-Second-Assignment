#! /usr/bin/env python

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This script implements the wall follow algorithm for doing the 
same that the name suggests. This is done via a finite state machine
with the four following states:
0 - find the wall
1 - turn left
2 - follow the wall
3 - stop
It also creates a ros service for receiving the commands to start
and stop the algorithm.
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from final_assignment.srv import AssignmentUserInterface, FollowWallSwitch, FollowWallSwitchResponse

import math

active_ = False
bug0_ = False
# Create a client to assignment user interface service
assignment_user_interface_client = rospy.ServiceProxy('/assignment_user_interface', AssignmentUserInterface)

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'stop'
}


def wall_follower_switch(req):
    global active_, bug0_
    active_ = req.active
    bug0_ = req.bug0
    if active_ == False :
        stop()
    # Create a response
    response = FollowWallSwitchResponse()
    response.result = 'ok'
    # Return the response
    return response
    


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    global state_, state_dict_, bug0_, active_
    global assignment_user_interface_client
    # Control if this node is used by bug0 or not
    if bug0_ == False  and active_ == True:
        # Send a request to the user interface
        resp = assignment_user_interface_client(state, 'wall_follow')
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

# Function that stop the obot when it receives the active equals to false
def stop():
    global pub_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    pub_.publish(msg)

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(1)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.6
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.8
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', FollowWallSwitch, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
