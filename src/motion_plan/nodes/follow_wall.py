#! /usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
velocity_msg = Twist()
follow_dir = -1

section = {
    'front': 0,
    'left': 0,
    'right': 0,
}

state_ = 0

state_dict_ = {
    0: 'Find wall',
    1: 'Turn right',
    2: 'Follow the wall',
    3: 'Turn left',
    4: 'Diagonally right',
    5: 'Diagonally left',
}
'''
This block contains all the functions designed to execute the navigation process.

Function: Change_state: This function gets information about the state of robot based on the distance from
                        from obstacle and changes the state of robot.

Argument: state:{Type: Integer} This parameter is used to get the required action set for thr robot.
                0 - 'Find wall',
                1 - 'turn right',
                2 - 'Follow the wall',
                3 - 'turn left',
                4 - 'diagonally right',
                5 - 'diagonally left',
                
'''


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('State of Bot - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


'''
Function: callback_laser: This function gets information about LaserScan Data from the sensor and has the conditions set
                          to change the state of robot based on the distance from the obstacle.
                          
                          LaserScan data is transferred in the numpy array to retrieve it more efficiently.
                          
                          I have used numpy min() inbuilt function to get LaserScan values between 0-40 degrees for left
                          side, 70-110 for centre/front side, and 140-180 degrees for right. 
                          
                          Once the callback_laser is called in subscriber it will initiate the bug_action block.
                          
Argument: msg: {Type: ranges}: This parameter gets sensor_msgs for LaserScan.
'''


def callback_laser(msg):
    global section

    laser_range = np.array(msg.ranges)
    section = {
        'front': min(laser_range[70:110]),
        'left': min(laser_range[0:40]),
        'right': min(laser_range[140:180]),
    }

    bug_action()


'''
Function: bug_action: This function uses a global function to set the direction or wall to follow. When the variable 
                      follow_dir == -1, the wall follower will reset. When follow_dir == 0, the robot is set to follow 
                      left wall, and when the follow-dir == 1, robot is set to follow the right wall. 
                      
                      Based on the global variable, once a wall is selected the algorithm to follow that particular wall
                      is implemented. 
'''


def bug_action():
    global follow_dir

    b = 1  # maximum threshold distance
    a = 0.5  # minimum threshold distance
    velocity = Twist()  # Odometry call for velocity
    linear_x = 0  # Odometry message for linear velocity will be called here.
    angular_z = 0  # Odometry message for angular velocity will be called here.

    rospy.loginfo("follow_direction {f}".format(f=follow_dir))  # This will indicate the direction of wall to follow.

    if section['front'] > b and section['left'] > b and section['right'] > b:  # Loop 1
        change_state(0)
        rospy.loginfo("Reset Follow_dir")
    elif follow_dir == -1:  # To set the direction of wall to follow
        if section['left'] < b:
            change_state(1)
            follow_dir = 0
            rospy.loginfo("following left wall")
        elif section['right'] < b:
            change_state(3)
            follow_dir = 1
            rospy.loginfo("following right wall")
        else:
            change_state(2)
            rospy.loginfo("follow direction not set")
    elif section['front'] < a and section['left'] < a and section['right'] < a:
        rospy.loginfo("Too Close")
    else:
        rospy.loginfo("Running")

    if follow_dir == 0:  # Algorithm for left wall follower
        if section['left'] > b and section['front'] > a:
            change_state(4)
        elif section['left'] < b and section['front'] > a:
            change_state(2)
        elif section['left'] < b and section['front'] < a:
            change_state(3)
        else:
            rospy.loginfo("follow left wall is not running")
    elif follow_dir == 1:  # Algorithm for right wall follower
        if section['right'] > b and section['front'] > a:
            change_state(5)
        elif section['right'] < b and section['front'] > a:
            change_state(2)
        elif section['right'] < b and section['front'] < a:
            change_state(1)
        else:
            rospy.loginfo("follow right wall is not running")


'''
Function: find_wall: This function publishes linear and angular velocities for finding wall.
'''


def find_wall():
    velocity = Twist()
    velocity.linear.x = 0.3
    velocity.angular.z = 0
    return velocity


'''
Function: turn_left:  This function publishes linear and angular velocities for turning left.
'''


def turn_left():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 0.3
    return velocity


'''
Function: turn_right:  This function publishes linear and angular velocities for turning right.
'''


def turn_right():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = -0.3
    return velocity


'''
Function: move_ahead:  This function publishes linear and angular velocities for moving straight.
'''


def move_ahead():
    velocity = Twist()
    velocity.linear.x = 0.3
    velocity.angular.z = 0
    return velocity


'''
Function: move_diag_right:  This function publishes linear and angular velocities for moving diagonally right.
'''


def move_diag_right():
    velocity = Twist()
    velocity.linear.x = 0.1
    velocity.angular.z = -0.3
    return velocity


'''
Function: move_diag_left:  This function publishes linear and angular velocities for moving diagonally left.
'''


def move_diag_left():
    velocity = Twist()
    velocity.linear.x = 0.1
    velocity.angular.z = 0.3
    return velocity


'''
Function: check: This function publishes velocity values for the logic based on the states.
'''


def check():
    global pub

    rospy.init_node('follow_wall')
    rospy.Subscriber('/scan', LaserScan, callback_laser)

    while not rospy.is_shutdown():

        velocity = Twist()
        if state_ == 0:
            velocity = find_wall()
        elif state_ == 1:
            velocity = turn_right()
        elif state_ == 2:
            velocity = move_ahead()
        elif state_ == 3:
            velocity = turn_left()
        elif state_ == 4:
            velocity = move_diag_right()
        elif state_ == 5:
            velocity = move_diag_left()
        else:
            rospy.logerr('Unknown state!')

        pub.publish(velocity)

    rospy.spin()


if __name__ == "__main__":
    check()
