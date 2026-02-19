#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
# home = np.radians([120, -90, 90, -90, -90, 0])
home = np.radians([214, -33.21, 47.11, -103.73, -90.36, 337.86-360])

# Tower A
QA_hover = [214*pi/180.0, -34.34*pi/180.0, 44.72*pi/180.0, -100.12*pi/180.0, -90.38*pi/180.0, (337.88-360)*pi/180.0]
QA_3     = [214*pi/180.0, -31.96*pi/180.0, 48.79*pi/180.0, -106.64*pi/180.0, -90.34*pi/180.0, (-22.15)*pi/180.0]
QA_2     = [214*pi/180.0, -28.67*pi/180.0, 51.12*pi/180.0, -112.19*pi/180.0, -90.30*pi/180.0, (337.83-360)*pi/180.0]
QA_1     = [214*pi/180.0, -24.74*pi/180.0, 52*pi/180.0, -117.01*pi/180.0, -90.27*pi/180.0, (337.82-360)*pi/180.0]

# Tower B
QB_hover = [206*pi/180.0, -49.09*pi/180.0, 72.56*pi/180.0, -113.24*pi/180.0, -90.33*pi/180.0, (329.9-360)*pi/180.0]
QB_3     = [206*pi/180.0, -45.76*pi/180.0, 75.5*pi/180.0, -119.5*pi/180.0, -90.33*pi/180.0, (329.9-360)*pi/180.0]
QB_2     = [206*pi/180.0, -41.78*pi/180.0, 77.36*pi/180.0, -125.35*pi/180.0, -90.33*pi/180.0, (329.9-360)*pi/180.0]
QB_1     = [206*pi/180.0, -36.42*pi/180.0, 78.19*pi/180.0, -131.54*pi/180.0, -90.33*pi/180.0, (329.9-360)*pi/180.0]

# Tower C
QC_hover = [196.94*pi/180.0, -58.66*pi/180.0, 89.11*pi/180.0, -120.25*pi/180.0, -90.41*pi/180.0, (320.64-360)*pi/180.0]
QC_3     = [196.94*pi/180.0, -54.55*pi/180.0, 92.04*pi/180.0, -127.29*pi/180.0, -90.41*pi/180.0, (320.64-360)*pi/180.0]
QC_2     = [196.94*pi/180.0, -49.44*pi/180.0, 93.95*pi/180.0, -134.32*pi/180.0, -90.41*pi/180.0, (320.64-360)*pi/180.0]
QC_1     = [196.94*pi/180.0, -44.46*pi/180.0, 94.63*pi/180.0, -139.98*pi/180.0, -90.41*pi/180.0, (320.64-360)*pi/180.0]

# # Hanoi tower location A
# Q11 = [214*pi/180.0, -31.96*pi/180.0, 48.79*pi/180.0, -106.64*pi/180.0, -90.34*pi/180.0, (-22.15)*pi/180.0]
# Q12 = [214*pi/180.0, -36.09*pi/180.0, 48.94*pi/180.0, -103.88*pi/180.0, -90.11*pi/180.0, (-22.22)*pi/180.0]
# Q13 = [196.06*pi/180.0, -45.13*pi/180.0, 95.6*pi/180.0, -140.09*pi/180.0, -90.12*pi/180.0, (-40.29)*pi/180.0]

# # Tower A
# # hover
# 214 -34.34 44.72 -100.12 -90.38 337.88
# # 3
# Q11 = [214*pi/180.0, -31.96*pi/180.0, 48.79*pi/180.0, -106.64*pi/180.0, -90.34*pi/180.0, (-22.15)*pi/180.0]
# # 2
# 214 -28.67 51.12 -112.19 -90.30 337.83
# # 1
# 214 -24.74 52 -117.01 -90.27 337.82

# # Tower B
# # hover 
# 206 -49.09 72.56 -113.24 -90.33 329.9
# # 3
# 206 -45.76 75.5 -119.5 -90.33 329.9
# # 2 
# 206 -41.78 77.36 -125.35 -90.33 329.9
# # 1
# 206 -36.42 78.19 -131.54 -90.33 329.9

# # Tower C

# # hover 
# 196.94 -58.66 89.11 -120.25 -90.41 320.64
# # 3
# 196.94 -54.55 92.04 -127.29 -90.41 320.64
# # 2 
# 196.94 -49.44 93.95 -134.32 -90.41 320.64
# # 1
# 196.94 -44.46 94.63 -139.98 -90.41 320.64



# Hanoi tower location A
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

# home = np.radians([214, -33.21, 47.11, -103.73, 90.36, 337.86 - 360])



thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
# Tower A
Tower_A = [QA_1, QA_2, QA_3, QA_hover]

#Tower B
Tower_B = [QB_1, QB_2, QB_3, QB_hover]

#Tower C
Tower_C = [QC_1, QC_2, QC_3, QC_hover]

Q = [Tower_A, Tower_B, Tower_C] 

# Q = [ [Q11, Q12, Q13], \
#       [Q11, Q12, Q13], \
#       [Q11, Q12, Q13] ]
############### Your Code End Here ###############



############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
# callback function for suciton state
def gripper_input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN # 1 for successful fuction, 0 for not



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global digital_in_0
    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0

    # hover over start tower 
    move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0, 4.0)

    # go down to pick up block 

    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)

    # suction go brr
    gripper(pub_cmd, loop_rate, suction_on)
    # we should add a wait here
    time.sleep(0.5) 

    # oops i think we missed the block
    if digital_in_0 == 0:
        rospy.loginfo("i cant pick it up :( )")
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        return error

    # if we got it we go up and hover over the tower we just picked up a block from
    move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0, 4.0)

    # now we move to the tower we want to place the block at 
    move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)


    # and now we want to place the block down 
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)

    # release the suction so the block gets placed (added a wait so it like doesnt rush)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5) 

    # now we want to go back up and hover and steer clear of any blocks 
    move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)


    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
    start_tower = 0
    goal_tower = 0

    while(not input_done):

        # user input string 
        start_string = input("Enter Start Tower <0 for A, 1 for B, 2 for C>: ")

        goal_string = input("Enter Goal Tower <0 for A, 1 for B, 2 for C>: ")

        start_tower = int(start_string)
        goal_tower = int(goal_string)

        if start_tower in [0,1,2] and goal_tower in [0,1,2] and start_tower != goal_tower:
            input_done = 1
            loop_count = 1
        else:
            print("Invalid Input")

        # input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
        # print("You entered " + input_string + "\n")

        # if(int(input_string) == 1):
        #     input_done = 1
        #     loop_count = 1
        # elif (int(input_string) == 2):
        #     input_done = 1
        #     loop_count = 2
        # elif (int(input_string) == 3):
        #     input_done = 1
        #     loop_count = 3
        # elif (int(input_string) == 0):
        #     print("Quitting... ")
        #     sys.exit()
        # else:
        #     print("Please just enter the character 1 2 3 or 0 to quit \n\n")
        input_done = 1
        loop_count = 1





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    while(loop_count > 0):

        move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        S = start_tower # start tower
        E = goal_tower # end tower
        M = 3 - S - E
        # we need to determine the middle tower too 


        # now we determine the 7 steps 
        rospy.loginfo("Sending move 1 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=S, start_height=2, end_loc=E, end_height=0):
            return

        # rospy.loginfo("Sending move 2 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=S, start_height=1, end_loc=M, end_height=0):
            return

        # rospy.loginfo("Sending move 3 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=E, start_height=0, end_loc=M, end_height=1):
            return

        # rospy.loginfo("Sending move 4 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=S, start_height=0, end_loc=E, end_height=0):
            return

        # rospy.loginfo("Sending move 5 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=M, start_height=1, end_loc=S, end_height=0):
            return

        # rospy.loginfo("Sending move 6 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=M, start_height=0, end_loc=E, end_height=1):
            return

        # rospy.loginfo("Sending move 7 ...")
        if move_block(pub_cmd=pub_command, loop_rate=loop_rate, start_loc=S, start_height=0, end_loc=E, end_height=2):
            return
        

        loop_count = loop_count - 1

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
