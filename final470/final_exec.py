#!/usr/bin/env python

import sys
import os
import copy
import time
import rospy


import numpy as np
from final_header import *
from final_func import *
from final_helpers import *
import cv2 as cv
from matplotlib import pyplot as plt


################ Pre-defined parameters and functions below (can change if needed) ################

# 20Hz
SPIN_RATE = 20  

# UR3 home location
home = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]  

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)  

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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
    


"""
Function to control the suction cup on/off
"""
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

            rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel, move_type):
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
    driver_msg.move_type = move_type  # Move type (MoveJ or MoveL)
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

################ Pre-defined parameters and functions above (can change if needed) ################

##========= TODO: Helper Functions =========##

def find_keypoints(image):
    """Gets keypoints from the given image that trace different shapes on the image.
    The returned keypoints can be any format you want, as long as they they are aligned with your algorithm when drawing the image. 

    Parameters
    ----------
    image : np.ndarray
        The given image (before or after preprocessing)

    Returns
    -------
    keypoints
        a list of keypoints detected in image coordinates
    """

    img_color = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
    # blurred = cv.GaussianBlur(img, (5, 5), 0)
    canny_img = cv.Canny(image, 100, 200)
    contours, hierarchy = cv.findContours(canny_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    all_contours_keypoints = [] # This will hold lists of coordinates

    keypoints = []
    sample_rate = 2 # Keep every 5th point
    epsilon = 0.5  # Smaller value for less aggressive approximation
    
    for cnt in contours:
        approx = cv.approxPolyDP(cnt, epsilon, closed=False)

        contour_kps = [] # The points for THIS specific shape

        for i in range(0, len(approx), sample_rate):
            x, y = approx[i][0]

            # Store the raw pixel coordinates directly as a tuple
            contour_kps.append((float(x), float(y)))
            keypoints.append(cv.KeyPoint(float(x), float(y), 1))

        # Only add the shape if it has at least 2 points to draw a line
        if len(contour_kps) > 1:
            all_contours_keypoints.append(contour_kps)

    img_with_kp = cv.drawKeypoints(img_color, keypoints, None, color=(0, 255, 0))

    cv.imshow('Keypoints', img_with_kp)

    # plt.subplot(121),plt.imshow(img,cmap = 'gray')
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    
    # plt.show()

    return all_contours_keypoints

HOVER_HEIGHT = 30
DRAWING_HEIGHT = 10

PAPER_X = 125
PAPER_Y = 60

PAPER_X_RANGE = 280
PAPER_Y_RANGE = 215

world_pixel = np.array([320,82])

# Function that converts image coord to world coord
# Width should be the shorter col dimension (the image is already in portrait)
def IMG2W(col, row, image_width, image_height):
    # 5 mm of margin in all directions
    scale = min((PAPER_X_RANGE-10)/image_height, (PAPER_Y_RANGE-10)/image_width)

    x = row*scale + PAPER_X + 5
    y = col*scale + PAPER_Y + 5

    return x, y

def draw_image(world_keypoints, pub_cmd, loop_rate, vel, accel):
    """Draw the image based on detected keypoints in world coordinates. 
    You can use any algorithm you want to draw the image, using the keypoints detected in world coordinates. 

    Parameters
    ----------
    world_keypoints:
        a list of keypoints detected in world coordinates
    """
    YAW = 0.0

    remaining = [list(c) for c in world_keypoints]
    current_pos = remaining[0][0]

    while remaining:
        # Greedy: pick contour (and direction) whose endpoint is closest to current_pos
        best_idx = None
        best_dist = float('inf')
        best_reversed = False

        for i, contour in enumerate(remaining):
            d_start = np.hypot(contour[0][0] - current_pos[0], contour[0][1] - current_pos[1])
            d_end   = np.hypot(contour[-1][0] - current_pos[0], contour[-1][1] - current_pos[1])
            if d_start < best_dist:
                best_dist, best_idx, best_reversed = d_start, i, False
            if d_end < best_dist:
                best_dist, best_idx, best_reversed = d_end, i, True

        contour = remaining.pop(best_idx)
        if best_reversed:
            contour = list(reversed(contour))

        # Hover to first point
        x0, y0 = contour[0]
        dest = lab_invk(x0, y0, HOVER_HEIGHT, YAW)
        move_arm(pub_cmd, loop_rate, dest, vel, accel, 'J')

        # Pen down
        dest = lab_invk(x0, y0, DRAWING_HEIGHT, YAW)
        move_arm(pub_cmd, loop_rate, dest, vel, accel, 'L')

        # Draw contour
        for x, y in contour[1:]:
            dest = lab_invk(x, y, DRAWING_HEIGHT, YAW)
            move_arm(pub_cmd, loop_rate, dest, vel, accel, 'L')

        # Pen up
        x_last, y_last = contour[-1]
        dest = lab_invk(x_last, y_last, HOVER_HEIGHT, YAW)
        move_arm(pub_cmd, loop_rate, dest, vel, accel, 'L')

        current_pos = contour[-1]

##==========================================##

"""
Program run from here
"""
def main():
    global home
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    # Velocity and acceleration of the UR3 arm
    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Move to the home position

    ##========= TODO: Read and draw a given image =========##

    image_choice = input("Select a file to draw: ")

    switcher = {
        "cat": "cat.jpg",
        "corn": "corn.jpeg",
        "zigzag": "zigzag.jpg",
        # "dog": "dog.jpg",
        # "fine": "fine.jpg"
    }
    file = switcher.get(image_choice, "Invalid file choice")
    img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images', file)

    # load image
    img = cv.imread(img_path, cv.IMREAD_GRAYSCALE)
    assert img is not None, "file could not be read, check with os.path.exists()"

    if img.shape[1] > img.shape[0]:  # wider than tall → rotate to portrait
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

    keypoints = find_keypoints(img)

    image_height, image_width = img.shape[:2]
    world_keypoints = [
        [IMG2W(col, row, image_width, image_height) for (col, row) in contour]
        for contour in keypoints
    ]

    draw_image(world_keypoints, pub_command, loop_rate, vel, accel)

    fig, ax = plt.subplots()
    for contour in world_keypoints:
        xs = [pt[0] for pt in contour]
        ys = [pt[1] for pt in contour]
        ax.plot(ys, xs, 'b-', linewidth=0.5)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.set_title('World Coordinate Preview')
    ax.set_xlabel('Y (mm)')
    ax.set_ylabel('X (mm)')
    plt.show()

    ##=====================================================##

    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Return to the home position
    rospy.loginfo("Task Completed!")

    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
