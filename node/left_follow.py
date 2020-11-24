#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


move = Twist()

bridge = CvBridge()

goal_position_R = 920#915
last_position_R = 920 #915

goal_position_L = 370
last_position_L = 370

last_error = -1

call_one = True
crosswalk_count = 0
back_sub_bool = False
rows_to_check = [(370,380),(390,400),(410,420),(440,450)]
cw_pivot = False
no_per_count = 0

backsub_count = 0

# size of the pictures is 720r x 1280c

def return_position(thresh):
    global last_position_R
    global last_position_L

    x_sum_R = 0
    x_count_R = 0
    hit_white_R = False

    x_sum_L = 0
    x_count_L = 0
    hit_white_L = False

    for i in range(550,580): #550,580
        for j in reversed(range(600, 1200)):
            # print("row:", i, "column:",j, "value:", thresh[i][j])
            if(not hit_white_R and thresh[i][j] > 127):
                hit_white_R = True
            if(hit_white_R and thresh[i][j] < 127):
                x_sum_R += j
                x_count_R += 1
                hit_white_R = False

    for i in range(550,580): #550,580
        for j in range(70, 600):
            # print("row:", i, "column:",j, "value:", thresh[i][j])
            if(not hit_white_L and thresh[i][j] > 127):
                hit_white_L = True
            if(hit_white_L and thresh[i][j] < 127):
                x_sum_L += j
                x_count_L += 1
                hit_white_L = False

    if x_count_R > 15:
        global last_position_R
        last_position_R = (float(x_sum_R)/x_count_R)
    if x_count_L > 15:
        global last_position_L
        last_position_L = (float(x_sum_L)/x_count_L)

    return last_position_L, last_position_R

def imageCallback(data):
    global last_position_L
    global last_position_R
    global goal_position_L
    global goal_position_R
    global last_error

    global call_one
    global no_per_count

    try:
        kernel = np.ones((10,10),np.uint)
        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        cv2.imshow("ddd",thresh)
        cv2.waitKey(1)

        position_L, position_R = return_position(opening)

        s_error_L = goal_position_L - position_L
        s_error_R = goal_position_R - position_R
        print(position_L)
        print(s_error_L)

        if abs(s_error_L) < abs(s_error_R):
            s_error = s_error_L
        else:
            s_error = s_error_R



        # if call_one:
        #     for i in range(1,5):
        #         move.linear.x = 0.15
        #         move.angular.z = 0.5
        #         pub.publish(move)
        #         time.sleep(0.9)
        #     call_one = False

        # cv2.imshow("images", output)
        # cv2.waitKey(1)

        # if(abs(s_error_R) > 200):
        #     move.linear.x = 0.0
        #     move.angular.z = 0.12
        if(abs(s_error) > 50):
            move.linear.x = 0.0
            move.angular.z = 0.12 * np.sign(s_error)
            last_error = np.sign(s_error)
        else:
            move.linear.x = 0.075
            move.angular.z = -last_error * 0.04

        pub.publish(move)
    except CvBridgeError, e:
        print(e)

if __name__ == '__main__':
    image_topic = "R1/pi_camera/image_raw"
    rospy.init_node('controller')
    sub_cam = rospy.Subscriber(image_topic, Image, imageCallback)
    pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
    score_pub = rospy.Publisher('license_plate', String, queue_size=1)
    rospy.sleep(2)
    score_pub.publish("funMode,passwd,0,XR58")
    rospy.Rate(5)
    rospy.spin()