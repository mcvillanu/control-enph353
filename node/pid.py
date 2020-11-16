#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

move = Twist()

bridge = CvBridge()

goal_position = 915
last_position = 915
call_one = True
last_error = -1

# size of the pictures is 720r x 1280c

def return_position(thresh):
    global last_position
    x_sum = 0
    x_count = 0
    hit_white = False

    for i in range(550,580):
        for j in reversed(range(600, 1275)):
            #print("row:", i, "column:",j, "value:", thresh[i][j])
            if(not hit_white and thresh[i][j] > 127):
                hit_white = True
            if(hit_white and thresh[i][j] < 127):
                x_sum += j
                x_count += 1
                hit_white = False

    if x_count > 28:
        global last_position
        last_position = (float(x_sum)/x_count)
        return last_position
    else:
        return last_position

#def is_crosswalk():


def imageCallback(data):
    global last_position
    global goal_position
    global last_error
    global call_one

    try:
        kernel = np.ones((10,10),np.uint)

        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)

        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)


        # # print(thresh.shape)
        # cv2.imshow("frame",opening)
        # cv2.waitKey(10)

        s_error = goal_position - return_position(opening)
        print(s_error)


        # if call_one:
        #     for i in range(1,5):
        #         move.linear.x = 0.13
        #         move.angular.z = 0.5
        #         pub.publish(move)
        #         time.sleep(0.9)
        #     call_one = False

        # if(s_error > 200):
        #     move.linear.x = 0.0
        #     move.angular.z = 0.3 * np.sign(s_error)

        # elif(abs(s_error) > 60):
        #     move.linear.x = 0.01
        #     move.angular.z = 0.12 * np.sign(s_error)
        # else:
        #     move.linear.x = 0.15
        #     move.angular.z = 0
        if(abs(s_error) > 50):
            move.linear.x = 0.0
            move.angular.z = 0.2 * np.sign(s_error)
            last_error = np.sign(s_error)
        else:
            move.linear.x = 0.1
            move.angular.z = -last_error * 0.04


        vel_pub.publish(move)
    except CvBridgeError, e:
        print(e)

if __name__== "__main__":
    image_topic = "R1/pi_camera/image_raw"
    rospy.init_node('controller')
    sub_cam = rospy.Subscriber(image_topic, Image, imageCallback)
    vel_pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
    score_pub = rospy.Publisher('license_plate', String, queue_size=1)
    rospy.sleep(2)
    score_pub.publish("funMode,passwd,0,XR58")
    rospy.Rate(5)
    rospy.spin()