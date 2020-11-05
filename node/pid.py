#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('topic_publisher')
pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(1000)
move = Twist()

bridge = CvBridge()

goal_position = 1150
last_position = 1000

# size of the pictures is 720r x 1280c

def return_position(thresh):
    global last_position
    x_sum = 0
    x_count = 0
    hit_white = False

    for i in range(690,718):
        for j in reversed(range(600, 1279)):
            #print("row:", i, "column:",j, "value:", thresh[i][j])
            if(not hit_white and thresh[i][j] > 127):
                hit_white = True
            if(hit_white and thresh[i][j] < 127):
                x_sum += j
                x_count += 1
                hit_white = False

    if x_count > 15:
        global last_position
        last_position = (float(x_sum)/x_count)
        return last_position
    else:
        return last_position

def imageCallback(data):
    global last_position
    global goal_position

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

        #print(s_error)

        if(abs(s_error) > 60):
            move.linear.x = 0.01
            move.angular.z = 0.153384 * np.sign(s_error)
        else:
            move.linear.x = 0.084339
            move.angular.z = 0

        pub.publish(move)
    except CvBridgeError, e:
        print(e)

while not rospy.is_shutdown():
    image_topic = "R1/pi_camera/image_raw"
    sub_cam = rospy.Subscriber(image_topic, Image, imageCallback)
    rospy.spin()