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

goal_position = 915 #915
last_position = 915 #915
call_one = True
last_error = -1
crosswalk_count = 0

lower = np.array([230,0,0], dtype = "uint8")
upper = np.array([255,50,50], dtype = "uint8")

# size of the pictures is 720r x 1280c

def return_position(thresh):
    global last_position
    x_sum = 0
    x_count = 0
    hit_white = False

    for i in range(550,580): #550,580
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

def at_crosswalk(mask):
    for i in range (650, 700):
        for j in range (500,700):
            if(mask[i][j]) == 255:
                return True

    return False

def imageCallback(data):
    global crosswalk_count
    global last_position
    global goal_position
    global last_error
    global call_one


    try:
        if crosswalk_count > 0:
            if crosswalk_count == 3:
                crosswalk_count = 0
            else:
                crosswalk_count +=1
                return

        kernel = np.ones((10,10),np.uint)

        rgb_image = bridge.imgmsg_to_cv2(data)


    # find the colors within the specified boundaries and apply
    # the mask
        
    #         output = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)
    # # show the images
    #         #cv2.imshow("images", np.hstack([rgb_image, output]))
    #         cv2.imshow("images", output)
    #         print(mask.shape)
    #         print(mask[447:448][620:650])
    #         cv2.waitKey(0)

        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)

        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)


        # print(thresh.shape)
        # cv2.imshow("frame",opening)
        # cv2.waitKey(10)

        s_error = goal_position - return_position(opening)
        print(s_error)


        # if call_one:
        #     for i in range(1,5):
        #         move.linear.x = 0.15
        #         move.angular.z = 0.5
        #         pub.publish(move)
        #         time.sleep(0.9)
        #     call_one = False


        
        mask = cv2.inRange(rgb_image, lower, upper)
        # output = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)
        at_cross_bool = at_crosswalk(mask)

        # cv2.imshow("images", output)
        # cv2.waitKey(1)

        if at_cross_bool:
            crosswalk_count += 1
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            time.sleep(0.2)

            move.linear.x = 0.3
            pub.publish(move)
            time.sleep(2)

            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            time.sleep(0.2)

            print("at red line")
            return

        if(abs(s_error) > 50):
            move.linear.x = 0.0
            move.angular.z = 0.2 * np.sign(s_error)
            last_error = np.sign(s_error)
        else:
            move.linear.x = 0.1
            move.angular.z = -last_error * 0.03

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