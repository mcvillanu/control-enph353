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

goal_position = 920#915
last_position = 920 #915
call_one = True
last_error = -1
crosswalk_count = 0
back_sub_bool = False
rows_to_check = [(370,380),(390,400),(410,420),(440,450)]
cw_pivot = False
no_per_count = 0
found_six_bool = False

backsub_count = 0

lower = np.array([230,0,0], dtype = "uint8")
upper = np.array([255,50,50], dtype = "uint8")

backSub = cv2.createBackgroundSubtractorMOG2()

# size of the pictures is 720r x 1280c

def return_position(thresh):
    global last_position
    x_sum = 0
    x_count = 0
    hit_white = False

    for i in range(550,580): #550,580
        for j in reversed(range(500, 1200)):
            # print("row:", i, "column:",j, "value:", thresh[i][j])
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

def at_crosswalk(mask):
    for i in range (650, 700):
        for j in range (500,700):
            if(mask[i][j]) == 255:
                return True

    return False

def is_person(frame):
    global rows_to_check

    for row1,row2 in rows_to_check:
        for r in range(row1, row2):
            for col in range(360,1024):
                if frame[r][col] == 255:
                    return True

    return False


def imageCallback(data):
    global crosswalk_count
    global back_sub_bool
    global backsub_count
    global cw_pivot
    global last_position
    global goal_position
    global last_error
    global call_one
    global no_per_count
    # global found_six_bool

    try:
        kernel = np.ones((10,10),np.uint)
        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        cv2.imshow("ddd",rgb_image)
        cv2.waitKey(1)

        # if cw_pivot:
        #     s_error = goal_position - return_position(opening)
        #     print("at cw ", s_error)

        #     if abs(s_error) > 20:
        #         move.angular.z = 0.2 * np.sign(s_error)
        #         pub.publish(move)
        #     else:
        #         move.angular.z = 0
        #         pub.publish(move)
        #         cw_pivot = False
        #     return

        

        if back_sub_bool:
            print("at fgMask")

            fgMask = backSub.apply(rgb_image)
            no_noise_fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_OPEN, kernel)
            # cv2.imshow('Frame', rgb_image)
            # cv2.imshow('FG Mask', no_noise_fgMask)
            # cv2.waitKey(1)

            if no_per_count == 4:
                move.linear.x = 0.3
                pub.publish(move)
                time.sleep(1.8)

                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                time.sleep(0.2)

                back_sub_bool = False
                backsub_count = 0
                no_per_count = 0

            if(backsub_count < 10):
                backsub_count += 1
                print("training backsub", backsub_count)
                return

            if not is_person(no_noise_fgMask):
                no_per_count += 1
            else:
                no_per_count = 0

            return

        if crosswalk_count > 0:
            if crosswalk_count == 6:
                crosswalk_count = 0
            else:
                crosswalk_count +=1
                print("cw count", crosswalk_count)
        #         return

        # if found_six_bool:
        #     move.linear.x = 0.3
        #     pub.publish(move)
        #     time.sleep(1.8)

        #     move.linear.x = 0.0
        #     move.angular.z = 0.0
        #     pub.publish(move)
        #     time.sleep(0.2)

        #     move.linear.x = 0.3
        #     pub.publish(move)
        #     time.sleep(1.8)

        #     move.linear.x = 0.0
        #     move.angular.z = 0.0
        #     pub.publish(move)
        #     time.sleep(0.2)

        #     for i in range(1,5):
        #         move.linear.x = 0.15
        #         move.angular.z = 0.5
        #         pub.publish(move)
        #         time.sleep(0.9)
        #     call_one = False




    # find the colors within the specified boundaries and apply
    # the mask
        
    #         output = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)
    # # show the images
    #         #cv2.imshow("images", np.hstack([rgb_image, output]))
    #         cv2.imshow("images", output)
    #         print(mask.shape)
    #         print(mask[447:448][620:650])
    #         cv2.waitKey(0)

        


        # print(thresh.shape)
        # cv2.imshow("frame",opening)
        # cv2.waitKey(10)

        s_error = goal_position - return_position(opening)
        print(s_error)


        if call_one:
            for i in range(1,5):
                move.linear.x = 0.15
                move.angular.z = 0.5
                pub.publish(move)
                time.sleep(0.9)
            call_one = False


        
        
        # output = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)
        mask = cv2.inRange(rgb_image, lower, upper)
        at_cross_bool = at_crosswalk(mask)

        # cv2.imshow("images", output)
        # cv2.waitKey(1)

        if at_cross_bool:
            # if found_six:
            #     found_six_bool = True
            #     move.linear.x = 0.0
            #     move.angular.z = 0.0
            #     pub.publish(move)
            #     time.sleep(0.2)
            #     return
            # if abs(s_error) > 20:
            #     cw_pivot = True
            #     return
                
            crosswalk_count += 1
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            time.sleep(0.2)
            back_sub_bool = True

            print("at red line")
            return

        if(abs(s_error) > 200):
            move.linear.x = 0.0
            move.angular.z = 0.12
        elif(abs(s_error) > 50):
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