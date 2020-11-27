
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

goal_position_L = 370
last_position_L = 370

call_one = True
last_error = -1

kernel = np.ones((10,10),np.uint)

lower = np.array([230,0,0], dtype = "uint8")
upper = np.array([255,50,50], dtype = "uint8")

lower_line_fol = np.array([0,0,100], dtype = "uint8")
upper_line_fol = np.array([50,50,150], dtype = "uint8")

crosswalk_count = 0
no_per_count = 0

rows_to_check = [(370,380),(390,400),(410,420),(440,450)]
backSub = cv2.createBackgroundSubtractorMOG2()
back_sub_bool = False
backsub_count = 0

found_six_bool = False

no_car_count = 0
waiting_to_go = False
backSub_turn = cv2.createBackgroundSubtractorMOG2()
back_sub_bool_turn = False
backsub_count_turn = 0

in_middle = False

# size of the pictures is 720r x 1280c

def return_position_L(thresh):
    global last_position_L

    x_sum_L = 0
    x_count_L = 0
    hit_white_L = False

    for i in range(550,580): #550,580
        for j in range(200, 600):
            if(not hit_white_L and thresh[i][j] > 127):
                hit_white_L = True

            if(hit_white_L and thresh[i][j] < 127):
                x_sum_L += j
                x_count_L += 1
                hit_white_L = False

    if x_count_L > 15:
        global last_position_L
        last_position_L = (float(x_sum_L)/x_count_L)

    return last_position_L

def return_position(thresh):
    global last_position
    x_sum = 0
    x_count = 0
    hit_white = False

    for i in range(550,580): #550,580
        for j in reversed(range(500, 1200)):
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

# can only pass in a cut frame, make faster
def is_person(frame):
    global rows_to_check

    for row1,row2 in rows_to_check:
        for r in range(row1, row2):
            for col in range(360,1024):
                if frame[r][col] == 255:
                    return True
    return False

def is_car(frame):
    print(frame.shape)
    for x in range(0, frame.shape[0]):
        for y in range(0,frame.shape[1]):
            if not frame[x][y] == 0:
                return True
    return False

def is_sky(clip):
    opening_sky = cv2.morphologyEx(clip, cv2.MORPH_OPEN, kernel)
    for i in range(0,opening_sky.shape[0]):
        for j in range(0,opening_sky.shape[1]):
            if opening_sky[i][j] <= 250:
                return False
    return True


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
    global found_six_bool
    global lower_line_fol
    global upper_line_fol
    global kernel

    global waiting_to_go
    global in_middle

    global backSub_turn
    global back_sub_bool
    global backsub_count_turn
    global no_car_count

    try:
        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)
        
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        # cv2.imshow("ddd",no_noise_fgMask_turn[400:440,250:700])
        # cv2.waitKey(1)
        # return
       
        # print(is_sky(thresh_sky))

        if back_sub_bool:
            fgMask = backSub.apply(rgb_image)
            no_noise_fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_OPEN, kernel)

            if no_per_count == 7:
                move.linear.x = 0.3
                pub.publish(move)
                time.sleep(1.8)

                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                time.sleep(0.5)

                back_sub_bool = False
                backsub_count = 0
                no_per_count = 0

            if(backsub_count < 10):
                backsub_count += 1
                return

            if not is_person(no_noise_fgMask):
                no_per_count += 1
            else:
                no_per_count = 0

            return

        if crosswalk_count > 0:
            if crosswalk_count == 10:
                crosswalk_count = 0
            else:
                crosswalk_count +=1
                print("cw count", crosswalk_count)

        # if call_one:
        #     for i in range(1,5):
        #         move.linear.x = 0.15
        #         move.angular.z = 0.5
        #         pub.publish(move)
        #         time.sleep(0.9)
        #     call_one = False

        mask = cv2.inRange(rgb_image, lower, upper)
        at_cross_bool = at_crosswalk(mask)

        if at_cross_bool:
            crosswalk_count += 1
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            time.sleep(0.2)
            back_sub_bool = True

            print("at red line")
            return

        # once we find all six, then we use left follow if there is a line, otherwise right follow

        if waiting_to_go:
            fgMask_turn = backSub_turn.apply(rgb_image)
            no_noise_fgMask_turn = cv2.morphologyEx(fgMask_turn, cv2.MORPH_OPEN, kernel)
            # cv2.imshow("skljdfkl", no_noise_fgMask_turn)
            # cv2.waitKey(1)
            # print(is_car(no_noise_fgMask_turn[320:490,650:1050]))
            # print(is_car(no_noise_fgMask_turn[400:440,250:700]))

            # return

            if(backsub_count_turn < 6):
                print("train")
                backsub_count_turn += 1
                return

            if is_car(no_noise_fgMask_turn[320:490,650:1050]) and is_car(no_noise_fgMask_turn[400:440,250:700]):
                print("there is car", no_car_count)
                no_car_count = 0
            else:
                print("no_car", no_car_count)
                no_car_count += 1

            if no_car_count == 10:
                print("no car count", no_car_count)
                move.linear.x = 0.13
                move.angular.z = 0.5
                pub.publish(move)
                time.sleep(3.7)

                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)

                waiting_to_go = False
                in_middle = True
                backsub_count_turn = 5
                no_car_count = 0
                crosswalk_count += 1

            

            return

        if found_six_bool:
            ret_sky,thresh_sky = cv2.threshold(gray_frame[0:50,:],80,255, cv2.THRESH_BINARY)
            print("looking for sky")

            if is_sky(thresh_sky):
                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)

                found_six_bool = False
                waiting_to_go = True

                return

            line_fol_mask = cv2.inRange(rgb_image, lower_line_fol, upper_line_fol)
            opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, kernel)

            s_error = goal_position_L - return_position_L(opening + opening_line_fol)

            if(abs(s_error) > 200):
                move.linear.x = 0.02
                move.angular.z = 0.3
            elif(abs(s_error) > 50):
                move.linear.x = 0.02
                move.angular.z = 0.15 * np.sign(s_error)
                last_error = np.sign(s_error)
            else:
                move.linear.x = 0.1
                move.angular.z = -last_error * 0.04
            pub.publish(move)   

            # print("left follow ", s_error)
        else:
            if in_middle:
                line_fol_mask = cv2.inRange(rgb_image, lower_line_fol, upper_line_fol)
                opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, kernel)
                s_error = goal_position - return_position(opening + opening_line_fol)
            else:
                s_error = goal_position - return_position(opening)
                print("right follow ", s_error)
        
        if(abs(s_error) > 220):
            move.linear.x = 0.0
            move.angular.z = 0.75
        elif(abs(s_error) > 50):
            move.linear.x = 0.0
            move.angular.z = 0.2 * np.sign(s_error)
            last_error = np.sign(s_error)
        elif(abs(s_error) > 20):
            move.linear.x = 0.0
            move.angular.z = 0.05 * np.sign(s_error)
            last_error = np.sign(s_error)
        else:
            move.linear.x = 0.13
            move.angular.z = -last_error * 0.015
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