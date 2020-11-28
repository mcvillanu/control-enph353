
import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

"""
Notes / TODO:
size of the pictures is 720r x 1280c
can combine is_per and is_car
rename crosswalk_count to something more general
"""

# objects
move = Twist()
bridge = CvBridge()
backSub = cv2.createBackgroundSubtractorMOG2() # background subtractor for pedestrians
backSub_turn = cv2.createBackgroundSubtractorMOG2() # background subtractor for vehicle

# line following parameters
goal_position = 920 # goal position for right side following
last_position = 920 # stores last position for right side following

goal_position_L = 370 # goal position for left side following
last_position_L = 370 # stores last position for left side following

last_error = -1 # direction of last error, for straight driving correction

# image processing parameters 
kernel = np.ones((10,10),np.uint) # 10x10 kernel to remove noise from images

lower_crosswalk = np.array([230,0,0], dtype = "uint8") # lower boundary for crosswalk detection
upper_crosswalk  = np.array([255,50,50], dtype = "uint8") # upper boundary for crosswalk detection

low_boundary_north_cars = np.array([85,85,185], dtype = "uint8") # lower boundary for parked cars on the north side
high_boundary_north_cars = np.array([115,115,215], dtype = "uint8") # upper boundary for parked cars on the north side
low_boundary_south_cars = np.array([0,0,75], dtype = "uint8") # lower boundary for parked cars on the south side
high_boundary_south_cars = np.array([15,15,120], dtype = "uint8") # upper boundary for parked cars on the south side
low_boundary_ew_cars = np.array([10,10,110], dtype = "uint8")
high_boundary_ew_cars = np.array([30,30,130], dtype = "uint8")

# iteration variables
crosswalk_count = 0 # discards frames after hardcoded movements
no_per_count = 0 # waits for a pedestiran to be frozen for x frames
no_car_count = 0 # waits for a vehicle to be frozen for x frames
backsub_count = 0 # training the pedestrian background subtractor for x iterations
backsub_count_turn = 0 # training the vehicle background subtractor for x iterations

# states
back_sub_bool = False # subtracting the background at a crosswalk
back_sub_bool_turn = False # subtracting the background at the entrance to the middle
found_six_bool = False # found the outer six plates, ready to go to the middle
call_one = True # first iteration of the code, hard coded left turn
in_middle = False # in the middle loop, using right following
waiting_to_go = False # at the entrance to the middle, ready for background subtraction

# function to return position of white line to the right of the view
def return_position_L(thresh):
    global last_position_L

    x_sum_L = 0
    x_count_L = 0
    hit_white_L = False

    for i in range(550,580): #550,580
        for j in range(150, 600):
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

# function to return position of white line to the left of the view
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

# checking the frame if there is a crosswalk immediately in front of the vehicle
def at_crosswalk(mask):
    for i in range (650, 700):
        for j in range (500,700):
            if(mask[i][j]) == 255:
                return True
    return False

# checks a background subtracted frame to check if there is movement in the center of the frame
def is_person(frame):
    rows_to_check = [(370,380),(390,400),(410,420),(440,450)]

    for row1,row2 in rows_to_check:
        for r in range(row1, row2):
            for col in range(360,1024):
                if frame[r][col] == 255:
                    return True
    return False

# checks if there is vehicle movement in the frame
def is_car(frame):
    print(frame.shape)
    for x in range(0, frame.shape[0]):
        for y in range(0,frame.shape[1]):
            if not frame[x][y] == 0:
                return True
    return False

# checks if there is an undisturbed section of sky in the frame
def is_sky(clip):
    opening_sky = cv2.morphologyEx(clip, cv2.MORPH_OPEN, kernel)
    for i in range(0,opening_sky.shape[0]):
        for j in range(0,opening_sky.shape[1]):
            if opening_sky[i][j] <= 250:
                return False
    return True

# runs our main logic, is called everytime a new frame is passed
def imageCallback(data):
    # states
    global call_one
    global in_middle
    global waiting_to_go
    global found_six_bool
    global back_sub_bool
    global back_sub_bool

    # frame processing parameters
    global kernel
    global lower_crosswalk
    global upper_crosswalk

    global low_boundary_north_cars
    global high_boundary_north_cars
    global low_boundary_south_cars
    global high_boundary_south_cars
    global low_boundary_ew_cars
    global high_boundary_ew_cars

    # iteration parameters
    global crosswalk_count
    global backsub_count
    global no_per_count
    global no_car_count
    global backsub_count_turn

    # line following parameters
    global goal_position
    global last_position
    global goal_position_L
    global last_position_L
    global last_error
    
    try:
        # initial image processing for all necessary frames
        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask_crosswalk = cv2.inRange(rgb_image, lower_crosswalk , upper_crosswalk )

        # uncomment this section to display vehicle view and manually control the car
        # cv2.imshow("ddd",rgb_image)
        # cv2.waitKey(1)
        # return

        """
        Crosswalk Background Subtraction

        This state is when have confirmed that we are at a crosswalk and we start doing background subtraction.
        We train the background subtractor for 10 iterations, then we check for movement afterwards.
        If we can get seven consecutive frames without pedestrian movement, we perform a hardcoded straight drive and stop.

        We then move back into the previous driving state.
        """
        if back_sub_bool:
            fgMask = backSub.apply(rgb_image)
            no_noise_fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_OPEN, kernel)

            if no_per_count == 10:
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

        """
        Frame Discarder

        Although inadequately named, this state is activated after hardcoded movements such as after
        crosswalks and after turning into the middle. This section discards 10 frames since using time.sleep
        seems to store/queue the frames even with queue_size set to 1.

        We then move back into the previous driving state.
        """
        if crosswalk_count > 0:
            if crosswalk_count == 10:
                crosswalk_count = 0
            else:
                crosswalk_count +=1
                print("cw count", crosswalk_count)

        """
        Call One

        This performs the hardcoded intitial left turn.

        We then move into the right side follow driving state.
        """
        if call_one:
            for i in range(1,5):
                move.linear.x = 0.15
                move.angular.z = 0.58
                pub.publish(move)
                time.sleep(1)
            call_one = False

        """
        At Crosswalk

        This checks if we are at a crosswalk. If so, then we stop movement and initiate the crosswalk background subtractor.
        """
        if at_crosswalk(mask_crosswalk):
            crosswalk_count += 1
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            time.sleep(0.2)
            back_sub_bool = True

            print("at red line")
            return

        """
        Waiting to Turn into Middle

        This state is true if we are at the entrance to the middle section. We perform vehicle background subtraction
        then a hardcoded left turn if there is no vehicle. Like the pedestrian background subtraction, we must train
        our subtractor for a few iterations and also discard frames after our turn.

        We then move into the "in_middle" state.
        """
        if waiting_to_go:
            fgMask_turn = backSub_turn.apply(rgb_image)
            no_noise_fgMask_turn = cv2.morphologyEx(fgMask_turn, cv2.MORPH_OPEN, kernel)

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
                move.linear.x = 0.14
                move.angular.z = 0.6
                pub.publish(move)
                time.sleep(3)

                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)

                waiting_to_go = False
                in_middle = True
                backsub_count_turn = 5
                no_car_count = 0
                crosswalk_count += 1
                last_error = -1
            return

        """
        Found Six Outer Plates

        This state happens when we find all six outer plates. We immediately switch into left follow driving in order to get to the
        entrance of the middle section. We follow until we are able to see uninterupped sky in the top of our view. We then
        exit this state and move into vehicle background subtraction.
        """
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

            line_fol_mask = cv2.inRange(rgb_image, low_boundary_ew_cars, high_boundary_ew_cars)
            opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, kernel)

            s_error = goal_position_L - return_position_L(opening + opening_line_fol)

            if(abs(s_error) > 220):
                move.linear.x = 0.02
                move.angular.z = 0.75 * np.sign(s_error)
            elif(abs(s_error) > 120):
                move.linear.x = 0.03
                move.angular.z = 0.45 * np.sign(s_error)
                last_error = np.sign(s_error)
            else:
                move.linear.x = 0.12
                move.angular.z = -last_error * 0.04
            pub.publish(move)
        

        
        # Right Follow Driving

        # This is the most general state. We will be right side following for the outer and inner loops.
        
        else:
            # Middle Right Follow Driving

            # In the middle, we follow lines described by the sum of the lines and the edges of parked cars.
            # We also will have different parameters from normal right driving since the boundaries of the parked
            # cars are quite deep relative to the lines.

            if in_middle:
                line_fol_mask = cv2.inRange(rgb_image, low_boundary_south_cars, high_boundary_south_cars) + cv2.inRange(rgb_image,low_boundary_north_cars,high_boundary_north_cars)
                opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, kernel)

                s_error = goal_position - return_position(opening+opening_line_fol)

                if(abs(s_error) > 220):
                    move.linear.x = 0.0
                    move.angular.z = 0.6 * np.sign(s_error)
                elif(abs(s_error) > 150):
                    move.linear.x = 0.0
                    move.angular.z = 0.5 * np.sign(s_error)
                    last_error = np.sign(s_error)
                elif(abs(s_error) > 100):
                    move.linear.x = 0.0
                    move.angular.z = 0.3 * np.sign(s_error)
                    last_error = np.sign(s_error)
                elif(abs(s_error) > 70):
                    move.linear.x = 0.0
                    move.angular.z = 0.15 * np.sign(s_error)
                    last_error = np.sign(s_error)
                else:
                    move.linear.x = 0.12
                    move.angular.z = -last_error * 0.04
                pub.publish(move) 

            
            # General Right Follow Driving

            # This is the general driving state for the outer loop.
            else:
                s_error = goal_position - return_position(opening)
                print("right follow ", s_error)

                # if(abs(s_error) > 220):
                #     move.linear.x = 0.0
                #     move.angular.z = 0.5

                # elif(abs(s_error) > 120):
                #     move.linear.x = 0.0
                #     move.angular.z = 0.25 * np.sign(s_error)
                #     last_error = np.sign(s_error)
                # elif(abs(s_error) > 50):
                #     move.linear.x = 0.0
                #     move.angular.z = 0.15 * np.sign(s_error)
                #     last_error = np.sign(s_error)
                # elif(abs(s_error) > 20):
                #     move.linear.x = 0.0
                #     move.angular.z = 0.05 * np.sign(s_error)
                #     last_error = np.sign(s_error)
                # else:
                #     move.linear.x = 0.15
                #     move.angular.z = -last_error * 0.01
                # pub.publish(move)    
                if(abs(s_error) > 220):
                    move.linear.x = 0.0
                    move.angular.z = 0.4
                elif(abs(s_error) > 50):
                    move.linear.x = 0.0
                    move.angular.z = 0.19 * np.sign(s_error)
                    last_error = np.sign(s_error)
                elif(abs(s_error) > 20):
                    move.linear.x = 0.0
                    move.angular.z = 0.05 * np.sign(s_error)
                    last_error = np.sign(s_error)
                else:
                    move.linear.x = 0.15
                    move.angular.z = -last_error * 0.015
                pub.publish(move)  
        
    except CvBridgeError, e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('controller')

    image_topic = "R1/pi_camera/image_raw"
    sub_cam = rospy.Subscriber(image_topic, Image, imageCallback)

    pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
    score_pub = rospy.Publisher('license_plate', String, queue_size=1)

    rospy.sleep(2)
    score_pub.publish("funMode,passwd,0,XR58")

    rospy.Rate(5)
    rospy.spin()