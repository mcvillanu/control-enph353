import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


# objects
move = Twist()
bridge = CvBridge()

# line following parameters
goal_position = 920 # goal position for right side following
goal_position_L = 370 # goal position for left side following

# image processing parameters 
kernel = np.ones((10,10),np.uint) # 10x10 kernel to remove noise from images
small_kernel = np.ones((5,5),np.uint)

lower_crosswalk = np.array([230,0,0], dtype = "uint8") # lower boundary for crosswalk detection
upper_crosswalk  = np.array([255,50,50], dtype = "uint8") # upper boundary for crosswalk detection

low_boundary_north_cars = np.array([85,85,185], dtype = "uint8") # lower boundary for parked cars on the north side
high_boundary_north_cars = np.array([115,115,215], dtype = "uint8") # upper boundary for parked cars on the north side
low_boundary_south_cars = np.array([0,0,75], dtype = "uint8") # lower boundary for parked cars on the south side
high_boundary_south_cars = np.array([15,15,120], dtype = "uint8") # upper boundary for parked cars on the south side
low_boundary_ew_cars = np.array([5,5,105], dtype = "uint8")
high_boundary_ew_cars = np.array([35,35,135], dtype = "uint8")

states = ["call_one", 
         "waiting_to_go", 
         "in_middle",
         "found_six_bool", 
         "back_sub_bool", 
         "back_sub_bool_turn", 
         "blue_wait"]

class Control():
    def __init__(self):
        self.call_one = True
        self.waiting_to_go = False
        self.in_middle = False
        self.found_six_bool = False
        self.back_sub_bool = False
        self.back_sub_bool_turn = False
        self.blue_wait = False

        self.first_cw = True
        self.ped_back_sub = cv2.createBackgroundSubtractorMOG2()
        self.vehicle_back_sub = cv2.createBackgroundSubtractorMOG2()
        self.last_position_R = 920
        self.last_position_L = 370
        self.last_error_array = [0,0,0]
        self.no_line_count = 0
        self.last_error = -1

        self.crosswalk_count = 0
        self.no_per_count = 0
        self.no_car_count = 0
        self.backsub_count = 0
        self.backsub_count_turn = 0
        self.last_pause = -10
        self.blue_wait_count = 0
        self.last_pause = rospy.get_rostime().secs


    def return_position_L(self, thresh):
        global last_position_L

        x_sum_L = 0
        x_count_L = 0
        hit_white_L = False

        for i in range(550,580): #550,580
            for j in range(100, 600):
                if(not hit_white_L and thresh[i][j] > 127):
                    hit_white_L = True

                if(hit_white_L and thresh[i][j] < 127):
                    x_sum_L += j
                    x_count_L += 1
                    hit_white_L = False

        if x_count_L > 15:
            self.last_position_L = (float(x_sum_L)/x_count_L)

    def return_position(self, thresh):
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
            self.last_position_R = (float(x_sum)/x_count)

    def at_crosswalk(self, mask):
        for i in range (650, 700):
            for j in range (500,700):
                if(mask[i][j]) == 255:
                    return True
        return False


    def is_person(self, frame):
        rows_to_check = [(370,380),(390,400),(410,420),(440,450)]

        for row1,row2 in rows_to_check:
            for r in range(row1, row2):
                for col in range(150,1154):
                    if not frame[r][col] == 0:
                        return True
        return False

    # checks if there is vehicle movement in the frame
    def is_car(self, frame):
        for x in range(0, frame.shape[0]):
            for y in range(0,frame.shape[1]):
                if frame[x][y] != 0:
                    return True
        return False

    # checks if there is an undisturbed section of sky in the frame
    def is_sky(self, clip):
        opening_sky = cv2.morphologyEx(clip, cv2.MORPH_OPEN, kernel)
        for i in range(0,opening_sky.shape[0]):
            for j in range(0,opening_sky.shape[1]):
                if opening_sky[i][j] <= 250:
                    return False
        return True

    def shift_array(self, array, new_val):
        val1 = array[1]
        val2 = array[2]
        ret_array = [val1, val2, new_val]
        self.last_error_array = ret_array[:]

    def all_same(self, array, new_val):
        if array[0] == array[1] == array[2] == new_val:
            return True
        else:
            return False

    def outerLapCallback(self, state):
        self.found_six_bool = True

    def blueCallback(self, state):
        if rospy.get_rostime().secs - self.last_pause > 8:
            self.blue_wait = True
            self.last_pause = rospy.get_rostime().secs

    def imageCallback(self, data):
        try:
            rgb_image = bridge.imgmsg_to_cv2(data)
            gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray_frame,220,255, cv2.THRESH_BINARY)
            opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            mask_crosswalk = cv2.inRange(rgb_image, lower_crosswalk , upper_crosswalk)


            if self.blue_wait:
                print("blue wait")
                self.blue_wait_count += 1

            
                if self.in_middle:
                    move.linear.x = 0.0
                    move.angular.z = -0.8
                    pub.publish(move)
                    rospy.sleep(0.4)
                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)
                    rospy.sleep(3)

                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)
                else:
                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)

                    if self.blue_wait_count == 3 or self.blue_wait_count == 5 or self.blue_wait_count == 6:
                        print("AT P4")
                        rospy.sleep(4)
                    else:
                        rospy.sleep(2)

                self.blue_wait = False

            """
            Crosswalk Background Subtraction
            This state is when have confirmed that we are at a crosswalk and we start doing background subtraction.
            We train the background subtractor for 10 iterations, then we check for movement afterwards.
            If we can get seven consecutive frames without pedestrian movement, we perform a hardcoded straight drive and stop.
            We then move back into the previous driving state.
            """
            if self.back_sub_bool:
                fgMask = self.ped_back_sub.apply(rgb_image)
                no_noise_fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_OPEN, small_kernel)
                limit = 2
                print(self.no_per_count)

                if self.first_cw:
                    limit = 6

                if self.no_per_count == limit:
                    print("limit", limit)
                    move.linear.x = 0.27
                    pub.publish(move)
                    rospy.sleep(2)

                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)
                    rospy.sleep(0.5)

                    self.back_sub_bool = False
                    self.backsub_count = 0
                    self.no_per_count = 0

                    self.first_cw = False

                if(self.backsub_count < 10):
                    self.backsub_count += 1
                    return

                if not self.is_person(no_noise_fgMask):
                    self.no_per_count += 1
                else:
                    self.no_per_count = 0
                return

            """
            Frame Discarder
            Although inadequately named, this state is activated after hardcoded movements such as after
            crosswalks and after turning into the middle. This section discards 10 frames since using time.sleep
            seems to store/queue the frames even with queue_size set to 1.
            We then move back into the previous driving state.
            """
            if self.crosswalk_count > 0:
                if self.crosswalk_count == 10:
                    self.crosswalk_count = 0
                    return
                else:
                    self.crosswalk_count +=1
                    print("cw count", self.crosswalk_count)
                    return

            """
            Call One
            This performs the hardcoded intitial left turn.
            We then move into the right side follow driving state.
            """
            if self.call_one:
                for i in range(0,5):
                    move.linear.x = 0.18
                    move.angular.z = 0.57
                    pub.publish(move)
                    rospy.sleep(0.83)
                self.call_one = False

            """
            At Crosswalk
            This checks if we are at a crosswalk. If so, then we stop movement and initiate the crosswalk background subtractor.
            """
            if self.at_crosswalk(mask_crosswalk):
                self.crosswalk_count += 1
                move.linear.x = 0.0
                move.angular.z = 0.0
                pub.publish(move)
                rospy.sleep(0.2)
                self.back_sub_bool = True

                print("at red line")
                return

            """
            Waiting to Turn into Middle
            This state is true if we are at the entrance to the middle section. We perform vehicle background subtraction
            then a hardcoded left turn if there is no vehicle. Like the pedestrian background subtraction, we must train
            our subtractor for a few iterations and also discard frames after our turn.
            We then move into the "in_middle" state.
            """
            if self.waiting_to_go:
                fgMask_turn = self.vehicle_back_sub.apply(rgb_image)
                no_noise_fgMask_turn = cv2.morphologyEx(fgMask_turn, cv2.MORPH_OPEN, small_kernel)
                self.found_six_bool = False

                if(self.backsub_count_turn < 6):
                    print("train")
                    self.backsub_count_turn += 1
                    return

                if self.is_car(no_noise_fgMask_turn[330:450,400:900]):
                    print("there is car", self.no_car_count)
                    self.no_car_count = 0
                else:
                    print("no_car", self.no_car_count)
                    self.no_car_count += 1

                if self.no_car_count == 13:
                    print("going")
                    # no_line_count=0
                    # return
                    move.linear.x = 0.0
                    move.angular.z = 0.8
                    pub.publish(move)
                    rospy.sleep(0.5)

                    move.linear.x = 0.23
                    move.angular.z = 0
                    
                    pub.publish(move)
                    rospy.sleep(1)
                    move.linear.x = 0.0
                    move.angular.z = 0.9
                    pub.publish(move)
                    rospy.sleep(2)

                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)

                    self.waiting_to_go = False
                    self.in_middle = True
                    self.backsub_count_turn = 5
                    self.no_car_count = 0
                    self.crosswalk_count += 1
                    self.last_error = -1
                return

            """
            Found Six Outer Plates
            This state happens when we find all six outer plates. We immediately switch into left follow driving in order to get to the
            entrance of the middle section. We follow until we are able to see uninterupped sky in the top of our view. We then
            exit this state and move into vehicle background subtraction.
            """
            if self.found_six_bool:
                ret_sky,thresh_sky = cv2.threshold(gray_frame[0:50,:],80,255, cv2.THRESH_BINARY)
                print("left follow")

                if self.is_sky(thresh_sky):
                    self.found_six_bool = False
                    self.waiting_to_go = True

                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)
                    move.linear.x = 0.0
                    move.angular.z = -0.8
                    
                    pub.publish(move)
                    rospy.sleep(0.5)
                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)

                    return


                line_fol_mask = cv2.inRange(rgb_image, low_boundary_ew_cars, high_boundary_ew_cars)
                opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, small_kernel)
                shift = opening_line_fol[:,0:1280-100]
                result = np.zeros((opening.shape[0],opening.shape[1]))
                result[:,100:1280] = shift

                self.return_position_L(opening + result)

                s_error = goal_position_L - self.last_position_L

                if(abs(s_error) > 220):
                    move.linear.x = 0.0
                    move.angular.z = 0.5 * np.sign(s_error)
                elif(abs(s_error) > 100):
                    move.linear.x = 0.0
                    move.angular.z = 0.2 * np.sign(s_error)
                    self.last_error = np.sign(s_error)
                else:
                    move.linear.x = 0.11
                    move.angular.z = -self.last_error * 0.04
                pub.publish(move)
            

            
            # Right Follow Driving

            # This is the most general state. We will be right side following for the outer and inner loops.
            
            else:
                # Middle Right Follow Driving

                # In the middle, we follow lines described by the sum of the lines and the edges of parked cars.
                # We also will have different parameters from normal right driving since the boundaries of the parked
                # cars are quite deep relative to the lines.

                if self.in_middle:
                    line_fol_mask = cv2.inRange(rgb_image, low_boundary_south_cars, high_boundary_south_cars) + cv2.inRange(rgb_image,low_boundary_north_cars,high_boundary_north_cars)
                    opening_line_fol = cv2.morphologyEx(line_fol_mask, cv2.MORPH_OPEN, kernel)

                    shifted = opening_line_fol[:,125:]
                    result = np.zeros((opening.shape[0],opening.shape[1]))
                    result[:,0:shifted.shape[1]] = shifted

                    self.return_position(opening+result)


                    s_error = -15 + goal_position - self.last_position_R
                        

                    if(abs(s_error) > 220):
                        move.linear.x = 0.0
                        move.angular.z = 0.6 * np.sign(s_error)
                    elif(abs(s_error) > 100):
                        move.linear.x = 0.0
                        move.angular.z = 0.45 * np.sign(s_error)
                        self.last_error = np.sign(s_error)
                    elif(abs(s_error) > 80):
                        move.linear.x = 0.0
                        move.angular.z = 0.15 * np.sign(s_error)
                        self.last_error = np.sign(s_error)
                    else:
                        move.linear.x = 0.15
                        move.angular.z = -self.last_error * 0.04
                    pub.publish(move)

                    
                # General Right Follow Driving

                # This is the general driving state for the outer loop.
                else:
                    self.return_position(opening)
                    s_error = goal_position - self.last_position_R
                    print("right follow ", s_error)

                    help = self.shift_array(self.last_error_array, s_error)

                    if self.all_same(self.last_error_array, s_error) and s_error < 0:
                        print("fixing it")
                        s_error = 221
                    if(abs(s_error) > 220):
                        move.linear.x = 0.0
                        move.angular.z = 0.75
                    elif(abs(s_error) > 50):
                        move.linear.x = 0.0
                        move.angular.z = 0.2 * np.sign(s_error)
                        self.last_error = np.sign(s_error)
                    elif(abs(s_error) > 20):
                        move.linear.x = 0.0
                        move.angular.z = 0.05 * np.sign(s_error)
                        self.last_error = np.sign(s_error)
                    else:
                        move.linear.x = 0.145
                        move.angular.z = -self.last_error * 0.015
                    pub.publish(move)  
        
        except CvBridgeError, e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('controller')
    image_topic = "R1/pi_camera/image_raw"
    outer_lap_topic = "/outer_lap_complete"

    controller = Control()

    sub_cam = rospy.Subscriber(image_topic, Image, controller.imageCallback)
    pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)

    found_six_sub = rospy.Subscriber(outer_lap_topic, Bool, controller.outerLapCallback)
    blue_sub = rospy.Subscriber('blue_stop',Bool, controller.blueCallback)

    score_pub = rospy.Publisher('license_plate', String, queue_size=1)
    rospy.sleep(2)
    score_pub.publish("funMode,passwd,0,XR58")

    rospy.Rate(5)
    rospy.spin()