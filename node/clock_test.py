from std_msgs.msg import Int64
import rospy

time = 0

def clockCallback(clock):
	print(dir(clock))
	return

if __name__ == '__main__':
    rospy.init_node('clock_tester')
    clock_topic = "/clock"

    clock = rospy.Subscriber(clock_topic, Int64, clockCallback)
    # print(rospy.time)
    rospy.Rate(5)
    rospy.spin()