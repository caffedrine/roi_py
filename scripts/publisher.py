#!/usr/bin/env python
# Software License Agreement (BSD License)
#

import rospy
from std_msgs.msg import String

#Import our custom message
from roi import *

def talker():
    
    #initialising rospy
    pub = rospy.Publisher('camera_and_imu', String, queue_size=1)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    while not rospy.is_shutdown():

    	#building message we want to send
        hello_str = "hello world %s" % rospy.get_time()
        
        #display message we are about to send
        rospy.loginfo(hello_str)
        
        #send message and all the stuff
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

