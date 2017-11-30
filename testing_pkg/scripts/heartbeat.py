#!/usr/bin/env python

import rospy
from std_msgs.msg import String

counter = 0

def talker():
    global counter

    pub = rospy.Publisher('heartBeat', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        sending_str = str(counter)
	print sending_str
       # rospy.loginfo(hello_str)
        pub.publish(String(sending_str))
	counter +=1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
