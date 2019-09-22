#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def simple_pub():
    rospy.init_node('sample_pub', anonymous=True)
    pub = rospy.Publisher('hello', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        str = "hello~ %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_pub()
    except rospy.ROSInterruptException:
        print "Program terminated"
