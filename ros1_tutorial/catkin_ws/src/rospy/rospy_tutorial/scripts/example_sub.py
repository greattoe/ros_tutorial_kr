#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def cb_func(msg):
    rospy.loginfo(rospy.get_caller_id() + ' msg: %s', msg.data)

def simple_sub():
    rospy.init_node('sample_sub')#, anonymous=True)
    rospy.Subscriber('hello', String, cb_func)
    rospy.spin()

if __name__ == '__main__':
    simple_sub() 
