#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees

def get_pose(dat):
    x  = round(dat.x) 
    y  = round(dat.y)
    th = round(degrees(dat.theta))
    rospy.loginfo("x = %s(m), y = %s(m), th = %s(deg)", x, y, th)

def sub_turtle_pose():
    rospy.init_node('sub_turtle_pose')#, anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, get_pose)
    rospy.spin()

if __name__ == '__main__':
    sub_turtle_pose()
