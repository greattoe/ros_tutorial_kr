#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import radians, degrees, pi

MAX_ANG_SPEED = 2.84
ANG_SPEED = MAX_ANG_SPEED / 4

class TB3Rotate:

    def __init__(self):    
        rospy.init_node('rotate_by_pose', anonymous = True)        
        rospy.Subscriber('/tb3pose', Pose, self.get_theta)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)        
        self.tb3pose2d = Pose() 
        self.theta_now = 0.0        
        
    def get_theta(self, dat):
        self.theta_now = dat.theta        
        
    def get_goal(self, angle):
        return self.theta_now + angle
        
    def rotate(self, angle):
        tw = Twist()
        target_angle = self.get_goal(angle)
        print "current = %f, target = %f" %(degrees(self.theta_now), degrees(target_angle))
        
        if angle >= 0:	# +angle
            tw.angular.z =  ANG_SPEED;  self.pub.publish(tw)
            while self.theta_now < target_angle:    pass
            tw.angular.z =  0;  self.pub.publish(tw)
        else:			# -angle
            tw.angular.z = -ANG_SPEED;  self.pub.publish(tw)
            while self.theta_now > target_angle:    pass
            tw.angular.z =  0;  self.pub.publish(tw)
            

if __name__ == '__main__':
    try:
        tb3 = TB3Rotate()
        angle = radians(float(input("input angle(deg) to turn: ")))
        tb3.rotate(angle)
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
