#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt, degrees, radians

MAX_ANG_SPEED =  2.84
ANG_SPD = MAX_ANG_SPEED * 0.125

class TB3Move:

    def __init__(self):    
        rospy.init_node('rotate_by_pose', anonymous = True)
        rospy.Subscriber('/tb3pose', Pose, self.get_pose_cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)        
        
        self.tb3pose  = Pose()  # for subscribe
        self.org      = Pose()  # for store starting point
        
    def get_pose_cb(self, msg):
        # callback function to subscribe "/tb3pose" topic
        self.tb3pose = msg
        
    def update_org(self):
        # save current tb3pose.theta to org.theta when called this function
        self.org = self.tb3pose 
        
    def elapsed_angle(self):
        return abs(self.tb3pose.theta - self.org.theta)
        
    def rotate(self, angle):
        tw = Twist()
        self.update_org()
        print "start from: %s" %(round(degrees(self.org.theta), 2))
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD;
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD;
            
        self.pub.publish(tw)
        while self.elapsed_angle() < abs(angle):    pass
            # print "%s of %s" %(round(degrees(self.elapsed_angle()),2) ,round(degrees(abs(angle)),2))
            
        tw.angular.z =  0;  self.pub.publish(tw)
        print "stop to   : %s" %(round(degrees(self.tb3pose.theta), 2))        

if __name__ == '__main__':
    try:
        tb3 = TB3Move()
        
        angle = radians(input("input angle to rotate(deg): "))
        tb3.rotate(angle)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

