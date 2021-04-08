#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

TARGET_ID = 13

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84


class AlignMarker:

    def __init__(self):
    
        rospy.init_node('align_to_marker', anonymous = True)       
        rospy.Subscriber('/tb3pose', Pose, self.get_theta)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.tw = Twist()
        self.tw.angular.z = MAX_ANG_SPEED * 0.1
        
        self.curr_th   = 0
        self.find_th   = 0
        self.lost_th   = 0
        
        self.is_1st_find    = True
        self.is_1st_lost    = True
        self.align_finished = False
             
        
    def get_theta(self, dat):
        self.curr_th = dat.theta 
        
        
    def get_marker(self, msg):
        
        self.pub.publish(self.tw)
        
        if len(msg.markers) > 0:
          
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                
                    print "found target marker"
                
                    if  self.is_1st_find == True:
                        self.find_th  = self.curr_th
                        print "get theta to start recognize marker."
                        self.is_1st_find  = False
                else:
                    print "id mismatch"
                
        else: # lost marker            
                
            print "lost marker"
        
            if  self.is_1st_find == False and self.is_1st_lost == True:
                self.lost_th = self.curr_th
                print "get theta to end recognize marker."
                self.is_1st_lost = False
                
                if self.align_finished == False:
                    self.align2marker()
                    self.align_finished = True
                
                
    def align2marker(self):
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        
        current = self.curr_th        
        target = current - abs(self.find_th - self.lost_th) * 0.7
        
        print "get target theta & start rotating."
        self.tw.angular.z = -MAX_ANG_SPEED * 0.125        
        self.pub.publish(self.tw)
        
        while self.curr_th > target:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        print "align complete."
          

if __name__ == '__main__':
    try:
        
        AlignMarker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
