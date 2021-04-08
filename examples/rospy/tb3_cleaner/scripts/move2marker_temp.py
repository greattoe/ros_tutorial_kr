#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees, pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = 4

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# set parking zone( kind of tolerlance )
MAX_DIST = 0.20
MIN_DIST = 0.15

class Move2Marker:

    def __init__(self):
    
        rospy.init_node('align_to_marker', anonymous = True)       
        rospy.Subscriber('/tb3pose', Pose, self.get_theta)       
        rospy.Subscriber('/marker_pose', Pose, self.get_xy_theta)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.ang_speed = MAX_ANG_SPEED * 0.100
        self.lin_speed = MAX_LIN_SPEED * 0.125
        
        self.tw = Twist()
        self.tw.angular.z = self.ang_speed
        
        self.curr_th   = 0
        self.find_th   = 0
        self.lost_th   = 0
        self.dist      = 0
        
        self.dist_x    = 0
        self.dist_y    = 0
        self.theta     = 0 
        
        self.x         = 0
        self.y         = 0
        self.th        = 0
        self.wise      = 1
        
        self.is_1st_find       = True
        self.is_1st_lost       = True
        self.align_finished    = False
        self.go2front_finished = False
             
        
    def get_theta(self, dat):
        self.curr_th = dat.theta 
        
        
    def get_xy_theta(self, xyt):
        self.dist_x = xyt.x;    self.dist_y = xyt.y;    self.theta = xyt.theta        
        
        
    def get_marker(self, msg):
        
        self.pub.publish(self.tw)
        
        if len(msg.markers) > 0:
          
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                
                    print "found target marker"
                    self.dist = msg.pose.pose.position.z                    
                
                    if  self.is_1st_find == True:
                        self.find_th  = self.curr_th
                        print "get theta to start recognize marker."
                        self.is_1st_find  = False
                        
                    if self.align_finished == True:
                        if self.go2front_finished == False:
                            self.go2front()
                            self.go2front_finished = True
                        self.go2marker()
                else:
                    print "id mismatch"
                    
                
        else: # lost marker            
                
            print "lost marker"
        
            if self.is_1st_find == False and self.is_1st_lost == True:
                self.lost_th = self.curr_th
                print "get theta to end recognize marker."
                self.is_1st_lost = False
                
                if self.align_finished == False:
                    self.align2marker()
                    self.align_finished = True
                    
                    if self.theta < 0:
                        self.wise = -1
                        
                    self.x = self.dist_x
                    self.y = self.dist_y
                    self.th = 0.5 * pi - self.theta                    
                
                
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
                    
                
                
    def go2front(self):
        
        lin_speed = MAX_LIN_SPEED * 0.5
        ang_speed = MAX_ANG_SPEED * 0.125
        
        current = self.curr_th        
        target = current + self.th # * 0.7
        
        self.tw.angular.z = ang_speed * self.wise      
        self.pub.publish(self.tw)
        
        while self.curr_th > target:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        
        duration = self.y / lin_speed
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.tw.linear.x = lin_speed       
        self.pub.publish(self.tw)
        
        while time2end > rospy.Time.now():  pass
        
        self.tw.linear.x = 0
        self.pub.publish(self.tw)
        
        current = self.curr_th
        
        if -self.wise < 0:
            target = current - 0.5 * pi # * 0.7
        else:
            target = current + 0.5 * pi # * 0.7
        
        self.tw.angular.z = ang_speed * -self.wise      
        self.pub.publish(self.tw)
        
        while self.curr_th > target:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)        
        
        
    def go2marker(self):
        '''
        # marker
        #   |        min    max
        #   |   --->  |      |  <---
        #   +---------+------+----------------
        #    backward   stop   forward 
        '''
        speed = MAX_LIN_SPEED * 0.125
        
        if   self.dist > MAX_DIST:
            if self.dist - MAX_DIST > 0.20:
                self.tw.linear.x =  speed * 2.0
            else:
                self.tw.linear.x =  speed * 0.55
        elif self.dist < MIN_DIST:
            self.tw.linear.x =  speed * 0.45 * -1
            
        else:
            self.tw.linear.x = 0;   self.pub.publish(self.tw) 
        
        self.pub.publish(self.tw)        
        print "distance to marker = %f(cm)" %(self.dist * 100)
          

if __name__ == '__main__':
    try:
        Move2Marker()        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
