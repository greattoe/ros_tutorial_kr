#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees, pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

# change id of your marker
TARGET_ID = 13

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# set parking zone( kind of tolerlance )
MAX_DIST = 0.20
MIN_DIST = 0.15


class KeepDist:

    def __init__(self):
    
        rospy.init_node('tb3_keep_dist', anonymous = True)        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.tw = Twist()
        self.dist = 0
        self.count = 0
        self.lin_speed = MAX_LIN_SPEED * 0.125
        
        
    def get_marker(self, msg):
    
        if len(msg.markers) > 0:
          
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                    
                    self.dist = msg.pose.pose.position.z
                    self.move2marker()
                else:
                    print "wrong marker id!"
        else:
            print "marker not found!"
        
                
    
    def move2marker(self):
        '''
        # marker
        #   |        min    max
        #   |   --->  |      |  <---
        #   +---------+------+----------------
        #    backward   stop   forward 
        '''
        speed = MAX_LIN_SPEED * 0.15
        
        if   self.dist > MAX_DIST:
            if self.dist - MAX_DIST > 0.20:
                self.tw.linear.x =  speed * 1.0
            else:
                self.tw.linear.x =  speed * 0.75
        elif self.dist < MIN_DIST:        
            self.tw.linear.x =  speed * 0.45 * -1
            
        else:
            self.tw.linear.x = 0
        
        self.pub.publish(self.tw)        
        print "distance to marker = %f(cm)" %(self.dist * 100)
          

if __name__ == '__main__':
    try:
        
        tb3 = KeepDist()
        tb3.move2marker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
