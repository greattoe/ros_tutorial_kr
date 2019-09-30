#!/usr/bin/env python

import rospy, math
import numpy as np
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from math import pow, atan2, sqrt

class TurtleBot3:

    def __init__(self):
    
        rospy.init_node('move2goal', anonymous = True)
        
        self.sub  = rospy.Subscriber('/odom', Odometry, self.get_odom )
        
        self.odom = Odometry()
        self.rate = rospy.Rate(10)        
        
        
    def get_odom(self, msg_odom):
        
        self.odom = msg_odom
        
        
    def print_pose(self):
        print(" ")
        print("position.x    = %f" %(self.odom.pose.pose.position.x)
        print("position.y    = %f" %(self.odom.pose.pose.position.x)
        print("position.z    = %f" %(self.odom.pose.pose.position.x)
        print(" ")
        print("orientation.x = %f" %(self.odom.pose.pose.orientation.x)
        print("orientation.y = %f" %(self.odom.pose.pose.orientation.x)
        print("orientation.z = %f" %(self.odom.pose.pose.orientation.x)
        print("orientation.w = %f" %(self.odom.pose.pose.orientation.x)
        print(" ")
        
        

if __name__ == '__main__':
    try:
        
        x = TurtleBot3()
        x.move2goal()
        #x.print_pose()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
