#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion, quaternion_from_euler

TARGET_ID =  5

ToRADIAN  = 57.2958
ToDEGREE  =  0.0174533


class AR_Marker:

    def __init__(self):
    
        rospy.init_node('pub_marker_pose', anonymous = True)
        
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.pub = rospy.Publisher('/ar_pose', Pose, queue_size = 10)
        
        self.rate = rospy.Rate(10)
        
        
    def get_marker(self, msg):
    
        p = Pose()
        
        for msg in msg.markers:
            if msg.id == TARGET_ID:
            
                pos_x, pos_y, theta = self.get_ar_pose(msg)

                p.x = pos_x
                p.y = pos_y
                
                if  (theta >  5.):
                    p.theta = theta - 2 * pi            
                elif(theta < -5.):
                    p.theta = theta + 2 * pi
                else:
                    p.theta = theta
                
                self.print_pose(p)
                self.pub.publish(p)
        
        """
                  y                        z 
                  ^  x                     ^
          marker  | /                      | robot 
        (on wall) |/                       | 
                  +------> z      x <------+  
                                          /
                                         /
                                        y        
        
          orientation x,y,z,w --+
                                +--> 4   +-------------------------+
        input orientaion of marker ----->|                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <--------|                         |
                                 +-- 3   +-------------------------+
                 r,p,y angle <---+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
                
    
    def get_ar_pose(self, msg):
        
        """
        x --->  z (yaw  ) 
        y ---> -y (pitch) 
        z --->  x (roll ) 
        """
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[1]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        
        pos_x =  msg.pose.pose.position.z
        pos_y = -msg.pose.pose.position.y

        return pos_x, pos_y, theta
    
    
        
    def print_pose(self, msg):
        """
        1 Radian = 57.2958 Degree
        1 Degree = 0.0174533 Radian
        """
        print("x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, msg.theta/ToDEGREE))
          

if __name__ == '__main__':
    try:
        
        AR_Marker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
