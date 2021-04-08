#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID =  13

class MarkerPose:

    def __init__(self):
    
        rospy.init_node('pub_marker_pose', anonymous = True)        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.pub = rospy.Publisher('/marker_pose', Pose, queue_size = 10)
        
        
    def get_marker(self, msg):
    
        p = Pose()
        
        for msg in msg.markers:
            if msg.id == TARGET_ID:
            
                pos_x, pos_y, theta = self.get_ar_pose(msg)

                p.x = pos_x
                p.y = pos_y
                
                if  (theta >  5.0):
                    p.theta = theta - 2 * pi            
                elif(theta < -5.0):
                    p.theta = theta + 2 * pi
                else:
                    p.theta = theta
                
                # self.print_pose(p)
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
             
        quart = euler_from_quaternion(q)
        theta = quart[1]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        
        pos_x =  msg.pose.pose.position.z
        pos_y = -msg.pose.pose.position.y

        return pos_x, pos_y, theta    
    
        
    def print_pose(self, msg):
        print "x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, degrees(msg.theta))
          

if __name__ == '__main__':
    try:
        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
