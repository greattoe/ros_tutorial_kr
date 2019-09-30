#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

ToDEGREE  =  0.0174533

class TurtleBot3:

    def __init__(self):
    
        rospy.init_node('pub_tb3_pose', anonymous = True)
        
        self.sub = rospy.Subscriber('/odom', Odometry, self.get_odom )
        self.pub = rospy.Publisher('/tb3pose', Pose, queue_size = 10)
        
        self.tb3pose = Pose()
        self.rate = rospy.Rate(10)
        
        self.prev_theta = 0.0
        self.theta_sum  = 0.0
        
        
    def get_odom(self, msg):
        
        pos_x, pos_y, theta = self.get_pose(msg)
        
        pose2d = Pose()
        
        pose2d.x = pos_x
        pose2d.y = pos_y
        self.tb3pose.theta = theta
        
        if   (self.tb3pose.theta - self.prev_theta) > 5.:
            d_theta = (self.tb3pose.theta - self.prev_theta) - 2 * pi            
        elif (self.tb3pose.theta - self.prev_theta) < -5.:
            d_theta = (self.tb3pose.theta - self.prev_theta) + 2 * pi
        else:
            d_theta = (self.tb3pose.theta - self.prev_theta)

        self.theta_sum  = self.theta_sum + d_theta
        self.prev_theta = self.tb3pose.theta
        
        pose2d.theta = self.tb3pose.theta = self.theta_sum
        
        self.tb3pose = pose2d
        self.print_pose(self.tb3pose)
        self.pub.publish(self.tb3pose)
        
        
    def get_pose(self, msg):
        
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[2]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        return pos_x, pos_y, theta    
    
        
    def print_pose(self, msg):
        """
        1 Radian = 57.2958 Degree
        1 Degree = 0.0174533 Radian
        """
        print("x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, msg.theta/ToDEGREE))
        
        

if __name__ == '__main__':
    try:
        TurtleBot3()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
