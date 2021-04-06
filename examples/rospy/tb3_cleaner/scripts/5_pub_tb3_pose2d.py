#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import degrees, pi
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

class TB3Pose2D:

    def __init__(self):
    
        rospy.init_node('pub_tb3_pose', anonymous = True)
        
        rospy.Subscriber('/odom', Odometry, self.get_odom )
        self.pub = rospy.Publisher('/tb3pose', Pose, queue_size = 10)
        
        self.tb3pose2d = Pose()        
        self.prv_theta = 0.0
        self.theta_sum = 0.0
        self.rate = rospy.Rate(10)
        
        
    def get_odom(self, dat):
        
        pos_x, pos_y, theta = self.get_pose(dat)
        
        pose2d       = Pose()   # turtlesim.msg.Pose()
        pose2d.x     = pos_x
        pose2d.y     = pos_y
        pose2d.theta = theta
        pose2d.linear_velocity  = dat.twist.twist.linear.x
        pose2d.angular_velocity = dat.twist.twist.linear.x
        
        if   (pose2d.theta - self.prv_theta) >  5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (pose2d.theta - self.prv_theta) - 2 * pi            
        elif (pose2d.theta - self.prv_theta) < -5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (pose2d.theta - self.prv_theta) + 2 * pi
        else:
            d_theta = (pose2d.theta - self.prv_theta)

        self.theta_sum = self.theta_sum + d_theta
        self.prv_theta = pose2d.theta
        
        pose2d.theta = self.theta_sum
        
        self.tb3pose2d = pose2d
        # self.print_pose(self.tb3pose2d)
        self.pub.publish(self.tb3pose2d)
        
        
    def get_pose(self, msg):
        
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                                            # quart[0] = roll
        quart = euler_from_quaternion(q)    # quart[1] = pitch
        theta = quart[2]                    # quart[2] = yaw <----
        
    	# make theta within from 0 to 360 degree
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        return pos_x, pos_y, theta
        
        
    def print_pose(self, msg):
        print("x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, degrees(msg.theta)))


if __name__ == '__main__':
    try:
        TB3Pose2D()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

