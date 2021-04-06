#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from math import pow, atan2, sqrt, pi

class TurtleBot3:

    def __init__(self):
    
        rospy.init_node('move_to_goal', anonymous = True)
        
        self.sub = rospy.Subscriber('/odom', Odometry, self.get_odom )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.rate = rospy.Rate(10)
        
        self.tb3pose = Pose()
        
        self.prev_theta = 0.0
        self.theta_sum  = 0.0
        
        
    def get_odom(self, msg):
        
        pos_x, pos_y, theta = self.get_pose(msg)
        
        self.tb3pose_x = pos_x
        self.tb3pose_y = pos_y
        self.tb3pose_theta = theta
        
        if   (self.tb3pose_theta - self.prev_theta) >  5.:
            d_theta = (self.tb3pose_theta - self.prev_theta) - 2 * pi            
        elif (self.tb3pose_theta - self.prev_theta) < -5.:
            d_theta = (self.tb3pose_theta - self.prev_theta) + 2 * pi
        else:
            d_theta = (self.tb3pose_theta - self.prev_theta)

        self.theta_sum  = self.theta_sum + d_theta
        self.prev_theta = self.tb3pose_theta
        
        self.tb3pose_theta = self.theta_sum
        
        
    def get_pose(self, data):
        
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
             data.pose.pose.orientation.z, data.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[2]

        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y

        return pos_x, pos_y, theta
    
    
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose.x-self.tb3pose_x),2) + pow((goal_pose.y-self.tb3pose_y),2))
        

    def get_lin_x(self, goal_pose, Kp_d = 1.1):
        return Kp_d * self.get_dist(goal_pose)
    
    
    def get_angle(self, goal_pose):
        return atan2(goal_pose.y - self.tb3pose_y, goal_pose.x - self.tb3pose_x)

    def get_ang_z(self, goal_pose, Kp_a = 1.0):        
        return Kp_a * (self.get_angle(goal_pose) - self.tb3pose_theta)
        
        
    def move2goal(self):
        goal_pose = Pose()

        goal_pose.x = input("Input x goal: ")
        goal_pose.y = input("Input y goal: ")

        tolerance = input("Input tolerance: ")

        t = Twist()
        cnt4print = 0

        while(self.get_dist(goal_pose) >= tolerance):
            
            if(cnt4print >= 10):
                cnt4print = 0
                self.print_pose()
            
            cnt4print = cnt4print + 1
            
            t.linear.x  = self.get_lin_x(goal_pose)
            t.linear.y  = t.linear.z  = 0
            
            t.angular.x = t.angular.y = 0
            t.angular.z = self.get_ang_z(goal_pose)

            self.pub.publish(t)
            self.rate.sleep()
           
        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        
        print("robot arrived at goal position!")
        
        rospy.spin()
        
        
    def print_pose(self):
        print("p.x: %f,  p.y: %f,  th: %f" %(self.tb3pose_x, self.tb3pose_y, self.tb3pose_theta))
        
        

if __name__ == '__main__':
    try:
        
        tb3 = TurtleBot3()
        tb3.move2goal()
        
    except rospy.ROSInterruptException:  pass
