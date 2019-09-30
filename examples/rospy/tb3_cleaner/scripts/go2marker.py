#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

TARGET_ID   = 5

class TurtleBot3:

    def __init__(self):
    
        rospy.init_node('move_to_marker', anonymous = True)
        
        self.sb_odom = rospy.Subscriber('/odom', Odometry, self.get_odom )
        self.sb_mark = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.ar_pose = Pose()
        self.tb3pose = Pose()
        self.rate = rospy.Rate(10)
        
        self.prev_theta = 0.0
        self.theta_sum  = 0.0
        
        self.tolerance  = 0.15
        self.Kp_dist    = 1.0
        self.Kp_ang     = 1.0
        
        self.target_found = False
        
        
    def get_marker(self, msg):
        
        # n = len(msg.markers)
        
        
        for msg in msg.markers:
            if(msg.id == TARGET_ID):
                
                if(self.target_found == False):
                    self.target_found = True
                
                pos_x, pos_y, theta = self.get_ar_pose(msg)

                self.ar_pose.x = pos_x
                self.ar_pose.y = pos_y
                
                if  (theta >  5.):
                    self.ar_pose.theta = theta - 2 * pi            
                elif(theta < -5.):
                    self.ar_pose.theta = theta + 2 * pi
                else:
                    self.ar_pose.theta = theta
                    
    
    def get_ar_pose(self, data):
        
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[1]
        '''
        theta = theta + pi / 2.0
        
        #
        #  th + 2*pi                th - 2*pi
        # -----------+------------+----------
        #            0           2*pi  
        '''
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
            
        pos_x =  data.pose.pose.position.z
        pos_y =  data.pose.pose.position.y
        
        return pos_x, pos_y, theta
        
        
    def get_odom(self, msg):
        
        pos_x, pos_y, theta = self.get_pose(msg)
        
        self.tb3pose.x = pos_x
        self.tb3pose.y = pos_y
        self.tb3pose.theta = theta
        
        if   (self.tb3pose.theta - self.prev_theta) > 5.:
            d_theta = (self.tb3pose.theta - self.prev_theta) - 2 * pi            
        elif (self.tb3pose.theta - self.prev_theta) < -5.:
            d_theta = (self.tb3pose.theta - self.prev_theta) + 2 * pi
        else:
            d_theta = (self.tb3pose.theta - self.prev_theta)

        self.theta_sum  = self.theta_sum + d_theta
        self.prev_theta = self.tb3pose.theta
        
        self.tb3pose.theta = self.theta_sum
        
        
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
    
    
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose.x-self.tb3pose.x),2) + pow((goal_pose.y-self.tb3pose.y),2))
        

    def get_lin_x(self, goal_pose):
        return self.Kp_dist * self.get_dist(goal_pose)
    
    '''
    def get_angle(self, goal_pose):
        return atan2(goal_pose.y - self.tb3pose.y, goal_pose.x - self.tb3pose,x)
    '''
    
    def get_ang_z(self, goal_pose):        
        return self.Kp_ang * (goal_pose.theta - self.tb3pose.theta)
        
        
    def move2marker(self):
        
        goal_pose = Pose()
        
        goal_pose = self.ar_pose
        '''
        goal_pose.x = ADJ_ORG_VAL + x # input("Input x goal: ")
        goal_pose.y = ADJ_ORG_VAL + y # input("Input y goal: ")

        tolerance = tol # input("Input tolerance: ")
        '''
        t = Twist()
        
        cnt4print  = 0
        
        while(self.target_found == False):
            self.rate.sleep()
        
        while(self.get_dist(goal_pose) >= self.tolerance):
            
            if(cnt4print >= 10):
                cnt4print = 0
                self.print_pose()
                self.print_ar_pose()
                
            cnt4print   = cnt4print + 1
            
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
        print("## robot  pose ##")
        print("p.x: %f,  p.y: %f,  th: %f" %(self.tb3pose.x, self.tb3pose.y,  self.tb3pose.theta/0.0174533))
        
        
    def print_ar_pose(self):
        print("## marker pose ##")
        print("p.x: %f,  p.y: %f,  th: %f" %(self.ar_pose.x, self.ar_pose.y, self.ar_pose.theta/0.0174533))
        
        

if __name__ == '__main__':
    try:
        
        tb3 = TurtleBot3()
        tb3.move2marker()
        
    except rospy.ROSInterruptException:  pass
