#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2

"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(radian/sec)
"""
MAX_LIN_X = 0.22
MAX_ANG_Z = 2.82

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2
        
        self.wise  = 1
        
        """        x
                   |
             ------+------
             |\    |    /|                     /|
       case1 |0\   |   /0| case2              / |         1 radian = 57.2958 degree
             |  \  |  /  |                   /30|         1 degree = 0.0174533 radian
       +x,+y |   \0|0/   | +x,-y            /   |
             |    \|/    |               2 /    | sqrt(3)
       y-----+-----+-----+-----           /     |
             |    /|\    |               /      |
       -x,+y |   /0|0\   | -x,-y        /       |
             |  /  |  \  |             /60    90|
       case4 |0/   |   \0| case3      ----------+
             |/    |    \|                 1
             ------+------                            
       
        d = dist  = sqrt(abs(x) + abs(y))
        
        0 = angle = math.atan(abs(y) / abs(x))
        
        case 1:  0 =  math.atan(abs(y) / abs(x))
        case 2: -0 = -math.atan(abs(y) / abs(x))
        case 3: -(180 * 0.0174533 - 0) = -(pi - 0) = -(pi - math.atan(abs(y) / abs(x)))
        case 4:   180 * 0.0174533 - 0  =   pi - 0  =   pi - math.atan(abs(y) / abs(x))
        """ 

    def get_dist(self, x, y):
        return sqrt(pow(abs(x), 2) + pow(abs(y), 2))
        
        
    def get_angle(self, x, y):
    
        if  (x >= 0 and y >= 0): # case 1: +0
            return  atan(abs(y) / abs(x))
            
        elif(x >= 0 and y <  0): # case 2: -0
            return -atan(abs(y) / abs(x))
            
        elif(x <  0 and y <  0): # case 3: -(pi-0)
            return -(pi - atan(abs(y) / abs(x)))
            
        elif(x <  0 and y >= 0): # case 4:  (pi-0)
            return   pi - atan(abs(y) / abs(x))
        

    def move2goal(self):
    
        goal_x    = input("Set your x goal: ")
        goal_y    = input("Set your y goal: ")
        
        dist      = self.get_dist( goal_x, goal_y)
        angle     = self.get_angle(goal_x, goal_y)
        
        if(angle < 0):
            angle = -angle
            wise  = -1
        else:
            wise  =  1
                
        time2turn = angle / self.ang_z
        time2go   = dist  / self.lin_x
        
        twist = Twist()
            
        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.angular.z = 0
        self.pub.publish(twist)
        
        twist.linear.x = self.lin_x
        time2end = rospy.Time.now() + rospy.Duration(time2go)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.linear.x = 0
        self.pub.publish(twist)
        
        wise = -wise
        
        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.angular.z = 0
        self.pub.publish(twist)
        
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:   pass
