#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('tb3_cleaner', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    print("input speed within 0.0 ~ 0.22")
    speed     = input("Input your speed: ")
    print("input distance within 0.0 ~ 2.0")
    distance  = input("Type your distance: ")
    print("direction foward:1, backward:0")
    isForward = input("Forward?: ") # True or False(1 or 0)

    if(isForward):
        msg.linear.x =  abs(speed)
    else:
        msg.linear.x = -abs(speed)
        
    msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = msg.angular.z = 0

    while not rospy.is_shutdown():
        
        duration = distance / speed
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        pub.publish(msg)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):
            pass    
        
        msg.linear.x = 0
        pub.publish(msg)

if __name__ == '__main__':
    try:
        move()
        rospy.spin()
    except rospy.ROSInterruptException: pass
