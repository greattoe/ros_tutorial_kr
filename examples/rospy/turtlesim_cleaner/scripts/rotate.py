#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed     = input("Input your speed (degrees/sec):")
    angle     = input("Type your distance (degrees):")
    clockwise = input("Clockwise?: ") # True or false

    # Converting from angles to radians
    angular_speed  = speed*2*PI/360
    relative_angle = angle*2*PI/360

    # We wont use linear components
    msg.linear.x  = msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        msg.angular.z = -abs(angular_speed)
    else:
        msg.angular.z =  abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        pub.publish(msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    # Forcing our robot to stop
    msg.angular.z = 0
    pub.publish(msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        rotate()
    except rospy.ROSInterruptException:
        pass
