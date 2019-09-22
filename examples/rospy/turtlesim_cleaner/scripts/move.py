#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node name 'robot_cleaner'
    rospy.init_node('robot_cleaner', anonymous=True)
    # declair publisher name:pub, topic:'/turtle1/cmd_vel', type:Twist
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # declair Twist type object name 'msg'
    msg = Twist()

    # Receiveing the user's input
    print("Let's move your robot")
    speed     = input("Input your speed: ")
    distance  = input("Type your distance: ")
    isForward = input("Foward?: ") # True or False(1 or 0)

    # Checking if the movement is forward or backwards
    if(isForward):
        msg.linear.x =  abs(speed)
    else:
        msg.linear.x = -abs(speed)
        
    # Since we are moving just in x-axis
    msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = msg.angular.z = 0

    while not rospy.is_shutdown():

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        # Loop to move the turtle in an specified distance
        while(current_distance < distance):
            # Publish the velocity
            pub.publish(msg)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        # After the loop, stops the robot
        msg.linear.x = 0
        # Force the robot to stop
        pub.publish(msg)

if __name__ == '__main__':
    try:
        # Testing our function
        move()
    except rospy.ROSInterruptException: pass
