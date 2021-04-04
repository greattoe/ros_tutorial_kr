#!/usr/bin/env python

import rospy
import geometry_msgs.msg

class MoveTurtle():

    def __init__(self):
        rospy.init_node("move_turtle")
        self.pub= rospy.Publisher("turtle1/cmd_vel",geometry_msgs.msg.Twist,queue_size=10)
        self.tw = geometry_msgs.msg.Twist()
        self.move_turtle()
   
    def move_turtle(self):
        self.tw.linear.x = self.tw.angular.z = 0.25
        self.pub.publish(self.tw)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            MoveTurtle()
            
    except KeyboardInterrupt:
        print "Program terminated!"
