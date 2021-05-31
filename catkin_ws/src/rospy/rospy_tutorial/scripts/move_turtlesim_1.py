#!/usr/bin/env python

import rospy
import geometry_msgs.msg


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
    		rospy.init_node("move_turtle")
    		pub = rospy.Publisher("turtle1/cmd_vel",geometry_msgs.msg.Twist,queue_size=10)
    		tw  = geometry_msgs.msg.Twist()
    		tw.linear.x = tw.angular.z = 0.25
    		pub.publish(tw)
            
    except rospy.ROSInterruptException:
        print "Program terminated!"
