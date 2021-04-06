#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from my_lib.GetChar import GetChar  # <----- this works by 'setup.py'

msg = """
---------------------------------------
              (forward)
                 'w'

  (left)'a'      's'       'd'(right)
              (backward)
---------------------------------------
type 'Q' for quit program...
---------------------------------------
"""

if __name__ == '__main__':

    rospy.init_node('remote_ctrl_turtle')

    pub  = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    tw   = Twist()
    rate = rospy.Rate(10)
    kb   = GetChar()

    tw.linear.x  = tw.linear.y  = tw.linear.z  = 0.0
    tw.angular.x = tw.angular.y = tw.angular.z = 0.0

    count = ch = 0

    print msg

    while not rospy.is_shutdown():
        ch = kb.getch()

        if   ch == 'w':
            tw.linear.x  =  2.0;    print "forward"
        elif ch == 's':
            tw.linear.x  = -2.0;    print "backward"
        elif ch == 'a':
            tw.angular.z =  2.0;    print "turn left"
        elif ch == 'd':
            tw.angular.z = -2.0;    print "turn right"
        elif ch == 'Q':             break
        else:                       pass

        pub.publish(tw)
        tw.linear.x  =  tw.angular.z = 0.0

        count = count + 1
        if count == 15:
            count = 0;    print msg

        rate.sleep()