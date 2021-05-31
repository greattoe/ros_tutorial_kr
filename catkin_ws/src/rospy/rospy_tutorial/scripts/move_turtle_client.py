#!/usr/bin/env python

import sys
import rospy
from my_pkg.srv import *

def move_turtle_client(angle, distance):
    rospy.wait_for_service('turtlesim_svc')

    try:
        svc = rospy.ServiceProxy('turtlesim_svc', AngDist)
        res = svc(x, y)
        return res.result
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [distance] [angle]" %sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    move_turtle_client(x, y)
    print "Requesting rotate %s(deg) & move %s(m)"%(x, y)
    #print "Request is %s"%(move_turtle_client(x, y))

