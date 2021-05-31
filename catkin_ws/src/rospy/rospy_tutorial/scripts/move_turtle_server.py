#!/usr/bin/env python

from my_pkg.srv import AngDist, AngDistResponse
import rospy
from geometry_msgs.msg import Twist
from math import radians


LIN_X = ANG_Z = 1.5

def svc_cb(req):
    res_rot = rotate(radians(req.angle))
    res_mov = move(req.distance)
    return AngDistResponse(res_rot and res_mov)

def turtlesim_svc_svr():
    rospy.init_node('turtlesim_svc_node')
    svc = rospy.Service('turtlesim_svc', AngDist, svc_cb)
    print "ready~"
    rospy.spin()


def move(distance):
    p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    t = Twist()

    t.linear.x  = t.linear.y  = t.linear.z  = 0
    t.angular.x = t.angular.y = t.angular.z = 0

    speed = LIN_X    

    if distance < 0:

        speed = speed * -1
        t.linear.x = speed

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance > distance):
            p.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
    else:

        t.linear.x = speed

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
            p.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_distance= speed * (t1 - t0)
    
    print "end move"
    t.linear.x = 0
    p.publish(t)
    return True


def rotate(angle):

    p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    t = Twist()

    speed  = ANG_Z

    if angle < 0:
        speed = speed * -1
        t.angular.z = speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle > angle):
            p.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_angle = speed * (t1 - t0)    
    else:

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < angle):
            p.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_angle = speed * (t1 - t0)
    
    print "end rotate"
    t.angular.z = 0
    p.publish(t)        
    return True


if __name__ == "__main__":
    turtlesim_svc_svr()

