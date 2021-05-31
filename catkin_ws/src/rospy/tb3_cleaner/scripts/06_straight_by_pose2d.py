#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt

MAX_LIN_SPEED =  0.22
LIN_SPD = MAX_LIN_SPEED * 0.125

class TB3Move:

    def __init__(self):    
        rospy.init_node('rotate_by_pose', anonymous = True)
        rospy.Subscriber('/tb3pose', Pose, self.get_pose_cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)        
        
        self.tb3pose  = Pose()  # for subscribe
        self.org      = Pose()  # for store starting point
        
    def get_pose_cb(self, msg):
        # callback function to subscribe "/tb3pose" topic
        self.tb3pose = msg
        
    def update_org(self):
        # save current tb3pose.x, y to org.x, y when called this function
        self.org = self.tb3pose
        
    def elapsed_dist(self):
        # calcurate and return elapsed distance
        return sqrt(pow((self.tb3pose.x - self.org.x), 2) + pow((self.tb3pose.y - self.org.y), 2))
    
    def straight(self, distance):
        # forward or backward until elaped distance is equal to target distance
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
        
        self.update_org()
        print "start from (%s, %s)." %(round(self.org.x, 2), round(self.org.y, 2))
        self.pub.publish(tw)    # start move
        
        while self.elapsed_dist() < abs(distance):  pass
        # wait until elapsed distance == target distance
            # print "%s(m) of %s(m)" %(round(self.elapsed_dist(),2), round(abs(distance),2))
        
        tw.linear.x = 0;    self.pub.publish(tw) # stop move
        print "stop to (%s, %s)." %(round(self.tb3pose.x, 2), round(self.tb3pose.y, 2))

if __name__ == '__main__':
    try:
        tb3 = TB3Move()
        
        dist = float(input("input distance to straight(m): "))
        tb3.straight(dist)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
