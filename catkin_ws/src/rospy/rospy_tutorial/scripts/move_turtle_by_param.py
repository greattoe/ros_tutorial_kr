#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
from rospy_tutorial.GetChar import GetChar
        
def move_turtle():
    pb = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    tw = Twist()
    tw.linear.x  = 0.50
    tw.angular.z = 0.25        
    pb.publish(tw)
    
if __name__ == '__main__':
    try:
        kb = GetChar()
        rospy.init_node('move_by_param') 
        print "type 'Q' to quit."

        while not rospy.is_shutdown():
            param = rospy.get_param("/turtle1/go_turtle")
            
            if param is True:
                move_turtle()                
            else:
                pass

    except rospy.ROSInterruptException:   pass
