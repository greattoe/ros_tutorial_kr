#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
        
def move_turtle(self):
    pb = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    tw = Twist()
    tw.linear.x  = 0.50
    tw.angular.z = 0.25        
    pb.publish(tw)
    
if __name__ == '__main__':
    try:   
        rospy.init_node('move_by_param')
        tp2 = TestParam1()
        
        while not rospy.is_shutdown():        
            param = rospy.get_param("/turtle1/go_turtle")
            
            if param is True:
                move_turtle()                
            else:   pass

    except rospy.ROSInterruptException:   pass
