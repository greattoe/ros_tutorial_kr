#!/usr/bin/env python

import rospy
from rospy_tutorial.GetChar import GetChar

class SetParam:
    def __init__(self):    
        rospy.init_node('set_go_tutle', anonymous=True)        
        param = rospy.get_param("/turtle1/go_turtle")
        print(param)

if __name__ == '__main__': 
    try:
        SetParam()        
        kb_input = GetChar()        
        print "\nType '1' for 'go', '0' for 'stop', any other key for 'quit'."    
        
        while not rospy.is_shutdown():        
            key = kb_input.getch()
            
            if   key == '1':
                rospy.set_param("/turtle1/go_turtle", True)
                print(rospy.get_param("/turtle1/go_turtle"))            
            elif key == '0':
                rospy.set_param("/turtle1/go_turtle", False)
                print(rospy.get_param("/turtle1/go_turtle"))
            else:
                break
        print "program terminated"           
    except rospy.ROSInterruptException:   pass
