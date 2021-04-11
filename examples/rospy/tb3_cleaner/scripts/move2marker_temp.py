#!/usr/bin/env python

import rospy
from math import degrees, radians, sin, cos, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = -1

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# set parking zone(kind of tolerlance)
MAX_DIST = 0.15
MIN_DIST = 0.10


class Move2Marker:

    def __init__(self):
    
        rospy.init_node('move_to_marker', anonymous = True)       
        rospy.Subscriber('/odom', Odometry, self.get_tb3_theta)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker_info)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.ang_speed = MAX_ANG_SPEED * 0.100
        self.lin_speed = MAX_LIN_SPEED * 0.125
        
        self.tw = Twist()
        self.tw.angular.z = self.ang_speed
        
        self.curr_th   = 0
        self.prev_th   = 0
        self.total_th  = 0
        
        self.find_th   = 0
        self.lost_th   = 0
        
        self.dist_x    = 0
        self.dist_y    = 0
        self.theta     = 0 
        
        self.pos_x     = 0
        
        self.x         = 0
        self.y         = 0
        self.th        = 0
        self.dist      = 0
        
        self.wise      = 0
        self.count     = 0
        
        self.right_case  = False
        self.left_case   = False
        self.center_case = False
        
        self.find_first_time_passed  = False
        self.lost_first_time_passed  = False
        
        self.step1_align2marker_end  = False
        self.step2_1st_rotation_end  = False
        self.step3_move_2_front_end  = False
        self.step4_2nd_rotation_end  = False
        self.step5_move_2_marker_end = False
        
        
    def get_marker_info(self, msg):
        
        self.pub.publish(self.tw)
        
        if len(msg.markers) > 0:
          
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                
                    print "found target marker"
                    
                    self.dist_x, self.dist_y, theta = self.get_ar_pose(msg)
                                      
                    self.dist  = msg.pose.pose.position.z
                    self.pos_x = msg.pose.pose.position.x
                    
                    if  (theta >  5.0):
                        self.theta = theta - 2 * pi            
                    elif(theta < -5.0):
                        self.theta = theta + 2 * pi
                    else:
                        self.theta = theta
                                        
                
                    if  self.find_first_time_passed == False:
                        self.find_th  = self.curr_th
                        print "## get theta to start recognize marker."
                        self.find_first_time_passed = True
                        
                    if self.step1_align2marker_end == True and self.step2_1st_rotation_end == False:
                        
                        self.get_marker_pose()
                        
                        if self.th >= 0:
                            angle =   0.5 * pi - abs(self.th)
                            angle = angle * 0.7
                        else:
                            angle = -(0.5 * pi - abs(self.th))
                            angle = angle
                        self.rotate(angle)
                        self.step2_1st_rotation_end = True
                    
                    if self.step4_2nd_rotation_end == True and self.step5_move_2_marker_end == False:
                        self.approach()
                    
                else:
                    print "id mismatch"
                    
                
        else: # lost marker            
                
            print "lost marker"
        
            if self.find_first_time_passed == True and self.lost_first_time_passed == False:
                self.lost_th = self.curr_th
                print "## get theta to end recognize marker."
                self.lost_first_time_passed = True
            
            if self.lost_first_time_passed == True and self.step1_align2marker_end == False:
                self.align2marker()                    
                self.step1_align2marker_end = True
                
            if self.step2_1st_rotation_end == True and self.step3_move_2_front_end == False:
                if self.y < 0:
                    self.move(abs(self.y) * 1.25)
                else:
                    self.move(abs(self.y) * 0.95)
                self.step3_move_2_front_end = True
            
            if self.step3_move_2_front_end == True and self.step4_2nd_rotation_end == False:
                if self.y < 0:
                    self.rotate( radians(90))
                else:
                    self.rotate(-radians(90))
                self.step4_2nd_rotation_end = True
        
        """
                  y                        z 
                  ^  x                     ^
          marker  | /                      | robot 
        (on wall) |/                       | 
                  +------> z      x <------+  
                                          /
                                         /
                                        y        
        
          orientation x,y,z,w --+
                                +--> 4   +-------------------------+
        input orientaion of marker ----->|                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <--------|                         |
                                 +-- 3   +-------------------------+
                 r,p,y angle <---+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
                
    
    def get_ar_pose(self, msg):
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        quart = euler_from_quaternion(q)
        theta = quart[1]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        
        dist_x =  msg.pose.pose.position.z * cos(theta)
        dist_y =  msg.pose.pose.position.z * sin(theta)

        return dist_x, dist_y, theta 
                    
    
    def get_tb3_theta(self, msg):
        
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                                            # quart[0] = roll
        quart = euler_from_quaternion(q)    # quart[1] = pitch
        theta = quart[2]                    # quart[2] = yaw <----
        
        if   (theta - self.prev_th) >  5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (theta - self.prev_th) - 2 * pi            
        elif (theta - self.prev_th) < -5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (theta - self.prev_th) + 2 * pi
        else:
            d_theta = (theta - self.prev_th)

        self.total_th = self.total_th + d_theta
        self.prev_th = theta
               
        self.curr_th   = self.total_th          
    
    
    def align2marker(self):
        print "## align"
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        
        current = self.curr_th        
        target = current - abs(self.find_th - self.lost_th) * 0.7
        
        self.tw.angular.z = -MAX_ANG_SPEED * 0.125        
        self.pub.publish(self.tw)
        
        while self.curr_th > target:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        print "align complete."
        
        
    def get_marker_pose(self):
        self.x = self.dist_x; self.y = self.dist_y; self.th = self.curr_th
        print "dist_x = %f, dist_y = %f, theta = %f" %(self.x, self.y, degrees(self.th))
                
    
    def rotate(self, angle):
    
        self.count = self.count + 1
        print "## rotate %d" %(self.count)
        
        current_angle = self.curr_th
        target_angle  = current_angle + angle
        print "current = %f, target = %f" %(degrees(target_angle) ,degrees(current_angle))
        
        if angle < 0:
            self.tw.angular.z = MAX_ANG_SPEED * 0.25 * -1
            self.pub.publish(self.tw)
            while target_angle < self.curr_th:  pass
        else:
            self.tw.angular.z = MAX_ANG_SPEED * 0.25
            self.pub.publish(self.tw)
            while target_angle > self.curr_th:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        print degrees(self.curr_th)
        
        
    def move(self, dist):
        speed = MAX_LIN_SPEED * 0.5
        duration = 3.0 * dist / speed
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        print "## move for %f(sec)" %(duration)
        
        self.tw.linear.x = self.lin_speed       
        self.pub.publish(self.tw)
        
        while time2end > rospy.Time.now():  pass
        
        self.tw.linear.x = 0       
        self.pub.publish(self.tw) 
        
        
    def ctrl_by_pos_x(self, ang_z):
        if abs(self.pos_x) > 0.02:
            if self.pos_x >= 0:
                self.tw.angular.z = radians(-ang_z)
            else:
                self.tw.angular.z = radians( ang_z)
        else:
            self.tw.angular.z = 0
                
        
        
    def approach(self):
        print "## approach"
        '''
        # marker
        #   |        min    max
        #   |   --->  |      |  <---
        #   +---------+------+----------------
        #    backward   stop   forward 
        '''
        speed = MAX_LIN_SPEED * 0.125
        
        if   self.dist > MAX_DIST:
            if self.dist - MAX_DIST > 0.20:
                self.ctrl_by_pos_x(3.25)
                self.tw.linear.x =  speed * 2.0
            else:
                self.tw.linear.x =  speed * 0.55
                self.ctrl_by_pos_x(2.75)
        elif self.dist < MIN_DIST:
            self.tw.linear.x =  speed * 0.45 * -1
            
        else:
            self.tw.linear.x = 0;   self.pub.publish(self.tw) 
        
        self.pub.publish(self.tw)        
        print "distance to marker = %f(cm)" %(self.dist * 100)
          

if __name__ == '__main__':
    try:
        TARGET_ID = int(input("input marker id: "))
        Move2Marker()        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
