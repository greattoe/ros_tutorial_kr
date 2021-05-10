## bebop_autonomy / move_by_odometry



---

## Odometry 에 의한 Bebop 이동 라이브러리 작성

**튜토리얼 레벨 :**  Intermediate(중급)(수정)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [링크 수정 필요]()

**이전 튜토리얼 :** [링크 수정 필요]()

**자료원본(출처) :** <https://bebop-autonomy.readthedocs.io/en/latest/>

**목록보기:** [README.md](../README.md)

---

Odometry 토픽의 pose.pose.positon.x, y, z 값을 참조한 x(전, 후진), y(좌, 우로 이동), z(상승, 하강) 축 방향 이동과 pose.pose.orientation.x, y, z, w 를 이용한 z축에 대한 회전( yaw )을 구현하고, 라이브러리로 만든다. 



#### 1. 사용자 정의 메세지 `Pos_XYZ_th.msg` 작성

Odometry 토픽이 발행하는 정보 중 pose.pose.position.x, y, z 거리값과 pose.pose.orientation.x, y, z, w 를 이용하여 구한 yaw 회전각 theta 로 이루어진 사용자 정의 메세지 `Pos_XYZ_th.msg` 를 만들어 사용하자. 

작업경로를 `~/catkin_ws/src/bb2_pkg` 로 변경

```bash
$ roscd bb2_pkg
```

`msg` 폴더 생성

```bash
$ mkdir msg
```

`Pos_XYZ_th.msg` 파일 생성

```bash
$ touch ./msg/Pos_XYZ_th.msg
```

`Pos_XYZ_th.msg` 파일 편집

```bash
$ gedit ./msg/Pos_XYZ_th.msg &
```

```
float32 x
float32 y
float32 z
float32 th
```



#### 2. `package.xml` 편집

사용자 정의 메세지, 서비스 등을 사용하려면 `package.xml` 를 편집해줘야 한다. 내용 중  `<build_depend>` 에  `message_generation` 을,  `<exec_depend>` 에  `message_runtime` 을 아래와 같이 추가해 줘야한다. 

`Package.xml` 파일 편집

```bash
$ gedit ./package.xml &
```

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend> <!--------- add this line ------>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend> <!--------------- add this line ------>
```



#### 3. `CMakeList.txt` 편집

사용자 정의 메세지를 사용하려면 `package.xml` 에 `<build_depend>` 에 `message_generation` 과 `<exec_depend>` 에 `message_runtime` 을 추가해 줘야한다. 

`Package.xml` 파일 편집

```bash
$ gedit ./package.xml &
```

```makefile
 .
 .
 .
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation # <----------- add this line
)
 .
 .
 .
## Generate messages in the 'msg' folder
add_message_files(  # <---+
   FILES            #     |
   Pos_XYZ_th.msg   #     +---- uncomment & input msg filename
#   Message2.msg    #     |
)                   # <---+
 .
 .
 .
## Generate added messages and services with any dependencies listed here
generate_messages(  # <---+
   DEPENDENCIES     #     +---- uncomment
   std_msgs         #     |
)                   # <---+
 .
 .
 .
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bb2_pkg
  CATKIN_DEPENDS message_runtime geometry_msgs rospy std_msgs # <---------- uncomment &
#  DEPENDS system_lib                                         # insert 'message_runtime'
)
 .
 .
 .
```



#### 4. 빌드

```bash
$ cd ~/catkin_ws && catkin_make
```



#### 5. 사용자 정의 라이브러리 `MoveBB2.py` 작성

`MoveBB2.py` 파일 생성. 

```bash
$ touch ./src/bb2_pkg/MoveBB2.py
```

`MoveBB2.py` 파일 편집. 

```bash
$ gedit ./src/bb2_pkg/MoveBB2.py &
```

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from math import radians, degrees, pi, sqrt
from tf.transformations import euler_from_quaternion
from bb2_pkg.msg import Pos_XYZ_th

LIN_SPD = 0.25
ANG_SPD = 0.50

class MoveBB2:

    def __init__(self):
        rospy.Subscriber('/bebop/odom', Odometry, self.get_odom_cb )
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        
        self.empty_msg = Empty()
        self.xyzth_now = self.xyzth_org = Pos_XYZ_th()
        self.theta_prv = self.theta_sum = 0.0
        self.do_print  = False
        
        
    def get_odom_cb(self, msg):
        self.xyzth_now.x  = msg.pose.pose.position.x
        self.xyzth_now.y  = msg.pose.pose.position.y
        self.xyzth_now.z  = msg.pose.pose.position.z
        
        theta = self.get_theta(msg)
        
        if   (theta - self.theta_prv) >  5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (theta - self.theta_prv) - 2 * pi            
        elif (theta - self.theta_prv) < -5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (theta - self.theta_prv) + 2 * pi
        else:
            d_theta = (theta - self.theta_prv)

        self.theta_sum    = self.theta_sum + d_theta
        self.theta_prv    = self.xyzth_now.th
        self.xyzth_now.th = self.theta_sum
        
        if self.do_print is True:
            self.print_xyzth(self.xyzth_now)
         
        
    def get_theta(slef, dat):
        q = (dat.pose.pose.orientation.x, dat.pose.pose.orientation.y, 
             dat.pose.pose.orientation.z, dat.pose.pose.orientation.w)
                                            # quart[0] = roll
        quart = euler_from_quaternion(q)    # quart[1] = pitch
        theta = quart[2]                    # quart[2] = yaw <----
        
    	# make theta within from 0 to 360 degree
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        return theta
        
        
    def print_xyzth(self, msg):
        print "x = %s, y = %s, z = %s, th = %s" %(msg.x, msg.y, msg.z, degrees(msg.th))
        
        
    def update_org(self):
        self.xyzth_org = self.xyzth_now
        
        
    def elapsed_dist(self):
        return sqrt(pow((self.xyzth_now.x - self.xyzth_org.x), 2) + pow((self.xyzth_now.y - self.xyzth_org.y), 2))
        
        
    def elapsed_angle(self):
        return abs(self.xyzth_now.th - self.xyzth_org.th)
        
        
    def elapsed_height(self):
        return abs(self.xyzth_now.z - self.xyzth_org.z)
        
        
    def move_x(self, distance, tolerance):
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_y(self, distance, tolerance):
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.y =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.y = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - tolerance:
            self.pub0.publish(tw)
        
        tw.linear.y = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_z(self, height, tolerance):
        tw = Twist()
        
        if height >= 0:   # distance(+): forward
            tw.linear.z =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.z = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at %s" %(round(self.xyzth_org.z, 2))
        
        while self.elapsed_height() < abs(height) - tolerance:
            self.pub0.publish(tw)
        
        tw.linear.z = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at %s" %(round(self.xyzth_now.z, 2))
        
        
    def rotate(self, angle, tolerance):
        tw = Twist()
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD
        
        self.update_org()
        print "start from: %s" %(round(degrees(self.xyzth_org.th), 2))
        
        while self.elapsed_angle() < abs(angle) - abs(angle) * tolerance:
            self.pub0.publish(tw)
            
        tw.angular.z =  0;  self.pub0.publish(tw)
        rospy.sleep(1.5)
        print "stop to   : %s" %(round(degrees(self.theta_now), 2))
        
    
    def takeoff(self):
        self.pub1.publish(self.empty_msg);  print "takeoff"
        
    
    def landing(self):
        self.pub2.publish(self.empty_msg);  print "landing"
        
    
    def emergency(self):
        self.pub3.publish(self.empty_msg);  print "emergency"
        
```



#### 6. 이 라이브러리를 사용한 예제

`bb2_pkg` 패키지에 노드가 구동되면 이륙하여, 회전할 각도 및 x, y, z 방향 이동거리를 입력받아 이동 후, 착륙하는 노드 `bebop_move_by_odom.py` 를  작성해보자. 

`~/catkin_ws/src/bb2_pkg/scripts` 로 작업경로 변경. 

```bash
$ roscd bb2_pkg/scripts
```

`bebop_move_by_odom.py` 파일 생성.  

```bash
$ touch bebop_move_by_odom.py
```

`bebop_move_by_odom.py` 파일에 실행속성 부여.  

```bash
$ chmod +x bebop_move_by_odom.py
```

`bebop_move_by_odom.py` 파일 편집.  

```bash
$ gedit bebop_move_by_odom.py &
```

```python
#!/usr/bin/env python

import rospy
from bb2_pkg.MoveBB2 import MoveBB2
from math import radians, degrees

if __name__ == '__main__':

    rospy.init_node('bb2_sub_odom', anonymous = True)
    
    try:
        bb2 = MoveBB2()
        
        angle  = radians(input("input angle to rotate in degree: "))
        dist_x = float(input("input distance to move to x in meter: "))
        dist_y = float(input("input distance to move to y in meter: "))
        dist_z = float(input("input distance to move to z in meter: "))
        
        bb2.takeoff();	rospy.sleep(2.0)
        bb2.move_z(dist_z, 0.25)
        bb2.rotate(angle,  0.25)
        bb2.move_x(dist_x, 0.25)
        bb2.move_y(dist_z, 0.25)
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
```















[튜토리얼 목록 열기](../README.md)


