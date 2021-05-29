## Turtlebot3/ Tutorials/ Straight by 2D Pose



---

## 2D Pose 값을 이용한 회전 및 직선이동 라이브러리

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [2D Pose 토픽에 의한 좌, 우 회전](./tb3_7_Rotate_by_Pose2D.md)

**다음 튜토리얼 :** 

**튜토리얼 목록 :** [README.md](../README.md)

------

이 튜토리얼에서는 [6. 2D Pose 토픽에 의한 직선 이동](./tb3_6_Straight_by_Pose2D.md), [7. 2D Pose 토픽에 의한 회전](./tb3_7_Rotate_by_Pose2D.md) 두 튜토리얼에서 작성한 코드를 하나로 합쳐 라이브러리(`tb3_cleaner/src/tb3_cleaner/MoveTB3.py`)로 등록하고, 이를 테스트하는 `node` `test_lib_tb3move.py` 를 작성한다.  



**필요한 작업** ( `setup.py` 와 작성할 내용에 대한 참고자료: [catkin + rospy 사용법(2/2) (setup.py)](.rospy_0_How2UsePythonWithCatkin_2.md) )

1. `tb3_cleaner/src/tb3_cleaner` 폴더 및 `tb3_cleaner/src/tb3_cleaner/__init__.py` 파일 생성
2. `tb3_cleaner/setup.py` 작성
3. `tb3_cleaner/CMakeList.txt` 편집
4. `tb3_cleaner/src/tb3_cleaner/MoveTB3.py` 작성
5. `tb3_cleaner/scripts/test_lib_tb3move.py` 작성



`tb3_cleaner` 패키지 폴더로 경로를 변경한다.

```bash
$ roscd tb3_cleaner
```

`tb3_cleaner/src/tb3_cleaner` 폴더 생성

```bash
$ mkdir ./src/tb3_cleaner
```

`tb3_cleaner/src/tb3_cleaner/__init__.py` 파일 생성

```bash
$ touch ./src/tb3_cleaner/__init__.py
```

`tb3_cleaner/src/tb3_cleaner/MoveTB3.py`  파일 생성

```bash
$ touch ./src/tb3_cleaner/MoveTB3.py
```

`tb3_cleaner`  폴더로 경로 변경

```bash
$ roscd tb3_cleaner
```

`tb3_cleaner/setup.py`  파일 생성

```bash
$ touch setup.py
```



`tb3_cleaner/setup.py`  파일 편집

```bash
$ gedit setup.py &
```

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['tb3_cleaner'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```



`tb3_cleaner/CMakeList.txt`  파일 편집

```bash
$ gedit CMakeList.txt &
```

```bash
cmake_minimum_required(VERSION 3.0.2)
project(tb3_cleaner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()  # <------------------------ uncomment this line

################################################
## Declare ROS messages, services and actions ##
################################################
# 이하 생략.
```



`tb3_cleaner/src/tb3_cleaner/MoveTB3.py` 파일 편집

```bash
$ gedit ./src/tb3_cleaner/MoveTB3.py &
```

```python
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import degrees, sqrt, pi

# define pi, 2pi, pi/2
_2PI = 2.0 * pi
_PI  = 1.0 * pi
_R   = 0.5 * pi

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.125
ANG_SPD = MAX_ANG_SPEED * 0.125

class MoveTB3:

    def __init__(self):  
        rospy.Subscriber('/tb3pose', Pose, self.get_pose)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)   
        self.tb3pose  = self.org = Pose()
                 
    def get_pose(self, msg):
        self.tb3pose = msg
            
    def update_org(self):
        self.org = self.tb3pose
                
    def elapsed_dist(self):
        return sqrt(pow((self.tb3pose.x - self.org.x), 2) + pow((self.tb3pose.y - self.org.y), 2))
            
    def straight(self, distance): 
        tw = Twist()    
        self.update_org()
        print "start from (%s, %s)" %(round(self.org.x, 2), round(self.org.y, 2))       
        
        if distance >= 0:   # +distance
            tw.linear.x =  LIN_SPD
        else:               # -distance
            tw.linear.x = -LIN_SPD
                        
        self.pub.publish(tw)        
        while self.elapsed_dist() < abs(distance):  pass
            #print "%s(m) of %s(m)" %(round(self.elapsed_dist(),2), round(abs(distance),2))
        
        tw.linear.x = 0;    self.pub.publish(tw)
        print "stop to (%s, %s)." %(round(self.tb3pose.x, 2), round(self.tb3pose.y, 2))   
        
    def elapsed_angle(self):
        return abs(self.tb3pose.theta - self.org.theta)
        
    def rotate(self, angle):
        tw = Twist()
        self.update_org()
        print "start from: %s" %(round(degrees(self.org.theta), 2))
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD;
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD;
            
        self.pub.publish(tw)
        while self.elapsed_angle() < abs(angle):    pass
            # print "%s of %s" %(round(degrees(self.elapsed_angle()),2) ,round(degrees(abs(angle)),2))
            
        tw.angular.z =  0;  self.pub.publish(tw)
        print "stop to   : %s" %(round(degrees(self.tb3pose.theta), 2))
```



라이브러리 `MoveTB3.py` 테스트 코드 `tb3_cleaner/scripts/test_lib_tb3move.py` 작성

 `tb3_cleaner/scripts` 폴더로 경로 변경

```bash
$ roscd tb3_cleaner/scripts
```

`tb3_cleaner/scripts/test_lib_tb3move.py` 파일 생성

```bash
$ touch test_lib_tb3move.py
```

`tb3_cleaner/scripts/test_lib_tb3move.py` 파일에 실행속성 부여

```bash
$ chmod +x test_lib_tb3move.py
```

`tb3_cleaner/scripts/test_lib_tb3move.py` 파일 편집

```bash
$ gedit test_lib_tb3move.py &
```

```python
#!/usr/bin/env python
'''
################################################################################
#  Be sure the topic("/tb3pose") has to be published before start this code!!! #
################################################################################
'''
import rospy
from tb3_cleaner.MoveTB3 import MoveTB3 # <---- import like this
'''  ----------- -------        -------
          ^         ^              ^
src/tb3_cleaner > MoveTB3.py > class MoveTB3:
'''
from math import radians

if __name__ == '__main__':

    try:
        rospy.init_node('rotate_by_pose', anonymous = True)        
        tb3 = TB3Move()
        angle = radians(input("input angle to rotate(deg): "))
        tb3.rotate(angle)
        dist = float(input("input distance to stright(m): "))
        tb3.straight(dist)
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

```



패키지 빌드 `setup.py` 추가 및 `CMakeList.txt` 수정 등의 이유로 패키지를 다시 빌드한다.

```sh
$ cd ~/catkin_ws && catkin_make
```

빌드 결과가 반영된 `~/catkin_ws/devel/setup.bash` 파일의 정보를 반영한다.

```sh
$ source ./devel/setup.bash
```



`roscore` 실행

```sh
$ roscore
```

`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 Turtlebot3 의 라즈베리파이로 ssh 를 통해 원격 연결한다.

```sh
$ ssh pi@xxx.xxx.xxx.xxx
```

라즈베리파이에서 ```turtlebot3_bringup``` 패키지의 `turtlebot3_robot.launch` 파일을 실행한다.

```sh
pi@raspberrypi:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 이전 튜토리얼에서 작성한  `pub_tb3_pose2d.py` 를 실행한다. 

```sh
$ rosrun turtlesim_cleaner pub_tb3_pose2d.py
```

`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 이 튜토리얼에서 작성한  `test_lib_tb3move.py` 를 실행한다.

```sh
$ rosrun tb3_cleaner test_lib_tb3move.py
input angle to rotate(deg): -90
start from: -248.35
stop to   : -338.7
input distance to straight(m): -0.25
start from (0.13, -0.28)
stop to    (0.26, -0.07)
```



[튜토리얼 목록 열기](../README.md)


