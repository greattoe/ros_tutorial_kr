## Turtlebot3/ Tutorials/ Go to Goal



---

## Turtlebot 목표지점 이동

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [좌 / 우 회전](./mv_tb3_2_RotateLeftRight.md)

**튜토리얼 목록 :** [README.md](../README.md)

------

이 튜토리얼에서는 현재 로봇의 위치를 기준(원점: 0, 0)으로 목표지점의 x, y 사용자 입력을 받아, Turtlebot3 로봇을 ( x, y )위치로 이동하는  노드를 작성할 것이다. 이 코드는 [turtlsim_cleaner](https://github.com/clebercoutof/turtlesim_cleaner) 코드를 참고하여 만들었음을 밝힌다.



### 1. 준비작업

앞서 [이 전 튜토리얼](./mv_tutle_2_RotateLeftRight.md)에서 사용한  `tf3_cleaner` 패키지의 `scripts` 폴더로 경로를 변경한다.

```
user@computer:~$ cd ~/catkin_ws/src/tf3_cleaner/scripts
```

`go2goal.py` 파일을 만들고 실행 속성을 부여한다. 

```
user@computer:~/catkin_ws/src/tf3_cleaner/scripts$ touch go2goal.py
user@computer:~/catkin_ws/src/tf3_cleaner/scripts$ chmod +x go2goal.py
```



### 2. 작성할 코드에 대하여

#### 2.1 TurtleBot 클라스

이 튜토리얼에서 작성할 TurtleBot 클라스는  `__init__()` , `get_dist()` ,  `get_angle()` ,  `move2goal()` 로 이루어져 있다.

#### 2.2 `__init__()` 

노드 초기화, 퍼블리셔 핸들 생성, 초기 선 속도와 각 속도 설정, 필요한 멤버 변수 선언 및 초기화 수행

#### 2.3 `get_dist()`

유클리드 거리계산 방법으로 로봇의 현 위치를 기준으로 목표지점( x, y )까지의 거리 계산 및 반환.

#### 2.4 `get_angle()`

목표 지점( x, y )까지 직선 주행을 위한 회전 방향 및 회전 량 계산 및 반환. 반환 값의 부호가 '+' 일 경우 ccw( 반 시계방향 ) 회전, '-' 인 경우는 cw( 시계 방향 ) 회전을 의미한다.

#### 2.5 `move2goal()`

사용자로 부터 입력받은 목표 지점의 x, y값을 `get_dist()` 와 `get_angle()` 에 전달하고, 그 반환 값을 적절히 이용하여 로봇을 목표지점으로 이동시키는 함수.



### 3. 코드 작성

`~/catkin_ws/src/tb3_cleaner/scripts` 폴더에 다음 내용으로  `go2goal.py` 파일을 작성한다.

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2

"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(radian/sec)
"""
MAX_LIN_X = 0.22
MAX_ANG_Z = 2.82

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2
        
        self.wise  = 1
        
        """        x
                   |
             ------+------
             |\    |    /|                     /|
       case1 |0\   |   /0| case2              / |         1 radian = 57.2958 degree
             |  \  |  /  |                   /30|         1 degree = 0.0174533 radian
       +x,+y |   \0|0/   | +x,-y            /   |
             |    \|/    |               2 /    | sqrt(3)
       y-----+-----+-----+-----           /     |
             |    /|\    |               /      |
       -x,+y |   /0|0\   | -x,-y        /       |
             |  /  |  \  |             /60    90|
       case4 |0/   |   \0| case3      ----------+
             |/    |    \|                 1
             ------+------                            
       
        d = dist  = sqrt(abs(x) + abs(y))
        
        0 = angle = math.atan(abs(y) / abs(x))
        
        case 1:  0 =  math.atan(abs(y) / abs(x))
        case 2: -0 = -math.atan(abs(y) / abs(x))
        case 3: -(180 * 0.0174533 - 0) = -(pi - 0) = -(pi - math.atan(abs(y) / abs(x)))
        case 4:   180 * 0.0174533 - 0  =   pi - 0  =   pi - math.atan(abs(y) / abs(x))
        """ 

    def get_dist(self, x, y):
        return sqrt(pow(abs(x), 2) + pow(abs(y), 2))
        
        
    def get_angle(self, x, y):
    
        if  (x >= 0 and y >= 0): # case 1: +0
            return  atan(abs(y) / abs(x))
            
        elif(x >= 0 and y <  0): # case 2: -0
            return -atan(abs(y) / abs(x))
            
        elif(x <  0 and y <  0): # case 3: -(pi-0)
            return -(pi - atan(abs(y) / abs(x)))
            
        elif(x <  0 and y >= 0): # case 4:  (pi-0)
            return   pi - atan(abs(y) / abs(x))
        

    def move2goal(self):
    
        goal_x    = input("Set your x goal: ")
        goal_y    = input("Set your y goal: ")
        
        dist      = self.get_dist( goal_x, goal_y)
        angle     = self.get_angle(goal_x, goal_y)
        
        if(angle < 0):
            angle = -angle
            wise  = -1
        else:
            wise  =  1
                
        time2turn = angle / self.ang_z
        time2go   = dist  / self.lin_x
        
        twist = Twist()
            
        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.angular.z = 0
        self.pub.publish(twist)
        
        twist.linear.x = self.lin_x
        time2end = rospy.Time.now() + rospy.Duration(time2go)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.linear.x = 0
        self.pub.publish(twist)
        
        wise = -wise
        
        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)
        
        self.pub.publish(twist)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):   pass
    
        twist.angular.z = 0
        self.pub.publish(twist)
        
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:   pass
```



### 4. 실행

`roscore` 실행

```
user@computer:~/catkin_ws$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 Turtlebot3 의 라즈베리파이로 ssh 를 통해 원격 연결한다.

```
user@computer:~$ ssh pi@xxx.xxx.xxx.xxx
```



라즈베리파이에서 ```turtlebot3_bringup``` 패키지의 `turtlebot3_robot.launch` 파일을 실행한다.

```
pi@raspberrypi:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `go2goal.py` 를 실행한다. 

```
user@computer:~$ rosrun turtlesim_cleaner move.py
Set your x goal:
Set your y goal: 
```

Turtlebot3( buger ) 로봇이 위에 입력한 정보와 같이 동작하는 지 확인한다.



[튜토리얼 목록 열기](../README.md)







