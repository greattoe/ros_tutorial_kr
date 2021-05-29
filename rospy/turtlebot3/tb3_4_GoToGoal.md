## Turtlebot3/ Tutorials/ Go to Goal



---

## Turtlebot 목표지점 이동 2

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [좌 / 우 회전](./mv_tb3_2_RotateLeftRight.md)

**튜토리얼 목록 :** [README.md](../README.md)

------

이 튜토리얼에서는 현재 로봇의 위치를 기준(원점: 0, 0)으로 목표지점의 x, y 사용자 입력을 받아, Turtlebot3 로봇을 ( x, y )위치로 로봇(터틀봇3-burger)을 pose값을 반영하여 이동( [turtlsim 거북이 목표지점 이동](./mv_tutle_3_Go2Goal.md) 의 거북이처럼 이동 )하는  노드를 작성할 것이다. 

이를 위해 `turtlesim` 의 `/turtle1/pose` 에 해당하는 터틀봇3가 발행하는 토픽을 찾아보았다. 주행기록에 해당하는 토픽인`/odom` 토픽 내용 중 해당하는 항목 이 문제는 [터틀봇3의 온라인 메뉴얼](http://emanual.robotis.com/docs/en/platform/turtlebot3/applications/#applications)에 [공개된 코드](https://github.com/ROBOTIS-GIT/turtlebot3_applications.git)를 참고하여 터틀봇3의 odom 토픽 값을 subscribe 한 토픽을 `turtlesim.msg/Pose` 형식으로 변환하여 적용하였다. 그 외에는 [turtlsim 거북이 목표지점 이동](./mv_tutle_3_Go2Goal.md) 튜토리얼의 코드를 거의 그대로 사용했다.



### 1. 준비작업

앞서 [이 전 튜토리얼](./mv_tutle_2_RotateLeftRight.md)에서 사용한  `tf3_cleaner` 패키지의 `scripts` 폴더로 경로를 변경한다.

```
user@computer:~$ cd ~/catkin_ws/src/tf3_cleaner/scripts
```

`gotogoal.py` 파일을 만들고 실행 속성을 부여한다. 

```
user@computer:~/catkin_ws/src/tf3_cleaner/scripts$ touch gotogoal.py
user@computer:~/catkin_ws/src/tf3_cleaner/scripts$ chmod +x gotogoal.py
```



### 2. 작성할 코드에 대하여

#### 2.1 TurtleBot3 클라스

이 튜토리얼에서 작성할 TurtleBot 클라스는  `__init__()` , `get_dist()` ,  `get_angle()` ,  `move2goal()` 로 이루어져 있다.

#####  `__init__()` 

노드 초기화, 퍼블리셔 핸들 생성, 초기 선 속도와 각 속도 설정, 필요한 멤버 변수 선언 및 초기화 수행

#####  `get_odom()`

터틀봇3-buger가 발행하는 `/odom` 토픽 `subscribe` 를 위한 `callback` 함수. `subscribe` 한 `Odometry` 데이터를 `get_pose()` 를 호출하여 전달 후, 그 반환값을 로봇이동에 참조하는 `pose` 값( `self.pos_x_2d`, `self.pos_y_2d`, `self.theta_2d` )으로 저장한다.

#####  `get_pose()`

 위 `get_odom()` 함수로부터 전달받은 Quaternon `/odom` 데이터를 `rospy`  `tf` 모듈이 제공하는 `euler_from_quaternion` 을 이용하여 이차원 `pose` 데이터로 변환하여 반환한다.

#####  `get_dist()`

유클리드 거리계산 방법으로 로봇의 현 위치를 기준으로 목표지점( x, y )까지의 거리 계산 및 반환.

#####  `get_lin_x()`

`get_dist()` 로 부터 반환받은 현재 위치와 목적 지점사이의 거리 x 비례상수를 로봇 이동에 사용될 `/cmd_vel` 토픽의 `linear.x` 값에 반영시킨다.

#####  `get_angle()`

로봇의 현재 `pose`를 기준으로 목표지점까지의 회전각도 계산 및 반환.

#####  `get_ang_z()`

`get_angle()` 로 부터 반환받은 현재 시점의 `pose`와 목적 지점 도착 시점 `pose` 사이의 회전해야할 회전량 x 비례상수를 로봇 이동에 사용될 `/cmd_vel` 토픽의 `angular.z` 값에 반영시킨다.

#####  `move2goal()`

사용자로 부터 입력받은 목표 지점의 x, y값과 허용오차를  `get_dist()` 와 `get_angle()` 에 전달하고,  `get_lin_x()` 와  `get_ang_z()` 의 반환 값을 적절히 이용하여 로봇을 목표지점으로 이동시키는 함수. 



### 3. 코드 작성

`~/catkin_ws/src/tb3_cleaner/scripts` 폴더에 다음 내용으로  `gotogoal.py` 파일을 작성한다.

```python
#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from math import pow, atan2, sqrt, pi


class TurtleBot3:

    def __init__(self):
        rospy.init_node('move_to_goal', anonymous = True)
        self.sub  = rospy.Subscriber('/odom', Odometry, self.get_odom  )
        self.pub  = rospy.Publisher( '/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(10)
        
        # Turtlebot3 pose #
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0
            
    def get_odom(self, msg):
        pos_x, pos_y, theta = self.get_pose(msg)
        
        self.pos_x_2d = pos_x
        self.pos_y_2d = pos_y
        self.theta_2d = theta
        
        if   (self.theta_2d - self.prev_theta_2d) > 5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) - 2 * pi            
        elif (self.theta_2d - self.prev_theta_2d) < -5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) + 2 * pi
        else:
            d_theta = (self.theta_2d - self.prev_theta_2d)

        self.theta_2d_sum  = self.theta_2d_sum + d_theta
        self.prev_theta_2d = self.theta_2d
        
        self.theta_2d = self.theta_2d_sum
            
    def get_pose(self, data):
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
             data.pose.pose.orientation.z, data.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[2]	# euler_from_quaternion(q)[0] - roll
		                                    # euler_from_quaternion(q)[1] - pitch
		                                    # euler_from_quaternion(q)[2] - yaw <---
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y

        return pos_x, pos_y, theta
        
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose.x-self.pos_x_2d),2) + pow((goal_pose.y-self.pos_y_2d),2))
        
    def get_lin_x(self, goal_pose, constant = 1.15):
        return constant * self.get_dist(goal_pose)
        
    def get_angle(self, goal_pose):
        return = atan2(goal_pose.y - self.pos_y_2d, goal_pose.x - self.pos_x_2d)

    def get_ang_z(self, goal_pose, constant = 1.15):        
        return constant * (self.get_angle(goal_pose) - self.theta_2d)
                
    def move2goal(self):
        goal_pose = Pose()

        goal_pose.x = input("Input x goal: " )
        goal_pose.y = input("Input y goal: " )
        tolerance = input("Input tolerance: ")

        t = Twist()
        cnt4print = 0	# counter for print pose every second(1Hz)

        while(self.get_dist(goal_pose) >= tolerance):
            
            if(cnt4print >= 10):
                cnt4print = 0
                self.print_pose()
            
            cnt4print   = cnt4print + 1
            
            t.linear.x  = self.get_lin_x(goal_pose)
            t.linear.y  = t.linear.z  = 0
            
            t.angular.x = t.angular.y = 0
            t.angular.z = self.get_ang_z(goal_pose)

            self.pub.publish(t)
            self.rate.sleep()
            
        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        print("Robot is arrived at goal position!")
        
        rospy.spin()
                
    def print_pose(self):
        print("p.x: %f,  p.y: %f,  th: %f" %(self.pos_x_2d, self.pos_y_2d, self.theta_2d))
        

if __name__ == '__main__':
    try:
        
        tb3 = TurtleBot3()
        tb3.move2goal()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
```



### 4. 실행

`roscore` 실행

```
$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 Turtlebot3 의 라즈베리파이로 ssh 를 통해 원격 연결한다.

```
$ ssh pi@xxx.xxx.xxx.xxx
```



라즈베리파이에서 ```turtlebot3_bringup``` 패키지의 `turtlebot3_robot.launch` 파일을 실행한다.

```
pi@raspberrypi:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `gotogoal.py` 를 실행한다. 

```
$ rosrun turtlesim_cleaner gotogoal.py
Set your x goal: 0.8
Set your y goal: -0.5
```

Turtlebot3( buger ) 로봇이 위에 입력한 정보와 같이 동작하는 지 확인한다.



[튜토리얼 목록 열기](../README.md)


