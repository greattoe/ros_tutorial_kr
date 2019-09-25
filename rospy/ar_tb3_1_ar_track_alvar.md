## Turtlebot3/ Tutorials/ AR Maker



---

## ar_track_alvar

**이 문서의 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**관련 자료 :** <http://wiki.ros.org/ar_track_alvar>

**REPOSITORY :** <https://github.com/ros-perception/ar_track_alvar>

**문서 목록 :** [README.md](../README.md)

---

ROS 에서 AR( Augmented Reality: 증강현실 ) Marker 관련 작업에 가장 널리 사용되는 패키지 중 하나



### 1. 패키지 설치

바이너리 설치와 소스코드 빌드 중 한가지 방법으로 설치



### 1.1 소스코드 빌드

github의 ar_track_alvar 패키지 Repository로부터 git clone 하기 위해 경로를 catkin 워크스페이스의 src 폴더로 변경

```
user@computer:~$ cd ~/catkin_ws/src
```

git clone 명령으로 소스코드 복사

```
user@computer:~/catkin_ws/src$ git clone https://github.com/ros-perception/ar_track_alvar
```

catkin_make 실행을 위한 경로 변경 후 빌드

```
user@computer:~/catkin_ws/src$ cd ~/catkin_ws/src
user@computer:~/catkin_ws$ catkin_make
```



#### 1.2 바이너리 설치

`$ apt-get install` 명령을 이용한 바이너리 설치는 'ar_track_alvar' 와 'ar_track_alvar_msg' 두 패키지를 설치한다. 

```
user@computer:~/catkin_ws$ sudo apt-get install ros-kinetic-ar-track-alvar*
```



### 2. AR Marker 토픽 Subscribe

회전 속도, 회전 각도, 회전 방향( 좌/우회전 )을 입력받아 입력값들의 형식 및 단위를 적절히 변환 후, 회전시간을 계산하여  `tb3_node` 에서 subscribe 하는 `'/cmd_vel'` 토픽으로 publish 한다.



### 3. 코드(수정)

`~/catkin_ws/src/tb3_cleaner/scripts` 폴더에 다음 내용으로  `rotate.py` ( 또는 원하는 이름의 파일 )파일을 작성한다.

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
toRAD = 0.0174533

def rotate():
    # Starts a new node
    rospy.init_node('tb3_cleaner', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    # Receiveing the user's input
    print("input speed within 0.0 ~ 160")
    speed     = input("Input your speed (degrees/sec):")
    
    print("input speed within 0 ~ 360")
    angle     = input("Type your distance (degrees):")
    
    print("direction cw:1, ccw:0")
    clockwise = input("Clockwise?: ")
    
    angular_speed  = speed * toRAD
    relative_angle = angle * toRAD

    msg.linear.x  = msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = 0
    
    if clockwise:
        msg.angular.z = -abs(angular_speed)
    else:
        msg.angular.z =  abs(angular_speed)
        
    duration = relative_angle / angular_speed
    time2end = rospy.Time.now() + rospy.Duration(duration)
    
    pub.publish(msg)
    rospy.sleep(0.001)
        
    while(rospy.Time.now() < time2end):
        pass    
        
    msg.angular.z = 0
    pub.publish(msg)
    

if __name__ == '__main__':
    try:
        rotate()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
```



### 4. 실행(수정)

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



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `rotate.py` 를 실행한다. 

```
user@computer:~$ rosrun turtlesim_cleaner move.py
Let's move your robot
Input your speed (degrees/sec):20
Type your distance (degrees):120
Clockwise?: 0
```

Turtlebot3( buger ) 로봇이 위에 입력한 정보와 같이 동작하는 지 확인한다.





[튜토리얼 목록 열기](../README.md)

[이 링크도 수정]()







