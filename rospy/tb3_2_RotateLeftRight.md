## Turtlebot3/ Tutorials/ Rotate Left and Right



---

## Turtlebot3 좌 / 우 회전

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [목표지점으로 이동](./mv_tb3_3_Go2Goal.md)

**이전 튜토리얼 :** [직선으로 이동](./mv_tb3_1_MoveInStraightLine.md)

**튜토리얼 목록 :** [README.md](../README.md)

---

이 튜토리얼에서는 사용자 입력을 받아, Turtlebot3 로봇을 원하는 각도 만큼 좌, 우로 이동하는  노드를 작성할 것이다. 이 코드는 [turtlsim_cleaner](https://github.com/clebercoutof/turtlesim_cleaner) 코드를 참고하여 만들었음을 밝힌다.



### 1. 준비작업

앞서 [이 전 튜토리얼](./mv_tutle_1_MoveInStraightLine.md)에서 만든  `tb3_cleaner` 패키지의 `scripts` 폴더로 경로를 변경한다.

```
user@computer:~$ cd ~/catkin_ws/src/tb3_cleaner/scripts
```

`rotate.py` 파일을 만들고 실행 속성을 부여한다. 

```
user@computer:~/catkin_ws/src/tb3_cleaner/scripts$ touch rotate.py
user@computer:~/catkin_ws/src/tb_cleaner/scripts$ chmod +x rotate.py
```



### 2. 구현할 기능

회전 속도, 회전 각도, 회전 방향( 좌/우회전 )을 입력받아 입력값들의 형식 및 단위를 적절히 변환 후, 회전시간을 계산하여  `tb3_node` 에서 subscribe 하는 `'/cmd_vel'` 토픽으로 publish 한다.



### 3. 코드

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

[목표지점으로 이동](./mv_tb3_3_Go2Goal.md)







