## Turtlebot3/ Tutorials/ Moving in a Straight Line



---

## Turtlebot3 직선 이동

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [좌/우회전](./mv_tb3_2_RotateLeftRight.md)

**튜토리얼 목록 :** [README.md](../README.md)

---

이 튜토리얼에서는 사용자 입력을 받아, Turtlebot3 로봇을 원하는 거리만큼 앞, 또는 뒤로 이동하는  노드를 작성할 것이다. 이 코드는 [turtlsim_cleaner](https://github.com/clebercoutof/turtlesim_cleaner) 코드를 참고하여 만들었음을 밝힌다.



### 1. 준비작업

`geometry_msgs` 와  `rospy` 에 의존성을 갖는 새로운 패키지 `tb3_cleaner` 생성

```
user@computer:~$ cd ~/catkin_ws/src
user@computer:~/catkin_ws/src$ catkin_create_pkg tb3_cleaner geometry_msgs rospy
```

생성된 패키지 폴더로 작업경로 변경

```
user@computer:~/catkin_ws$ cd ~/catkin_ws/src/tb3_cleaner
user@computer:~/catkin_ws/src/tb3
```

`setup.py` 편집

```
user@computer:~/catkin_ws/src/turtlesim_cleaner$ gedit setup.py &
```

다음 내용과 같이 `~/catkin_ws/src/turtlesim_cleaner/setup.py` 파일 작성.

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['tb3_cleaner'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```

파이썬 코드를 작성할 `scripts` 폴더 생성 후, 생성된 폴더로 경로 변경

```
user@computer:~/catkin_ws/src/tb3_cleaner$ mkdir scripts
user@computer:~/catkin_ws/src/tb3_cleaner$ cd scripts
user@computer:~/catkin_ws/src/tb3_cleaner/scripts$ 
```



### 2. 구현할 기능

이동 속도, 이동 거리, 이동 방향( 전/후진 )을 입력받아 입력값들의 형식 및 단위를 적절히 변환 후, 이동시간을 계산하여  `Turtlebot3` 에서 subscribe 하는 `'/cmd_vel'` 토픽으로 publish 한다.



### 3. 코드

`~/catkin_ws/src/tb3_cleaner/scripts` 폴더에 다음 내용으로  `move.py` ( 또는 원하는 이름의 파일 )파일을 작성한다.

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('tb3_cleaner', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    print("input speed within 0.0 ~ 0.22")
    speed = input("Input your speed: ")
    
    print("input distance within 0.0 ~ 2.84")
    distance = input("Type your distance: ")
    
    print("direction foward:1, backward:0")
    isForward = input("Forward?: ") # True or False(1 or 0)

    if(isForward):
        msg.linear.x =  abs(speed)
    else:
        msg.linear.x = -abs(speed)
        
    msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = msg.angular.z = 0

    while not rospy.is_shutdown():
        
        duration = distance / speed
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        pub.publish(msg)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):
            pass    
        
        msg.linear.x = 0
        pub.publish(msg)

if __name__ == '__main__':
    try:
        move()
        rospy.spin()
    except rospy.ROSInterruptException: pass
```

작성한 파일에 실행 속성 부여.

```
user@computer:~/catkin_ws/src/tb3_cleaner/scripts$ chmod +x move.py
```



### 4. 빌드 및 실행

`catkin_make` 실행을 위해 작업 경로를 `~/catkin_ws` 로 변경한다.

```
user@computer:~/catkin_ws/src/tb3_cleaner/scripts$ cd ~/catkin_ws
```

`catkin_make` 실행.

```
user@computer:~/catkin_ws$ catkin_make
```

변경된  `~/catkin_ws/devel/setup.bash` 의 내용을 `source` 명령을 이용하여 반영시킨다.

```
user@computer:~/catkin_ws$ source ./devel/setup.bash
```



`roscore` 실행

```
user@computer:~/catkin_ws$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고  `Turtlebot3` 의 라즈베리파이에 `ssh` 프로토콜로 원격 연결한다.

```
user@computer:~$ ssh pi@xxx.xxx.xxx.xxx
```

라즈베리파이에서 ```turtlebot3_bringup``` 패키지의 `turtlebot3_robot.launch` 파일을 실행한다.

```
pi@raspberrypi:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `move.py` 를 실행한다. 

```
user@computer:~$ rosrun turtlesim_cleaner move.py
Let's move your robot
Input your speed: 0.5
Type your distance: 3.0
Foward?: 1
```



Turtlebot3( buger ) 로봇이 위에 입력한 정보와 같이 동작하는 지 확인한다.



[튜토리얼 목록 열기](../README.md)

[다음 튜토리얼](./mv_tutle_2_RotateLeftRight.md)







