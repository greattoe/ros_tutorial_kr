## Turtlebot3/ Tutorials/ Straight by 2D Pose



---

## 2D Pose 값(odom)을 이용한 이동

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [2D Pose 값을 이용한 회전 및 직선이동 라이브러리](./tb3_8_Rotate_n_Straight_Library.md)

**다음 튜토리얼 :** 

**튜토리얼 목록 :** [README.md](../../../README.md)

------

이 튜토리얼에서는 현재 위치를 기준으로 x 축 방향으로의 거리와 y 축 방향으로의 거리를 입력받아 해당 위치로 이동하는 기능을 이전 튜토리얼에서 작성한 `MoveTB3.py` 라이브러리를 사용하여 구현한다. 

**구현할 기능**

1. 시작 위치의 tb3pose2d 값 저장 ( x, y, theta )
2. x 축 방향 이동거리 입력 받기( 양수 입력은 +x 방향, 음수 입력은 -x 방향으로 이동 )
3. y 축 방향 이동거리 입력 받기( 양수 입력은 +y 방향, 음수 입력은 -y 방향으로 이동 )
4. 현재의 2d_pose 값을 기준으로 회전각 및 이동거리 산출
5. 산출된 회전각만큼 회전
6. 산출된 거리만큼 전진
7. 산출된 -회전각 만큼 회전( 시작 pose 로 복귀 )



```python
#!/usr/bin/env python
import rospy
from tb3_cleaner.MoveTB3 import MoveTB3
from math import radians

class MoveTo_XY:
    def __init__(self):
        

if __name__ == '__main__':

    try:
        rospy.init_node('move_to_xy', anonymous = True)        
        tb3 = TB3Move()
        
        while not rospy.is_shutdown():
            dist_x = float(input("input distance x(m): "))
            dist_y = float(input("input distance y(m): "))
            tb3.move2xy(dist_x, dist_y)
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

