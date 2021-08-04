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
7. 산출된 -회전각 만큼 회전( 시작 pose 로 복귀 **)**

**`tb3_move2xy.py` 작성**

```
roscd tb3_cleaner/scripts
```

```
touch tb3_move2xy.py 
```

```
chmod +x tb3_move2xy.py
```

```
gedit tb3_move2xy.py &
```

```python
#!/usr/bin/env python
import rospy
from tb3_cleaner.MoveTB3 import MoveTB3
from math import degrees, sqrt, atan2

class MoveTo_XY:

    def __init__(self):
        self.move = MoveTB3()
    
    def move2xy(self, x, y):
        
        angle = atan2(y, x)
        dist  = sqrt(pow(x, 2) + pow(y, 2))
        
        print "rotate %s(deg), and than straight %s(m)" %(degrees(angle), dist)
        
        self.move.rotate(angle)
        self.move.straight(dist)
        self.move.rotate(-angle)
    
    
if __name__ == '__main__':

    try:
        rospy.init_node('move_to_xy', anonymous = True)        
        xy = MoveTo_XY()
        
        while not rospy.is_shutdown():
            dist_x = float(input("input distance x(m): "))
            dist_y = float(input("input distance y(m): "))
            xy.move2xy(dist_x, dist_y)
            
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

```



**작성 코드 테스트**

1. PC 에서 `roscore` 실행

2. 터틀봇3 SBC 에 `ssh` 연결

3. 터틀봇3 에서  `roslaunch turtlebot3_bringup turtlebot3_robot.launch` 실행

4. PC 에서 `rosrun tb3_cleaner pub_tb3_pose2d.py` 실행

5. PC 에서 `rosrun tb3_cleaner tb3_move2xy.py` 실행

   - x 축 방향 이동거리 입력
   - y 축 방향 이동거리 입력

   ```
   $ rosrun tb3_cleaner tb3_move2xy.py
   input distance x(m): 0.5
   input distance y(m): 0.5
   rotate 45.0(deg), and than straight 0.707106781187(m)
   start from: -1.1
   stop to   : 43.91
   start from (0.51, 0.5)
   stop to    (0.99, 1.02)
   start from: 47.23
   stop to   : 1.67
   ```

   







[튜토리얼 목록 열기](../../../README.md)

