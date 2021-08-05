## Turtlebot3/ Tutorials/ search_ar_marker



---

## AR 마커 찾기

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 18.04 **/** Melodic

**이전 튜토리얼 :** [목표지점으로 이동 3](./tb3_9_move2xy.md)

**다음 튜토리얼 :** 

**튜토리얼 목록 :** [README.md](../../../README.md)

------

**선행되어야 할 학습**

1. [카메라 Calibration](../../camera_calibration/how_to_calibrate_monocular_camera.md)

2. [ar_track_alvar 구동](../ar_1_ar_track_alvar.md)
3. [ AR 마커 정보 해석](../ar_2_analysis_marker.md)

위 3가지 튜토리얼을 수행했다면 PC 에는 `ar_track_alvar` 패키지가 설치되어 있고, `catkin_ws/src/` 에는 `launch` 파일만 존재하는 `ar_marker` 패키지와 `uvc_track_marker.launch` 파일이 작성되어 있을 것이다.

**필요한 작업**

1. 
2. 

**`tb3_searching_marker.py` 작성**

```
roscd tb3_cleaner/scripts
```

```
touch tb3_searching_marker.py 
```

```
chmod +x tb3_searching_marker.py
```

```
gedit tb3_searching_marker.py &
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

