## Turtlebot3/ Tutorials/ Rotate by 2D Pose



---

## Rotate by 2D Pose

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [Odometry 토픽을 이용한 2D Pose 토픽 발행](./tb3_5_Sub_Odom_Pub_Pose2D.md)

**다음 튜토리얼 :** [AR마커 검색]()

**튜토리얼 목록 :** [README.md](../README.md)

------

[이전 튜토리얼](./tb3_5_Sub_Odom_Pub_Pose2D.md)에서는 터틀봇3의 주행기록( Odometry )에 해당하는 `/odom` 토픽을 Subscribe( 구독 )하여 `turtlesim` 의 `pose` 형식으로 변환한 `/tb3pose` 을 Publish( 발행 )하는 노드( node )를 작성했다. 이 때 해당 노드가 발행하는 토픽 `/tb3pose` 는 다음과 같다. 

```json
---
x: -0.000152075575897
y: 5.91442449149e-06
theta: 0.0540136098862
linear_velocity: 0.0
angular_velocity: 0.0
---
```

이 토픽을 `subscribe` 한 `theta` 값을 기준으로 **터틀봇3**  회전시키는 노드를 만들어보자. 

**구현할 기능**

1. `theta` 값을 갱신하는 `subscriber`
2. 회전할 각도를 현재 `theta` 값에 반영하여 목표 `theta` 값을 계산
3. 입력값의 부호에 따라 회전방향을 결정

등의 기능을 구현한다.

`tb3_cleaner` 패키지 폴더의 `scripts` 폴더로 경로를 변경한다.

```bash
$ cd ~/catkin_ws/src/tb3_cleaner/scripts
```

`rotate_by_pose2d.py` 파일을 만들고 실행 속성을 부여한다. 

```bash
$ touch rotate_by_pose2d.py
$ chmod +x rotate_by_pose2d.py
```

`rotate_by_pose2d.py` 파일을 gedit 로 편집한다. 

```bash
$ gedit rotate_by_pose2d.py &
```

```python
#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import radians, degrees, pi

MAX_ANG_SPEED = 2.84
ANG_SPEED = MAX_ANG_SPEED / 4

class TB3Rotate:

    def __init__(self):    
        rospy.init_node('rotate_by_pose', anonymous = True)        
        rospy.Subscriber('/tb3pose', Pose, self.get_theta)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)        
        self.tb3pose2d = Pose() 
        self.theta_now = 0.0        
        
    def get_theta(self, dat):
        self.theta_now = dat.theta        
        
    def get_goal(self, angle):
        return self.theta_now + angle
        
    def rotate(self, angle):
        tw = Twist()
        target_angle = self.get_goal(angle)
        print("current = %f, target = %f" %(degrees(self.theta_now), degrees(target_angle)))
        
        if angle >= 0:	# +angle
            tw.angular.z =  ANG_SPEED;  self.pub.publish(tw)
            while self.theta_now < target_angle:    pass
            tw.angular.z =  0;  self.pub.publish(tw)
        else:			# -angle
            tw.angular.z = -ANG_SPEED;  self.pub.publish(tw)
            while self.theta_now > target_angle:    pass
            tw.angular.z =  0;  self.pub.publish(tw)
            

if __name__ == '__main__':
    try:
        tb3 = TB3Rotate()
        angle = radians(float(input("input angle(deg) to turn: ")))
        tb3.rotate(angle)
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
```



`roscore` 실행

```bash
$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 Turtlebot3 의 라즈베리파이로 ssh 를 통해 원격 연결한다.

```bash
$ ssh pi@xxx.xxx.xxx.xxx
```



라즈베리파이에서 ```turtlebot3_bringup``` 패키지의 `turtlebot3_robot.launch` 파일을 실행한다.

```bash
pi@raspberrypi:~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 이전 튜토리얼에서 작성한  `pub_tb3_pose2d.py` 를 실행한다. 

```bash
$ rosrun turtlesim_cleaner pub_tb3_pose2d.py
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 이 튜토리얼에서 작성한  `rotate_by_pose2d.py` 를 실행한다.

```bash
$ rosrun turtlesim_cleaner rotate_by_pose2d.py
input angle(deg) to turn: -215
current = 51.628403, target = -163.371597
```



[튜토리얼 목록 열기](../README.md)


