## Turtlebot3/ Tutorials/ Straight by 2D Pose



---

## 2D Pose 값을 이용한 전, 후진

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [Odometry 토픽을 이용한 2D Pose 토픽 발행](./tb3_5_Sub_Odom_Pub_Pose2D.md)

**다음 튜토리얼 :** [2D Pose 토픽에 의한 좌, 우 회전](./tb3_7_Rotate_by_Pose2D.md)

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

이 토픽을 `subscribe` 한 `x`,`y` 값을 기준으로 **터틀봇3** 를 전진 또는 후진시키는 노드 `straight_by_tb3pose2d` 를 작성한다.

 

**`straight_by_tb3pose2d` 에서 구현해야하는 기능**

1. `x` ,  `y` 값을 `update` 하기위한`/tb3pose` 토픽 `subscriber`
2. 전진, 또는 후진을 시작하는 순간의 `x`, `y` 값 저장
3. 입력값의 부호에 따른 이동 방향(전, 후진) 결정
4. 이동한 거리(또는 남은 거리) 계산
5. 입력받은 거리만큼 이동

등의 기능을 구현한다.



`tb3_cleaner` 패키지 폴더의 `scripts` 폴더로 경로를 변경한다.

```bash
$ cd ~/catkin_ws/src/tb3_cleaner/scripts
```

`straight_by_pose2d.py` 파일을 만들고 실행 속성을 부여한다. 

```bash
$ touch straight_by_pose2d.py
$ chmod +x straight_by_pose2d.py
```

`straight_by_pose2d.py` 파일을 gedit 로 편집한다. 

```bash
$ gedit straight_by_pose2d.py &
```

```python
#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt

MAX_LIN_SPEED =  0.22
LIN_SPD = MAX_LIN_SPEED * 0.125

class TB3Move:

    def __init__(self):    
        rospy.init_node('rotate_by_pose', anonymous = True)
        rospy.Subscriber('/tb3pose', Pose, self.get_pose_cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)        
        
        self.tb3pose  = Pose()  # for subscribe
        self.org      = Pose()  # for store starting point
        
    def get_pose_cb(self, msg):
        # callback function to subscribe "/tb3pose" topic
        self.tb3pose = msg
        
    def update_org(self):
        # save current tb3pose.x, y to org.x, y when called this function
        self.org = self.tb3pose
        
    def elapsed_dist(self):
        # calcurate and return elapsed distance
        return sqrt(pow((self.tb3pose.x - self.org.x), 2) + pow((self.tb3pose.y - self.org.y), 2))
    
    def straight(self, distance):
        # forward or backward until elaped distance is equal to target distance
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        print("start at (%s, %s)" %(round(self.org.x, 2), round(self.org.y, 2)))#,
        self.pub.publish(tw)    # start move
        
        while self.elapsed_dist() < abs(distance):  pass
        # wait until elapsed distance = target distance
            # print "%s(m) of %s(m)" %(round(self.elapsed_dist(),2), round(abs(distance),2))
        
        tw.linear.x = 0;    self.pub.publish(tw) # stop move
        print("stop  at (%s, %s)" %(round(self.tb3pose.x, 2), round(self.tb3pose.y, 2)))

if __name__ == '__main__':
    try:
        tb3 = TB3Move()
        
        dist = float(input("input distance to straight(m): "))
        tb3.straight(dist)
        
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
$ rosrun tb3_cleaner straight_by_pose2d.py 
input distance to straight(m): -0.25
start from (0.13, -0.28)
stop to    (0.26, -0.07)
```



[튜토리얼 목록 열기](../README.md)


