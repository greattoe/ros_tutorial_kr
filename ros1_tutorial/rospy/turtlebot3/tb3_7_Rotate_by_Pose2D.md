## Turtlebot3/ Tutorials/ Rotate by 2D Pose



---

## 2D Pose 값을 이용한 좌, 우회전

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [Odometry 토픽을 이용한 2D Pose 토픽 발행](./tb3_5_Sub_Odom_Pub_Pose2D.md)

**다음 튜토리얼 :** [좌,우 회전 및 전,후진 지원 라이브러리]()

**튜토리얼 목록 :** [README.md](../README.md)

------

이 튜토리얼에서는 [5. Odometry 토픽을 이용한 2D Pose 토픽 발행](./tb3_5_Sub_Odom_Pub_Pose2D.md) 튜토리얼에서 작성한 노드 `pub_tb3_pose2d.py`가 `publish(발행)`하는 토픽 `/tb3pose` 을 `subscribe(구독)`한 정보를 기준으로 입력받은 각도 만큼 회전하는 기능을 구현한다.

아래는 `pub_tb3_pose2d.py` 노드가 발행하는 `/tb3pose` 토픽이다. 

```json
---
x: -0.000152075575897
y: 5.91442449149e-06
theta: 0.0540136098862
linear_velocity: 0.0
angular_velocity: 0.0
---
```

이 토픽을 `subscribe` 한 `theta` 값을 기준으로 **터틀봇3** 를 좌 또는 우회전시키는 노드 `rotate_by_tb3pose2d.py` 를 작성한다. 



**`rotate_by_tb3pose2d.py` 에 구현해야하는 기능**

1. `theta` 값의 갱신을 위한 `/tb3pose` 토픽 `subscriber`
2. 회전을 시작하는 순간의 `theta` 값 저장
3. 입력값의 부호에 따라 회전방향 결정
4. 회전한 각(또는 남은 각) 계산
5. 입력받은 각도만큼 회전

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
from math import sqrt, degrees, radians

MAX_ANG_SPEED =  2.84
ANG_SPD = MAX_ANG_SPEED * 0.125

class MoveTB3:

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
        # save current tb3pose.theta to org.theta when called this function
        self.org = self.tb3pose 
        
    def elapsed_angle(self):
        return abs(self.tb3pose.theta - self.org.theta)
        
    def rotate(self, angle):
        tw = Twist()
        self.update_org()
        print "start from: %s" %(round(degrees(self.org.theta), 2))
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD;
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD;
            
        self.pub.publish(tw)
        while self.elapsed_angle() < abs(angle):    pass
            # print "%s of %s" %(round(degrees(self.elapsed_angle()),2) ,round(degrees(abs(angle)),2))
            
        tw.angular.z =  0;  self.pub.publish(tw)
        print "stop to   : %s" %(round(degrees(self.tb3pose.theta), 2))        

if __name__ == '__main__':
    try:
        tb3 = MoveTB3()
        angle = radians(input("inTB3put angle to rotate(deg): "))
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
$ rosrun tb3_cleaner pub_tb3_pose2d.py
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 이 튜토리얼에서 작성한  `rotate_by_pose2d.py` 를 실행한다.

```bash
$ rosrun tb3_cleaner rotate_by_pose2d.py
input angle to rotate(deg): -90
start from: -248.35
stop to   : -338.7
```



[튜토리얼 목록 열기](../README.md)


