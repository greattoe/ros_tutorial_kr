## Turtlebot3/ Tutorials/ Make 2D Pose Publisher



---

## Subscribe /odom & Publish 2D Pose

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [목표위치로 이동2](./tb3_4_GoToGoal.md)

**다음 튜토리얼 :** [2D Pose 토픽(/tb3pose)에 의한 회전](./tb3_6_Rotate_by_Pose2D.md)

**튜토리얼 목록 :** [README.md](../../README.md)

------

이 튜토리얼에서는 터틀봇3의 주행기록( Odometry )에 해당하는 `/odom` 토픽을 Subscribe( 구독 )하여 `turtlesim` 의 `pose` 형식으로 변환한 `/tb3pose` 을 Publish( 발행 )하는 노드( node )를 작성해 본다. 



다음은  **turtlesim** 의 **`/turtle1/pose`** 토픽이다. 

```json
---
x: 5.544444561
y: 5.544444561
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```



다음은 **터틀봇3** 의 **`/odom`** 토픽이다. 

```json
---
header: 
  seq: 1296
  stamp: 
    secs: 1569340595
    nsecs: 252484956
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: 0.091345705092
      y: 0.00217283004895
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0129650924355
      w: 0.999915957451
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.000215363950701
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```



**turtlesim** 의 `/turtle1/pose` 토픽과 **터틀봇3** 의 `/odom` 토픽의 관계는 다음과 같다. 

```json
---
x: 5.544444561 ----------> pose.pose.position.x
y: 5.544444561 ----------> pose.pose.position.y
theta: 0.0 --------------> euler_from_quarternion(pose.pose.orientation.x,y,z,w)[2]
linear_velocity: 0.0 ----> twist.twist.linear.x
angular_velocity: 0.0 ---> twist.twist.angular.z
---
```



이들을 바탕으로 **터툴봇3** 의 x-y 이차원 평면을 기준으로한 2D pose를 `turtlesim.msgs.Pose` 메세지 형식으로 발행(publish ) 하는 코드를 작성해보자.



이 전에 만들어 사용해오던  `tb3_cleaner` 패키지 폴더의 `scripts` 폴더로 경로를 변경한다.

```bash
$ cd ~/catkin_ws/src/tb3_cleaner/scripts
```

`pub_tb3_pose2d.py` 파일을 만들고 실행 속성을 부여한다. 

```bash
$ touch pub_tb3_pose2d.py
$ chmod +x pub_tb3_pose2d.py
```

`pub_tb3_pose2d.py` 파일을 gedit 로 편집한다. 

```bash
$ gedit pub_tb3_pose2d.py &
```

```python
#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import degrees, pi
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

class TB3Pose2D:

    def __init__(self):
    
        rospy.init_node('pub_tb3_pose', anonymous = True)
        
        rospy.Subscriber('/odom', Odometry, self.get_odom )
        self.pub = rospy.Publisher('/tb3pose', Pose, queue_size = 10)
        
        self.tb3pose2d = Pose()  # turtlesim.msg.Pose()      
        self.prv_theta = 0.0
        self.theta_sum = 0.0
        self.rate = rospy.Rate(10)
        
        
    def get_odom(self, dat):
        
        pos_x, pos_y, theta = self.get_pose(dat)
        
        pose2d       = Pose()   # turtlesim.msg.Pose()
        pose2d.x     = pos_x
        pose2d.y     = pos_y
        pose2d.theta = theta
        pose2d.linear_velocity  = dat.twist.twist.linear.x
        pose2d.angular_velocity = dat.twist.twist.linear.x
        
        if   (pose2d.theta - self.prv_theta) >  5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (pose2d.theta - self.prv_theta) - 2 * pi            
        elif (pose2d.theta - self.prv_theta) < -5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (pose2d.theta - self.prv_theta) + 2 * pi
        else:
            d_theta = (pose2d.theta - self.prv_theta)

        self.theta_sum = self.theta_sum + d_theta
        self.prv_theta = pose2d.theta
        
        pose2d.theta = self.theta_sum
        
        self.tb3pose2d = pose2d
        # self.print_pose(self.tb3pose2d)
        self.pub.publish(self.tb3pose2d)
        
        
    def get_pose(self, msg):
        
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                                            # quart[0] = roll
        quart = euler_from_quaternion(q)    # quart[1] = pitch
        theta = quart[2]                    # quart[2] = yaw <----
        
    	# make theta within from 0 to 360 degree
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        return pos_x, pos_y, theta
        
        
    def print_pose(self, msg):
        print("x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, degrees(msg.theta)))


if __name__ == '__main__':
    try:
        TB3Pose2D()
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



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `pub_tb3_pose2d.py` 를 실행한다. 

```bash
$ rosrun tb3_cleaner pub_tb3_pose2d.py
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 토픽 `/tb3pose` 의  발행여부를 확인한다.

```bash
$ rostopic list /tb3pose
```



토픽 `/tb3pose` 를 화면에 `echo` 한 후, **터틀봇3** 를 움직여 값의 변화가 정확한 지 확인한다.

```bash
$ rostopic echo /tb3pose
x: -0.000152075575897
y: 5.91442449149e-06
theta: 0.0540497899055
linear_velocity: 0.0
angular_velocity: 0.0
---
x: -0.000152075575897
y: 5.91442449149e-06
theta: 0.0540136098862
linear_velocity: 0.0
angular_velocity: 0.0
---
```



[이전 튜토리얼](./tb3_4_GoToGoal.md)

[다음 튜토리얼](./tb3_6_Rotate_by_Pose2D.md)

[튜토리얼 목록 열기](../README.md)

