## turtlesim/ Tutorials/ Moving in a Straight Line



---

## turtlesim 직선 이동

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [좌/우회전](./tb3_2_Rotate_Left_n_Right.md)

**튜토리얼 목록 :** [README.md](../README.md)

---

이 튜토리얼 시리즈에서는 ROS 기본을 익히기 위한 turtlesim 노드의 거북이를 움직이는 파이썬 스크립트를 작성할 것이다. 다음 링크에서 전체 소스코드를 찾을 수 있다. <https://github.com/clebercoutof/turtlesim_cleaner>



### 1. 준비작업

`geometry_msgs` 와  `rospy` 에 의존성을 갖는 새로운 패키지 `turtlesim_cleaner` 생성

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg turtlesim_cleaner geometry_msgs rospy
```

생성된 패키지 폴더의 `src` 폴더로 작업경로 변경

```bash
$ cd ~/catkin_ws/src/turtlesim_cleaner/src
```

파이썬 코드를 작성할 `scripts` 폴더 생성 후, 생성된 폴더로 경로 변경

```bash
$ mkdir scripts
$ cd scripts
```



### 2. 구현할 기능

이동 속도, 이동 거리, 이동 방향( 전/후진 )을 입력받아 입력값들의 형식 및 단위를 적절히 변환 후, 이동시간을 계산하여  `turtlesim_node` 에서 subscribe 하는 `'/turtle1/cmd_vel'` 토픽으로 publish 한다.



### 3. 코드

`~/catkin_ws/src/turtlesim_cleaner/scripts` 폴더에 `move.py` 파일을 작성한다.

```
$ gedit move.py &
```

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node name 'robot_cleaner'
    rospy.init_node('robot_cleaner', anonymous=True)
    # declair publisher name:pub, topic:'/turtle1/cmd_vel', type:Twist
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # declair Twist type object name 'msg'
    msg = Twist()

    # Receiveing the user's input
    print("Let's move your robot")
    speed     = input("Input your speed: ")
    distance  = input("Type your distance: ")
    isForward = input("Foward?: ") # True or False(1 or 0)

    # Checking if the movement is forward or backwards
    if(isForward):
        msg.linear.x =  abs(speed)
    else:
        msg.linear.x = -abs(speed)
        
    # Since we are moving just in x-axis
    msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = msg.angular.z = 0

    while not rospy.is_shutdown():

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        # Loop to move the turtle in an specified distance
        while(current_distance < distance):
            # Publish the velocity
            pub.publish(msg)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        # After the loop, stops the robot
        msg.linear.x = 0
        # Force the robot to stop
        pub.publish(msg)

if __name__ == '__main__':
    try:
        # Testing our function
        move()
    except rospy.ROSInterruptException: pass
```

작성한 파일에 실행 속성 부여.

```
$ chmod +x move.py
```



### 4. 빌드 및 실행

`catkin_make` 실행을 위해 작업 경로를 `~/catkin_ws` 로 변경한다.

```
$ cd ~/catkin_ws
```

`catkin_make` 실행.

```
$ catkin_make
```

변경된  `~/catkin_ws/devel/setup.bash` 의 내용을 `source` 명령을 이용하여 반영시킨다.

```
$ source ./devel/setup.bash
```



`roscore` 실행

```
$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 `turtlesim` 노드를 실행한다.

```
$ rosrun turtlesim turtlesim_node
```

![대기중인 거북이](../img/move_py_1.png)



`Ctrl+Alt+T` 를 입력하여 새 터미널을 열고 작성한  `move.py` 를 실행한다. 

```
$ rosrun turtlesim_cleaner move.py
Let's move your robot
Input your speed: 0.5
Type your distance: 3.0
Foward?: 1
```

![직진하는거북이](../img/move_py_2.png)



[튜토리얼 목록 열기](../README.md)

[다음 튜토리얼](./mv_tutle_2_RotateLeftRight.md)







