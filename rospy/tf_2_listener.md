## tf/ Tutorials/ Writing a tf listener (Python)



------

## tf listener 작성 방법 (Python)

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [tf broadcaster](./tf_1_broadcaster.md)

**다음 튜토리얼 :** [add tf frame](tf_3_adding_frame.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29>

------

이 전의 튜토리얼에서 tf broadcaster를 만들어 turtlesim 거북이의 pose 를 tf 로 broadcast 했었다. 이 튜토리얼에서는 tf listener 를 만들고 본격적인 tf 사용법을 알아보자.



### 1. tf listener 생성 방법 

첫 번째 코드를 작성하기 위해 이 전 튜토리얼에서 만든 learing_tf 패키지 폴더로 이동하자

```
user@computer:~/catkin_ws$ roscd learning_tf/nodes
```



#### 1.1 코드

자신이 선호하는 편집기를 가지고 ~/catkin_ws/src/learning_tf/nodes 에 아래 코드와 같이 **turtle_tf_listener.py** 파일을 작성한다.

```python
#!/usr/bin/env python 

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':   
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()       #

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
```

작성한 코드에 실행속성 주는 것을 잊지말자.

```
user@computer:~/catkin_ws/learning_tf/nodes$ chmod +x turtle_tf_listener.py
```



#### 1.2 코드 설명

거북이의 pose 를 tf 에게 publish 하는 것과 연관된 코드를 살펴보자

tf 패키지는 tf.TransformListener를 제공함으로서  transform 을 수신 작업을  보다 간편하게 만들어 준다.

```python
import tf
```

여기서 TransformListener 객체를 생성한다. listener 가  한 번 만들어지면 네트워크를 통해 tf transformation 수신을 시작하며, buffer 에는 최대 10초까지의 수신 데이터를 담아 둘 수 있다.

```python
    listener = tf.TransformListener()
```

이 지점에서 실제 작업이 이루어지며, 특정 transformation 을 위한 쿼리를 listener 에게 요청한다. argumet 들을 살펴보자.

1. /turtle1 프레임으로부터 ...
2. ... /turtle2 프레임에게로의 transform이 필요하다.
3. Time은 transform이 수행되기를 바라는 시간을 말하는데,  `rospy` 에서 제공되는 rospy.Time(0)을 사용하면 가장 최근에 transform이 이루어진 시간을 알 수 있다.

이 함수는 두 개의 리스트를 반환한다. parent 프레임에 연관지은 (x, y, z) linear transform 과 parent 프레임 기준으로부터 child 프레임으로의 회전에 필요한 (x, y, z, w) quaternion 이다.

발생가능한 예외 처리를 위해 이와 관련된 모든 코드를 try - except 블럭으로 둘러싼다. 

```c++
rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
```



### 2. listener 의 실행

 ~/catkin_ws/src/learning_tf/launch 폴더의 **start_demo.launch** 파일을 열고 아래의 노드 블럭을 `<launch>` 블럭 안에 추가하라.

```xml
  <launch>
    <!-- Turtlesim Node-->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>

    <node name="turtle1_tf_broadcaster" pkg="learning_tf" type="turtle_tf_broadcaster.py" respawn="false" output="screen" >
      <param name="turtle" type="string" value="turtle1" />
    </node>
    <node name="turtle2_tf_broadcaster" pkg="learning_tf" type="turtle_tf_broadcaster.py" respawn="false" output="screen" >
      <param name="turtle" type="string" value="turtle2" /> 
    </node>
    <!-- 여기에 추가 -------------------------------------->
    <node pkg="learning_tf" type="turtle_tf_listener.py"
          name="listener" />
    <!-------------------------------------------------->
  </launch>
```

우선 앞 서 진행했던 튜토리얼의 launch 파일이 실행 중이라면 ctrl-c 를 입력하여 그것을 먼저 종료한다. 이제 직접 모든 기능이 갖춰진 데모를 시작할 준비가 완료되었다.

```
user@computer:~/catkin_ws$ roslaunch learning_tf start_demo.launch
```

turtlesim 노드가 실행되고 거북이가 두 마리 보일 것이다.



### 3. 결과 확인

제대로 동작한다면 키보드로 첫 번 째 거북이를 이리저리 움직이면 두 번 째 거북이가 첫 번 째 거북이를  따라다니는 것을 볼 수 있을 것이다.

turtlesim 구동이 시작될 때 아마 다음 메세지를 보게 될 것이다.

```
[ERROR] 1253915565.300572000: Frame id /turtle2 does not exist! When trying to transform between /turtle1 and /turtle2.
[ERROR] 1253915565.401172000: Frame id /turtle2 does not exist! When trying to transform between /turtle1 and /turtle2.
```

이 것은 두 번 째 거북이에 대한 메세지를 수신하기 전에 transform을 계산하려 하기 때문에 발생한다. turtlesim 이 구동되고 나서 tf frame 을 broadcasting 하기까지 약간의 시간이 소요되기 때문이다.

다음 튜토리얼은 tf frame을 추가하는 방법이다. 



[튜토리얼 목록 열기](../README.md)



[다음 튜토리얼](./tf_3_adding_frame.md)

[이전 튜토리얼](./tf_1_broadcaster.md)

