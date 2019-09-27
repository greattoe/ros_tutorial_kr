## tf/ Tutorials/ Writing a tf broadcaster (Python)



---


## tf broadcaster 작성 방법 (Python)

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [tf(trnasform) 소개](./tf_0_Instroduction.md)

**다음 튜토리얼 :** [tf listener 작성](./tf_2_listener.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29>

------

이 후의 두 튜토리얼에서 [tf introduction](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) 튜토리얼의 데모와 같이 동작하는 코드를 작성할 것이다. 그리고 나서 tf 의 고급 기능을 이용한 데모 코드의 기능 확장에 집중할 것이다.

시작하기 전에 이 프로젝트를 위한  tf, [roscpp](http://wiki.ros.org/roscpp), [rospy](http://wiki.ros.org/rospy) and [turtlesim](./turtlesim.md)에 대한 의존성을 갖는 learning_tf 라는 새로운 ROS 패키지를 만든다.

```
user@computer:~$ cd ~/catkin_ws/src
user@computer:~/catkin_ws/src$ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
```

아무 내용도 작성하지 않은 빈 패키지이지만 roscd 명령으로 이동할 수 있도록 미리 빌드한다.

```
user@computer:~/catkin_ws/src$ cd ~/catkin_ws
user@computer:~/catkin_ws$ catkin_make
user@computer:~/catkin_ws$ source ./devel/setup.bash
```



### 1. transforms 를 broadcast 하는 방법 

이 튜토리얼은 tf 에게 좌표 프레임을 broadcast 하는 방법을 가르쳐 줄 것이다. 이 경우 거북이들의 이동에 따른 그들의 좌표 프레임의 변화를 broadcast 하기를 바랄 것이다.

첫 번째 코드를 작성하기 위해 조금 전 생성한 learning_tf 패키지 폴더로 이동하자

```
user@computer:~/catkin_ws$ roscd learning_tf
```



#### 1.1 코드

일단 learning_tf 패키지에 'nodes' 폴더를 만드는 것 부터 시작하자.

```
user@computer:~/catkin_ws/learning_tf$ mkdir nodes
user@computer:~/catkin_ws/learning_tf$ cd nodes
user@computer:~/catkin_ws/learning_tf/nodes$
```

자신이 선호하는 편집기를 가지고 ~/catkin_ws/src/learning_tf/nodes 폴더에 아래 코드와 같이 **turtle_tf_broadcaster.py** 파일을 작성한다.


```python
#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```

코드 작성 후, 잊지말고 작성한 코드에 실행속성을 부여해 준다.

```
user@computer:~/catkin_ws/learning_tf/nodes$ chmod +x turtle_tf_bradcaster
```



#### 1.2 코드 설명

거북이의 pose 를 tf 에게 publish 하는 것과 연관된 코드를 살펴보자

이 노드는 'turtle1', 'turtle2' 같은 거북이 이름에 해당하는 'turtle' 파라매터 하나만을 취한다.

```python
turtlename = rospy.get_param('~turtle')
```

그리고 매 'turtle**n**/pose' 토픽( n = 1, 2,... ) subscribe하기위한 subscriber를 정의한다.  

```python
    rospy.Subscriber('/%s/pose' % turtlename, # 토픽 이름
                     turtlesim.msg.Pose,      # 토픽 형식
                     handle_turtle_pose,      # 핸들 함수
                     turtlename)
```

핸들함수에는 tf 브로드캐스터가 선언한다. 거북이의 pose 메세지 브로드캐스팅을 위한 핸들 함수는 turtlename 거북이의 tf 변환과 회전을 'world' 프레임으로부터 거북이이름의 프레임으로 앞서 선언한 브로드캐스터 br을 통해 브로드캐스팅한다.

```python
def handle_turtle_pose(msg, turtlename):# 거북이 이름을 매개변수로 받는 핸들함수 정의
	br = tf.TransformBroadcaster()          # tf 브로드캐스터 br 선언
    # br.sendTransform(translation, rotation, time, childframe, parentframe)
	br.sendTransform((msg.x, msg.y, 0),     # translation
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta), # rotation
                     rospy.Time.now(),		# time
                     turtlename,			# child 프레임
                     "world")				# parent 프레임
```



### 2. broadcaster 의 실행

이제 이 데모코드를 실행할 launch 파일 만들 순서다. ~/catkin_ws/src/learning_tf/launch 폴더를 만들고 그 안에 **start_demo.launch** 파일을 다음과 같이 작성하라.

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

  </launch>
```

우선 앞 서 진행했던 튜토리얼의 launch 파일이 실행 중이라면 ctrl-c 를 입력하여 그것을 먼저 종료한다. 이제 직접 작성한 거북이 broadcaster를 시작할 준비가 완료되었다.

```
user@computer:~/catkin_ws$ roslaunch learning_tf start_demo.launch
```

turtlesim 노드가 실행되고 거북이가 한 마리 보일 것이다.



### 3. 결과 확인

**tf_echo** 툴을 이용하여 실제로 거북이의 pose 가 tf 로 publish 되고 있는가 체크한다.

```
user@computer:~/catkin_ws$ rosrun tf tf_echo /world /turtle1
```

위의 명령은 첫 번 째 거북이의 pose 값을 보여준다. 키보드를 이용하여 거북이를 이리저리 움직여 보라. 만일 같은 명령을 /world 와 /turtle2 에 대해 수행한다면 하나의 transform 도 보이지 않을 것이다. 왜냐하면 두 번 째 거북이는 아직 거기 없기 때문이다. 하지만 곧 다음 튜토리얼에서 두 번 째 거북이를 추가할 것이고, 두 번 째 거북이의 pose 값도 tf 로 broadcast 될 것이다.



[튜토리얼 목록 열기](../README.md)



[다음 튜토리얼](./tf_2_listener.md)

[이전 튜토리얼](./tf_0_Instroduction.md)

