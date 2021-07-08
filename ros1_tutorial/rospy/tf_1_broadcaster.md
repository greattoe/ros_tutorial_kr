## tf/ Tutorials/ Writing a tf broadcaster (Python)



---


## tf broadcaster 작성 방법 (Python)

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [tf( trnasform ) 소개](./tf_0_Instroduction.md)

**다음 튜토리얼 :** [tf listener 작성](./tf_2_listener.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29>

------

이 후의 두 튜토리얼에서 [tf introduction](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) 튜토리얼의 데모와 같이 동작하는 코드를 작성할 것이다. 그리고 나서 tf 의 고급 기능을 이용한 데모 코드의 기능 확장에 집중할 것이다.

시작하기 전에 이 프로젝트를 위한  tf, [roscpp](http://wiki.ros.org/roscpp), [rospy](http://wiki.ros.org/rospy) and [turtlesim](./turtlesim.md)에 대한 의존성을 갖는 learning_tf 라는 새로운 ROS 패키지를 만든다.

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
```

아무 내용도 작성하지 않은 빈 패키지이지만 roscd 명령으로 이동할 수 있도록 미리 빌드한다.

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash
```



### 1. `tf`를 이용한 `broadcaster` 작성 

이 문서는 `tf` 를 이용하여 좌표 프레임( coordinate frames )을 `broadcast` 하는 방법을 `turtlesim` 패키지를 이용하여 설명한다. 그럼 `turtlesim` 패키지 거북이들의 이동에 따라 변화하는 거북이들의 좌표 프레임의 변화를 `broadcast` 하는 코드를 작성해보자.

`broadcaster` 노드 작성을 위해 앞서 만든 `learning_tf` 패키지 폴더로 경로를 변경한다. 

```bash
$ roscd learning_tf
```



#### 1.1 코드 작성

일단 `learning_tf` 패키지에 `nodes` 폴더를 만들고 그 안으로 경로를 변경한다. 

```bash
$ mkdir nodes
$ cd nodes$
```

`turtle_tf_broadcaster.py` 파일을 만들고, 실행속성을 부여한다. 

```bash
$ touch turtle_tf_broadcaster.py
$ chmod +x turtle_tf_broadcaster.py
```

`turtle_tf_broadcaster.py` 파일 편집

```bash
$ gedit turtle_tf_broadcaster.py &
```


```python
#!/usr/bin/env python  

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
    rospy.init_node('turtle_tf_broadcaster')	#
    turtlename = rospy.get_param('~turtle')		# 
    rospy.Subscriber('/%s/pose' % turtlename,	# 
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```



#### 1.2 코드 설명

이제 작성된 코드내용 중 거북이의 `pose` 를 `tf` 로 `publish` 하는 것과 관련된 부분을 살펴보자

아래는 `turtle1`, `turtle2` 같은 거북이 이름에 해당하는 `turtle` 파라메터(`parameter`)로 부터 가져온 값을 `turtlename` 변수에 치환하는 과정이다. 

```python
turtlename = rospy.get_param('~turtle')		
```

이 노드는 새로운 `turtle1/pose` ( 또는`turtle2/pose` 나, `turtle3/pose` 등 )인 토픽이 `publish` 될 때 마다, 그 토픽을 `subscribe` 하고 `handle_turtle_pose` 함수를 호출한다.  다음은 그 것을 위한 `subscriber` 를 선언 과정이다.

```python
    rospy.Subscriber('/%s/pose' % turtlename, # 토픽이름 /turtle1/pose or /turtle2/pose ...
                     turtlesim.msg.Pose,      # 토픽형식
                     handle_turtle_pose,      # 핸들함수(콜백+브로드캐스트)
                     turtlename)
```

핸들러 함수 `handle_turtle_pose` 는  이 거북이의 `transform` 및 `rotation` 을 `world` 프레임으로부터 `turtle1`( 또는 `turtle2` 등 ) 프레임으로의 `transform` 으로 `publish` 한다. 

```python
def handle_turtle_pose(msg, turtlename):    # 거북이 이름을 매개변수로 받는 핸들함수 정의
	br = tf.TransformBroadcaster()          # tf 브로드캐스터 br 선언
    # br.sendTransform( translation, rotation, time, childframe, parentframe )
    br.sendTransform((msg.x, msg.y, 0),     # translation
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta), # rotation
                     rospy.Time.now(),      # time
                     turtlename,            # child 프레임
                     "world")               # parent 프레임
```



### 2. broadcaster 의 실행

이제 이 데모코드를 실행할 `launch` 파일을 만들기 위해 `learning_tf/launch` 폴더를 만들고 경로를 해당 폴더로 변경한다. 

```bash
$ roscd learing_tf
$ mkdir launch
$ cd launch
```

`start_demo.launch` 파일을 다음과 같이 작성한다. 

```
$ gedit start_demo.launch
```

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



### 3. 결과 확인

**`tf_echo`** 툴을 이용하여 실제로 거북이의 pose 가 tf 로 publish 되고 있는가 체크한다.

```bash
$ rosrun tf tf_echo /world /turtle1
```

위의 명령은 첫 번 째 거북이의 `pose` 값을 보여준다. 키보드를 이용하여 거북이를 이동시키면 다음과 같이 `pose` 값의 변화가 나타나는 것을 알 수 있다. 

```
At time 1621313805.269
- Translation: [6.675, 6.926, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.854, 0.520]
            in RPY (radian) [0.000, -0.000, 2.048]
            in RPY (degree) [0.000, -0.000, 117.342]
At time 1621313806.278
- Translation: [6.029, 8.089, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.978, 0.209]
            in RPY (radian) [0.000, -0.000, 2.720]
            in RPY (degree) [0.000, -0.000, 155.845]
At time 1621313807.269
- Translation: [4.890, 8.600, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.988, -0.157]
            in RPY (radian) [0.000, -0.000, -2.827]
            in RPY (degree) [0.000, -0.000, -161.986]
At time 1621313808.278
- Translation: [3.637, 7.957, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.896, -0.445]
            in RPY (radian) [0.000, -0.000, -2.219]
            in RPY (degree) [0.000, -0.000, -127.150]
```

같은 명령을 `/world` 와 `/turtle2` 에 대해 수행한다면 어떤 `transform` 도 보이지 않는다. 그 이유는 아직 두 번 째 거북이가 나타나지 않았기 때문이다. 하지만 곧 다음 튜토리얼에서 두 번 째 거북이를 추가할 것이고, 두 번 째 거북이의 `pose` 값 역시 `tf` 로 `broadcast` 될 것이다.

[튜토리얼 목록 열기](../README.md)



[다음 튜토리얼](./tf_2_listener.md)

[이전 튜토리얼](./tf_0_Instroduction.md)

