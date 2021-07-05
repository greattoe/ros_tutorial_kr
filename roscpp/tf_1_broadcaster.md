## tf/ Tutorials/ Writing a tf broadcaster (C++)



---


## tf broadcaster 작성 방법 (C++)

이 후의 두 튜토리얼에서 [tf introduction](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) 튜토리얼의 데모와 같이 동작하는 코드를 작성할 것이다. 그리고 나서 tf 의 고급 기능을 이용한 데모 코드의 기능 확장에 집중할 것이다.

시작하기 전에 이 프로젝트를 위한  tf, [roscpp](http://wiki.ros.org/roscpp), [rospy](http://wiki.ros.org/rospy) and [turtlesim](./turtlesim.md)에 대한 의존성을 갖는 learning_tf 라는 새로운 ROS 패키지를 만든다.

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
```

아무 내용도 작성하지 않은 빈 패키지만 roscd 명령으로 이동할 수 있도록 빌드해두자.

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash
```



### 1. transforms 를 broadcast 하는 방법 

이 튜토리얼은 tf 에게 좌표 프레임을 broadcast 하는 방법을 가르쳐 줄 것이다. 이 경우 거북이들의 이동에 따른 그들의 좌표 프레임의 변화를 broadcast 하기를 바랄 것이다.

첫 번째 코드를 작성하기 위해 조금 전 생성한 패키지 폴더로 이동하자

```bash
$ roscd learning_tf
```



#### 1.1 코드

자신이 선호하는 편집기를 가지고 ~/catkin_ws/src/learning_tf/src 에 아래 코드와 같이 **turtle_tf_bradcaster.cpp** 파일을 작성한다.

<https://raw.github.com/ros/geometry_tutorials/hydro-devel/turtle_tf/src/turtle_tf_broadcaster.cpp>

```bash
$ roscd learning_tf
```

```c++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
```



#### 1.2 코드 설명

거북이의 pose 를 tf 에게 publish 하는 것과 연관된 코드를 살펴보자

tf 패키지는 transform 을 publish 하는 작업을  쉽게 하도록 돕기위해 TransformBroadcaster 의 구현 방법을 제공하다. 이것을 이용하기 위해서는 **tf/transform_broadcaster.h** 헤더파일을 반드시 include 시켜야 한다.

```c++
#include <tf/transform_broadcaster.h>
```

나중에 네트워크를 통해 transforms 전송에 사용하게 될 TransformBroadcaster 객체 br 을 생성한다.

```c++
  static tf::TransformBroadcaster br;
```

이제 Transform 객체 transform 을 생성하고, 거북이의 2D pose 값을 3D transform 으로 복사한다. 

```c++
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta); 
```

rotation 을 설정한다.

```c++
  transform.setRotation(q);
```

이 부분이 실제 작업이 수행되는 부분이다. TransformBroadcaster 가 요구하는 4개의 arguments 와 같이 transform을 전송하라.

```c++
  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));
```

1. 첫 번 째 할일은 transform 자체를 전달하는 것이다.
2. 이제 publish 되고 있는 transform 에게 `ros::Time::now()`를 이용하여 현재 시간을 timestamp 로 부여하는 것이다.
3. 그리고 나서, 우리가 만든 링크의 parent 프레임 이름( 이 번 경우 "world" )을 전달해야 한다.
4. 마지막으로, 우리가 만든 링크의 child 프레임 이름( 이 번 경우 거북이 이름 그 자체 )을 전달해야 한다.

*Note :  sendTransform 과 StampedTransform 의 parent 와 child 프레임은 서로 순서가 반대이다.*



### 2. broadcaster 의 실행

코드 작성을 마쳤으면 일단 빌드를 위해 CMakeList.txt 파일을 열어 다음 라인을 추가한다.

```makefile
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
```

~/catkin_ws 에서 catkin_make 를 실행하여 빌드한다.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

문제 없이 빌드를 마쳤다면 `~/catkin_ws/devel/lib/learning_tf` 폴더 안에  **turtle_tf_broadcaster** 라는 이름의 바이너리 파일이 만들어졌을 것이다.

그렇다면 이제 이 데모코드를 실행할 launch 파일 만들 순서다. `~/catkin_ws/src/learning_tf/launch` 폴더를 만들고 그 안에 **start_demo.launch** 파일을 다음과 같이 적성하라.

```xml
  <launch>
    <!-- Turtlesim Node-->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    <!-- Axes -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <node pkg="learning_tf" type="turtle_tf_broadcaster"
          args="/turtle1" name="turtle1_tf_broadcaster" />
    <node pkg="learning_tf" type="turtle_tf_broadcaster"
          args="/turtle2" name="turtle2_tf_broadcaster" />

  </launch>
```

우선 앞 서 진행했던 튜토리얼의 launch 파일이 실행 중이라면 ctrl-c 를 입력하여 그것을 먼저 종료한다. 이제 직접 작성한 거북이 broadcaster를 시작할 준비가 완료되었다.

```bash
$ roslaunch learning_tf start_demo.launch
```

turtlesim 노드가 실행되고 거북이가 한 마리 보일 것이다.



### 3. 결과 확인

**tf_echo** 툴을 이용하여 실제로 거북이의 pose 가 tf 로 publish 되고 있는가 체크한다.

```bash
$ rosrun tf tf_echo /world /turtle1
```

위의 명령은 첫 번 째 거북이의 pose 값을 보여준다. 키보드를 이용하여 거북이를 이리저리 움직여 보라. 만일 같은 명령을 `/world` 와 `/turtle2` 에 대해 수행한다면 하나의 transform 도 보이지 않을 것이다. 왜냐하면 두 번 째 거북이는 아직 거기 없기 때문이다. 하지만 곧 다음 튜토리얼에서 두 번 째 거북이를 추가할 것이고, 두 번 째 거북이의 pose 값도 tf 로 broadcast 될 것이다.

[튜토리얼 목록 열기](../README.md)

[다음 튜토리얼](./tf_3_listener.md)