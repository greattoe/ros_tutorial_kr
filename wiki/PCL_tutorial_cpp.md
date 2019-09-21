## pcl/ Tutorials

[TOC]
---



## How to use a PCL tutorial in ROS



### 1. Create a ROS package 

```
user@computer:~/catkin_ws/src$ catkin_create_pkg pcl_conversions pcl_ros roscpp sensor_msgs
```

패키지를 만들었으면 pckage.xml 다음 행들을 추가한다.

```
<build_depend>libpcl-all-dev</build_depend>
  <exec_depend>libpcl-all</exec_depend>
```



### 2. Create code skeleton



#### 1.1 코드

자신이 선호하는 편집기를 가지고 ~/catkin_ws/src/learning_tf/src 에 아래 코드와 같이 **turtle_tf_listener.cpp** 파일을 작성한다.

<https://raw.github.com/ros/geometry_tutorials/groovy-devel/turtle_tf/src/turtle_tf_listener.cpp>

```c++
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = 
    node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    turtlesim::Velocity vel_msg;
    vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
```

위의 코드는 tf 튜토리얼을 끝마치는 데에 있어 필수적이다. 그리고 ROS Hydro 환경에서 절대 컴파일 해서는 않된다. 몇 가지 토픽이나 메세지명에 변화가 생겼기 때문이다. turtlesim/Velocity.h 헤더파일은 더 이상 사용되지 않는다. 현재 이 헤더파일은 geometry_msgs/Twist.h 로 대체되었다. 또 /turtle/command_velocity 토픽 또한 현재는 /turtle/cmd_vel 로 바꾸었다. 이런 이유로 제대로 동작 시키기 위해 약간의 수정이 필요하다.

<https://raw.github.com/ros/geometry_tutorials/hydro-devel/turtle_tf/src/turtle_tf_listener.cpp>

```c++
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
```

만일 실행 시 에러가 발생한다면 listener 호출 하는 부분을 다음 코드로 대신하여 시도해 보시오.  :

```c++
try {
    listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
```



#### 1.2 코드 설명

거북이의 pose 를 tf 에게 publish 하는 것과 연관된 코드를 살펴보자

tf 패키지는 transform 을 수신하는 작업을  쉽게 하도록 돕기위해 TransformListener 의 구현 방법을 제공하다. 이것을 이용하기 위해서는 **tf/transform_listener.h** 헤더파일을 반드시 include 시켜야 한다.

```c++
#include <tf/transform_broadcaster.h>
```

여기서 TransformListener 객체를 생성한다. listener 가  한 번 만들어지면 네트워크를 통해 tf transformation 수신을 시작하며, buffer 에는 최대 10초까지의 수신 데이터를 담아 둘 수 있다. TransformListener 객체는 그 범위를 유지해야한다. 그렇지 않을 경우 그 캐시는 채워질 수 없고 거의 모든 쿼리가 실패하게 될 것이다. 일반적인 방법은 TransformListener 객체를 클래스의 멤버 변수로 만드는 것입니다.

```c++
  tf::TransformListener listener;
```

이 지점에서 실제 작업이 이루어지며, 특정 transformation 을 위한 쿼리를 listener 에게 요청한다. 4개의 argumet 들을 살펴보자.

1. /turtle1 프레임으로부터 /turtle2 프레임으로의 변환 요청이다.
2. 시간은 transform 을 원하는 바로 그 시간이다.  `ros::Time(0)`를 제공함으로써 가장 최근의 변환 시간을 알 수 있다.
3. 객체에는 변환 결과가 저장된다.

이 모든 것을 가능한 예외를 처리하기 위해 try - catch 블럭으로 둘러싼다. 

```c++
  try{
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
  }
```

여기서 transform 은 첫번째 거북이로부터의 거리와 각도에 기초한 두 번 째 거북이를 위한 새로운 선속도와 각속도를 구하는데 사용된다. 새로 구해진 속도는 "turtle2/cmd_vel" 토픽으로 publish 되고, turtlesim 은 두 번 째 거북이의 움직임을 업데이트 하는 데에 사용할 것이다. 

```c++
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.linear.x  = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
```



### 2. listener 의 실행

코드 작성을 마쳤으면 일단 빌드를 위해 CMakeList.txt 파일을 열어 다음 라인을 추가한다.

```shell
add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
```

~/catkin_ws 에서 catkin_make 를 실행하여 빌드한다.

```
user@computer:~/catkin_ws/src/learning_tf$ cd ~/catkin_ws
user@computer:~/catkin_ws$ catkin_make
```

문제 없이 빌드를 마쳤다면 ~/catkin_ws/devel/lib/learning_tf 폴더 안에  **turtle_tf_listener** 라는 이름의 바이너리 파일이 만들어졌을 것이다.

그렇다면 이제 이 데모코드를 실행할 launch 파일 만들 순서다. ~/catkin_ws/src/learning_tf/launch 폴더의 **start_demo.launch** 파일을 열고 아래의 노드 블럭을 `<launch>` 블럭 안에 추가하라.

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
    <!-- 여기에 추가 -------------------------------------->
    <node pkg="learning_tf" type="turtle_tf_listener"
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