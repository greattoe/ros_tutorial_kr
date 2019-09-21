## rospy_tutorial/ Tutorials/ Makefile

[TOC]

---

## ROS 파이썬 Makefile 작성 

**출처 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile>

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---

이 튜토리얼은 이 전에 이미 이 전 튜토리얼에서 다루었던 내용을 Makefile에 초점을 맞춰 반복한다. makefile 및 CMakelists.txt에 대한 정보가 필요하지 않다면 이 튜토리얼을 건너 뛰어도 된다.



### 1. Intro

```CMakeLists.txt``` 와 ```Makefile``` 은 매우 간단하지만 다음과 같은 중요한 기능을 제공합니다.

* 메세지 및 서비스코드 자동 생성
* 테스트 수행

테스트 수행 기능은 사용자 작성 패키지와 사용자 작성 패키지가 의존하는 다른 모든 패키지들에 대한 테스트( 실제 운영 이전에  regression( 재귀, 회귀 )을 찾을 수 있는 테스트 ```rospack pkg test``` ) 수행능력을 가진다는 점에서 매우 중요하다. 



```catkin``` 빌드환경에서 필요한 것은 ```CMakeLists.txt``` 파일 뿐이다. ```CMakeLists.txt``` 파일은 ```build``` 폴더에 ```Makefile``` 을 생생한다. 이를 위해  ```package.xml``` 에는 다음 내용이 반드시 기재되어 있어야 한다.


```
<bildtool_depend>catkin</buildtool_depend>
```

다음처럼 ```catkin workspace``` 폴더에 새로운 패키지를 만들 수 있다.

```
user@computer:~$ cd ~/catkin_ws/src
user@computer:~/catkin_ws/src$ catkin_create_pkg my_pkg message_generation rospy
```

위 명령을 수행하면 ```my_pkg``` 라는 새로운 패키지가 생성되고,  ```message_generation``` 과 ```rospy``` 에대한 의존성이  ```my_pkg``` 에 추가되어 코드 작성시 사용할 수 있게 된다.

이 튜토리얼은 ```message_generation``` 을 사용한다. 이 ```message_generation``` 은  ```catkin``` 패키지가 새로운 ROS 메시지나 서비스를 정의하여 사용하는 경우에만 필요하다.

```my_pkg``` 의 ```CMakeLists.txt``` 파일에는 적어도 다음 내용들이 반드시 기재되어 있어야 한다. 

```
cmake_minimum_required(VERSION 2.8.3)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS message_generation rospy ...)
catkin_package()
```



#### 1.1 Messages 와 Services

```Messages``` 또는  ```Services``` 를 추가하기 위해서는 ```package.xml``` 파일과 ```CMakelists.txt``` 파일에 몇 가지의 추가할 항목들이 있다.

```message_generation``` 을 사용하기 위해서  ```package.xml``` 에는 다음 내용이 반드시 기재되어 있어야 한다. 

```
<bild_depend>message_generation</build_depend>
```

