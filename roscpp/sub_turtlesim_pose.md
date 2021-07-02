## turtlesim 포즈 구독 노드 작성

**참고자료 :** <http://wiki.ros.org/turtlesim>



---

### 1. turtlesim_node 구동 및 /turtle1/pose 토픽 확인


roscore 실행

```bash
$ roscore 
```

turtlesim 노드 실행

```bash
$ rosrun turtlesim turtlesim_node
```
rostopic list 실행

```bash
$ rostopic list
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

`rostopic type [토픽명]`  명령으로  `/turtle1/pose` 토픽의 정보 확인

```bash
$ rostopic type /turtle1/pose 
turtlesim/Pose
```

`rostopic echo` 명령으로 `/turtle1/Pose` 토픽 내용을 화면에 출력해보자.

```bash
$ rostopic echo /turtle1/pose 
x: 5.544444561
y: 5.544444561
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
x: 5.544444561
y: 5.544444561
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

여기까지 알아낸 것을 정리해 보면

1. `/turtle1/pose` 토픽은 `turtlesim/Pose` 형식이다.
2. `turtlesim/Pose` 형식은 `x` ,  `y` ,  `theta` ,  `linear_velocity` ,  `angular_velocity` 라는 구성요소를 가지고 있다.

라는 것을 알 수 있다. 이를 바탕으로 `/turtle1/pose` 토픽 Subscriber 노드를 작성하여 `pkg_4_turtlesim` 패키지에 추가해보자.



---

### 2. `/turtle1/pose` 토픽 Subscriber 노드 작성

`turtlesim` 의 거북이 제어와 관련된 새로운 사용자 패키지 `test_turtlesim` 을 만들고 거북이를 키보드로 제어하는 노드 를 추가하기 위해 노드명, 토픽명, 소스 파일명을 다음과 같이 미리 정했다. ( `package.xml` 과 `CMakeList.txt` 수정 작업 시 혼란을 피하기 위해 )

**노  드  명:** `sub_turtlesim_pose`

**토  픽  명:** `/turtle1/pose`

**토픽형식:** `turtlesim/Pose`

**pkg    명:** `pkg_4_turtlesim`

**파  일  명:** `~/catkin_ws/src/pkg_4_turtlesim/src/sub_turtlesim_pose.cpp`

**기       능:** `turtlesim` 의 실시간 포즈( `x` ,  `y` ,  `theta` )를 화면에 출력한다. 

#### 3.1 `sub_turtlesim_pose.cpp` 작성

`~/catkin_ws/src/pkg_4_turtlesim/src` 로 경로 변경

```bash
$ roscd ~/pkg_4_turtlesim/src
```

`sub_turtlesim_pose.cpp` 파일 편집

```bash
$ gedit sub_turtlesim_pose.cpp &
```

```c++
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <stdio.h>

void cb_get_pose(const turtlesim::Pose& msg) {
  printf("x = %f, y = %f, theta = %f", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sub_turtlesim_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, cb_get_pose);
  ros::spin();
  return 0;
}
```



#### 2.3 CMakeList.txt 편집

의존성들을 확인하고,  `add_executable` 항목과 `target_link_libraries` 항목을 설정 한다.

```makefile
cmake_minimum_required(VERSION 2.8.3)
project(roscpp_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# 일부 생략

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_1st_pkg
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_1st_pkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_1st_pkg_node.cpp)
#                --------------------     -------------------
#       노드명 -------------^   소스코드명 -----------^
# add_executable(노드명 src/소스코드명.cpp)
add_executable(teleop_turtlesim   src/teleop_turtlesim.cpp  )
add_executable(sub_turtlesim_pose src/sub_turtlesim_pose.cpp) # 여기에 추가

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} )
#                       --------------------
#              노드명 -------------^
add_executable(teleop_turtlesim   src/teleop_turtlesim.cpp  )
add_executable(sub_turtlesim_pose src/sub_turtlesim_pose.cpp) # 여기에 추가

#############
## Install ##
#############

# 이하 생략
```



#### 2.4 빌드 및 실행

1. `$ cd ~/catkin_ws` 
2. `$ catkin_make` 
3. `$ source ./devel/setup.bash` 
4. `$ rospack profile` 
5. `$ roscore` 
6. `$ rosrun turtlesim turtlesim_node` 
7. `$ rosrun pkg_4_turtlesim teleop_turtlesim` 
8. `$ rosrun pkg_4_turtlesim sub_turtlesim_pose` 

```bash
$ rosrun pkg_4_turtlesim sub_turtlesim_pose
x: 0.924138307571
y: 1.11112904549
theta: 3.16644406319
linear_velocity: 0.0
angular_velocity: 0.0
---
x: 0.924138307571
y: 1.11112904549
theta: 3.16644406319
linear_velocity: 0.0
angular_velocity: 0.0
---
```





---

[^1]:**rospack** : ROS 패키지 관리 도구 rospack은 dpkg 와 pkg-config 의 일부분이다. rospack의 주된 기능은 ROS_ROOT 및   ROS_PACKAGE_PATH의 패키지를 크롤링하고 각 패키지의 manifest.xml을 읽고 구문을 분석하며 모든 패키지에 대한 완전한 의존성 트리를 구성하는 것이다.
[^2]: **rospack profile** : rospack의 성능은 매니페스트 파일을 포함하지 않는 매우 광범위하고 깊은 디렉토리 구조의 존재로 인해 악영향을 받을 수 있다. 이러한 디렉토리가 rospack의 검색 경로에 있으면, 패키지를 찾을 수 없다는 것을 발견하기 위해서만 크롤링하는 데 많은 시간을 소비 할 수 있다. 이러한 디렉토리에 rospack_nosubdirs 파일을 작성하여이 대기 시간을 방지 할 수 있다. rospack이 성가신 속도로 느리게 실행되는 것처럼 보이는 경우 profile 명령을 사용하여 크롤링 할 가장 느린 20 개의 트리를 인쇄하거나 profile --length = N을 사용하여 가장 느린 N 개의 트리를 인쇄 할 수 있다.

