## bebop_autonomy / Install



---

## Install bebop_autonomy

**튜토리얼 레벨 :**  Intermediate(중급)(수정)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [Parrot-Sphinx](./bb2_2_parrot_sphinx.md)

**자료원본(출처) :** <https://bebop-autonomy.readthedocs.io/en/latest/>

**목록보기:** [README.md](../README.md)

---

이 문서는 [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/) 를 참조하여, `catkin_make` 환경에서 `bebop_autonomy` ROS 패키지를 설치하는 방법을 설명한다. 



### 1. 설치 전 요구사항

* ROS Indigo, jade or Kinetic (우분투 환경에서만 테스트 되었음)

* ROS 패키지 빌드를 위한 기본적인 지식

* `build-esstential` ,  `python-rosdep` ,  `python-catkin-tools`  우분투 패키지

  ```bash
  $ sudo apt-get install build-essential python-rosdep python-catkin-tools
  ```

  

### 2. 설치( 소스코드로부터 빌드 ) 

catkin 워크스페이스의 src 폴더로 작업경로 변경

```bash
$ cd ~/catkin_ws/src
```

bebop_autonomy Github 로부터 소스코드 복제

```bash
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

catkin 워크스페이스로 작업경로 변경

```bash
$ cd ~/catkin_ws
```

빌드

```bash
$ catkin_make
```

빌드 중 다음과 같은 에러가 발생할 경우,

```bash
-- Could not find the required component 'parrot_arsdk'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "parrot_arsdk" with
  any of the following names:

    parrot_arsdkConfig.cmake
    parrot_arsdk-config.cmake

  Add the installation prefix of "parrot_arsdk" to CMAKE_PREFIX_PATH or set
  "parrot_arsdk_DIR" to a directory containing one of the above files.  If
  "parrot_arsdk" provides a separate development package or SDK, be sure it
  has been installed.
Call Stack (most recent call first):
  bebop_autonomy/bebop_driver/CMakeLists.txt:7 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/username/catkin_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/username/catkin_ws/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
$ 
```

`ros-kinetic-parrot-arsdk` 패키지를 설치 후,

```bash
$ sudo apt-get install ros-kinetic-parrot-arsdk
```

다시 빌드한다.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

간혹, 아래와 같은 에러가 발생하면, 

```bash
[ 96%] Generating C++ code from bebop_msgs/Ardrone3PilotingStateAlertStateChanged.msg
[ 96%] Generating C++ code from bebop_msgs/Ardrone3PilotingStateFlatTrimChanged.msg
[ 97%] Generating C++ code from bebop_msgs/CommonChargerStateMaxChargeRateChanged.msg
[ 97%] Generating C++ code from bebop_msgs/CommonMavlinkStateMissionItemExecuted.msg
[ 97%] Generating C++ code from bebop_msgs/CommonCommonStateProductModel.msg
[ 97%] Generating C++ code from bebop_msgs/CommonAnimationsStateList.msg
[ 97%] Generating C++ code from bebop_msgs/CommonCommonStateMassStorageInfoStateListChanged.msg
[ 98%] Built target bebop_msgs_generate_messages_cpp
[ 98%] Linking CXX executable
Makefile:138: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j8 -l8" failed
$
```

다시 한 번 빌드한다.

```bash
$ catkin_make
```







[튜토리얼 목록 열기](../README.md)                                                [다음 튜토리얼](./bb2_2_parrot_sphinx.md)