## kinetic / Installation & configure / Ubuntu

---

## ROS 설치 및 환경설정

**튜토리얼 레벨 :**  Beginner(초급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**참조 :**

- <http://wiki.ros.org/kinetic/Installation/Ubuntu>
- <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>
- <http://wiki.ros.org/ROS/Tutorials/BuildingPackages>

**다음 튜토리얼 :** [catkin 빌드환경에서의 rospy 사용법(1/2)](../rospy/rospy_1_How2UsePythonWithCatkin_1.md) 

**튜토리얼 목록 :** [README.md](../README.md)

---

우분투 16.04 LTS 환경에서의 ROS Kinetic Kame 버전 설치 방법은 다음과 같다.



### 0. Ubuntu 16.04 LTS 설치

[Ubuntu 16.04 LTS 설치](../ubuntu16/install_ubuntu_1604_lts.md)



### 1. 설치

ROS Kinentic 은 우분투의 경우 wily(15.10) 와 Xenial(16.04), 데비안(Debian)의 경우 Jessie 버전에서 운영 가능하다.



#### 1.1 우분투 원격저장소(Repository) 설정

설정이 허용되는 우분투 원격저장소(repository) 타입은 "restricted", "universe", "multiverse" 이다. 자세한 사항은 [우분투 가이드](https://help.ubuntu.com/community/Repositories/Ubuntu)를 참조한다.



#### 1.2 ROS 저장소 등록

ROS 패키지 저장소(repository) 주소를 저장소 리스트에  등록한다.

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```



#### 1.3 key 설정

리포지토리에 접속에 필요한 key를 key 서버로부터 받아 등록한다.

```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

key  서버 접속에 문제가 있다면 `hkp://pgp.mit.edu:80` 를 `hkp://keyserver.ubuntu.com:80` 대신에 시도해 볼 수 있다. 

또는 위 명령대신 다음 명령을 실행할 수 있다.

```
$ curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```



#### 1.4 설치

우선 변경된 저장소 목록의 내용을 반영하기 위해 데비안 패키지 인덱스 업데이트를 실행한다.

```
$ sudo apt-get update
```

업데이트를 마친 후 Desktop-full / Desktop / ROS-Base(Bare bones) 설치 방법 중 자신의 ROS 운영 목적에 맞는 방법을 선택하여 설치한다.

**Desktop-full 설치**

일반적으로 권장되는 설치방법으로 ROS, rqt, rviz, 일반적인 로봇 라이브러리, 2D/3D 시뮬레이터, 네비게이션, 2D/3D 인식 관련 패키지들이 함께 설치된다.

```
$ sudo apt-get install ros-kinetic-desktop-full
```

**Desktop 설치**

ROS, rqt, rviz, 일반적인 로봇 라이브러리 패키지들이 함께 설치된다.

```
$ sudo apt-get install ros-kinetic-desktop
```

**ROS-Base(Bare bones) 설치**

GUI 도구를 제외한 ROS 패키지와 빌드 및 통신관련 라이브러리들만 설치된다.

```
$ sudo apt-get install ros-kinetic-ros-base
```

**개별 패키지 설치**

`Desktop-full 설치` 이 외의 방법으로 ROS 설치 후, 특정 패키지를 추가 설치할 필요가 있을 경우 다음과 같이 설치한다. 

```
$ sudo apt-get install ros-kinetic-패키지명
```

예를 들어 `ar-track-alvar` 패키지를 추가 설치 한다면, 다음과 같이 설치한다.

```
$ sudo apt-get install ros-kinetic-ar-track-alvar
```

추가로 설치할 수 있는 ROS 개별 패키지 목록은 다음 명령을 실행하여 알아낼 수 있다.

```
$ apt-cache search ros-kinetic
```



#### 1.5 rosdep 초기화

`rosdef` 는 컴파일하려는 ROS 코드의 시스템 의존성  ROS를 사용하기 전에 rosdep 를 초기화해야한다. rosdep을 사용하면 컴파일하려는 소스에 대한 시스템 종속성을 쉽게 설치할 수 있다. 이를 위해서 ROS에서 일부 핵심 구성 요소를 실행해야 한다.

```
$ sudo rosdep init
$ rosdep update
```



#### 1.6 roscore 실행 

- ROS 환경변수 반영

`roscore` 를 실행하려면 ROS 환경 변수가 반영되어 있어야 한다. bash 쉘을 사용한다면 터미널 창을 열고 다음 명령을 실행한다.

```
$ source /opt/ros/kinetic/setup.bash
```

- roscore 실행


```
$ roscore
```



#### 1.7 빌드 의존성 설치 

ROS 노드 패키지 빌드 의존성 패키지들을 설치하려면 다음 명령을 실행한다.

```
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```





### 2. ROS 환경 설정

앞서 ROS를 설치하고,  `roscore` 를 실행하여 정상 설치 여부를 확인했다. 이 번에는 원활한 ROS 운영을 위해 주로 사용하는 환경설정에 대해 알아본다. 



#### 2.1 기본 환경 설정

**1.6 roscore 실행** 에서 `roscore` 구동 전에 실행한  `source /opt/ros/kinetic/setup.bash` 명령은 ROS 관련 환경변수를 현재 열린 터미널 환경에 반영( 기본 ROS 명령어와 `apt-get install ros-kinetic-[패키지명]` 명령으로 설치한 ROS 패키지의 실행이 가능하도록 )시킨다. 

```
$ source /opt/ros/kinetic/setup.bash
```



#### 2.2 catkin 빌드환경 설정

catkin 빌드환경으로 작성한 코드를 빌드하기 위한 설정은 다음과 같다.

작업 폴더( workspace 로 사용할 ) catkin_ws  폴더와 그 하위 폴더 src 생성

```
$ mkdir -p ~/catkin_ws/src
```

생성된 `src` 폴더로 작업 경로 변경

```
$ cd ~/catkin_ws/src
```

`catkin` 작업 폴더 초기화( 이 초기화 작업은 새 

```
catkin_init_workspace
```

테스트 빌드를 위한 작업경로 변경

```
$ cd ~/catkin_ws
```

`catkin_make` 명령으로 테스트 빌드

```
$ catkin_make
```

실제로 소스코드를 작성하여 빌드했다면 새로 빌드된 패키지 정보가 포함된 ROS 환경변수가 실행 중인 터미널 환경에 반영되어야 한다. `source` 명령을 이용하여 다음과 같이 실행한다.

```
$ source ~/catkin_ws/devel/setup.bash
```



#### 2.3 ROS 네트워크 환경 설정

ROS는 기본적으로 TCP/IP 기반 의 메시지 통신을 바탕으로 운영되므로 네트워크 설정에 오류가 있을 경우 작동할 수 없다. ROS 네트워크 설정은 `roscore` 노드가 구동되는 마스터 PC의 주소를 나타내는 `ROS_MASTER_URI` 와 로봇에 탑재된 ROS가 설치된 호스트 PC의 주소를 나타내는 `ROS_HOSTNAME` 을 설정해 주는 것으로 `export` 명령을 사용하여 설정한다. 

다음은 마스터 PC의 주소가 192.168.0.101, 호스트 PC의 주소가 192.168.0.102 인 경우의 ROS 네트워크 설정 예이다.

```
$ export ROS_HOSTNAME=192.168.0.102
$ export ROS_MASTER_URI=http://192.168.0.101:11311
```

한 대의 PC가 마스터 PC와 호스트 PC의 역할을 수행할 경우 `localhost`( 네트워크에서 기기 자신을 가리키는 도메인 )를 이용하여 설정할 수 있으며, 다음은 그 예이다.

```
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
```

위 사례들에서 보여지듯이 `ROS_HOSTNAME`  설정은 IP 주소만을 사용하지만 `ROS_MASTER_URI` 설정에는 반드시 네트워크 프로토콜( http:// )과 포트번호( :11311 )가 포함되어 있어야 한다는 것에 주의한다.



#### 2.4 '~/.bashrc' 파일 편집

새 터미널 창을 열 때 마다 실행해주어야 하는 `source ...` , `export ...` 등의 명령을 사용자 환경이 기록되있는 파일인 `.bashrc`파일에 등록하여 터미널 창을 열 때 자동으로 실행되도록 하자.

```
$ gedit ~/.bashrc
```

다음 내용을 `~/.bashrc` 파일의 마지막에 추가한다.

```bash
	.
	.
	.
# Configuration for ROS
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

alias cs='cd ~/catkin_ws/src'
alias cw='cd ~/catkin_ws'
alias cm='cd ~/catkin_ws && catkin_make'
alias sb='source ~/.bashrc'
```

변경된 `~/.bashrc` 파일의 내용이 반영되려면 열려진 터미널 창을 종료 후 다시 실행하거나 다음처럼 `source` 명령을 이용하여 적용할 수 있다.

```
$ source ~/.bashrc
```



[이전 튜토리얼](../ubuntu16/install_ubuntu_1604_lts.md) 

[다음 튜토리얼](../rospy/rospy_1_How2UsePythonWithCatkin_1.md) 

[튜토리얼 목록](../README.md) 

