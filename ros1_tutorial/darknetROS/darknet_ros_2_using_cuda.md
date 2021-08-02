## YOLO with ROS



------

## darknet_ros with CUDA

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 18.04 **/** Melodic

**자료원본(출처) :** <https://github.com/leggedrobotics/darknet_ros>

​                           <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html>

**튜토리얼 목록 :** [README.md](../README.md)

------

사용 중인 컴퓨터에 NVIDIA GPU가 있다면, CUDA 를 사용할 수 있다. CUDA는 NVIDIA에서 제공하는 병렬 컴퓨팅 플랫폼 및 프로그래밍 모델로서 GPU(그래픽 처리 장치)의 성능을 활용하여 컴퓨팅 성능을 획기적으로 높일 수 있는 기술이다. 

CUDA 지원 GPU(NVIDIA Geforce 시리즈 외)에는 수천 개의 컴퓨팅 스레드를 집합적으로 실행할 수 있는 수백 개의 코어가 존재하며, 이 코어들에는 공유 리소스(레지스터 파일 및 공유 메모리 등)가 있다. 온칩 공유 메모리를 사용하면 시스템 메모리 버스를 거치지 않고 이들 코어에서 실행되는 병렬 작업이 데이터를 공유할 수 있다.

그럼 **'Ubuntu 18.04 + ROS Melodic + Nvidia GPU'** 환경에서 `darknet-ros` 패키지를 CUDA를 사용하도록 빌드하는 방법을 알아보자. 



### 1. CUDA 설치

사전준비 / Nvidia GPU 드라이버 설치 / CUDA Toolkit 설치 / cuDNN 설치 

#### 1-1. 사전 준비

- CUDA 지원 GPU 존재 확인

  ```bash
  lspci | grep -i nvidia
  ```

  ```
  02:00.0 3D controler: NVIDIA Corporation Device 1f96 (rev a1)
  ```

  "NVIDIA" 문자열이 들어간 장치가 검색되는 지 확인

- 리눅스 버전 확인

  ```bash
  uname -m && cat /etc/*release
  ```

  ```
  x86_64
  DISTRIB_ID=Ubuntu
  DISTRIB_RELEASE=18.04
  DISTRIB_CODENAME=bionic
  DISTRIB_DESCRIPTION="Ubuntu 18.04.5 LTS"
  NAME="Ubuntu"
  VERSION="18.04.5 LTS (Bionic Beaver)"
  ID=ubuntu
  ID_LIKE=debian
  PRETTY_NAME="Ubuntu 18.04.5 LTS"
  VERSION_ID="18.04"
  HOME_URL="https://www.ubuntu.com/"
  SUPPORT_URL="https://help.ubuntu.com/"
  BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
  PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
  VERSION_CODENAME=bionic
  UBUNTU_CODENAME=bionic
  ```

  우분투의 경우 x86_64 플랫폼의 경우 18.04.5, 20.04.2 가 지원된다.

- gcc 컴파일러 설치 여부 및 버전 확인

  ```bash
  gcc --version
  ```

  ```
  gcc (Ubuntu 7.5.0-3ubuntu1~18.04) 7.5.0
  Copyright (C) 2017 Free Software Foundation, Inc.
  This is free software; see the source for copying conditions.  There is NO
  warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ```

  gcc 버전은 6.0 부터 지원된다. 

- kernel header 및 개발 패키지 설치여부 확인

  우분투 18.04 의 경우 다음 명령으로 kernel header 및 개발 패키지를 설치한다.

  ```
  sudo apt-get install linux-headers-$(uname -r)
  ```



















### 2. darknet-ros 빌드



#### 2-1 소스코드 복사

`-recursive` 옵션을 사용하여 `git clone` 명령을 수행하려면 `ssh-key` 가 자신의 github 계정에 등록되어 있어야 한다. 그 방법은 [**github 계정에 `ssh-key` 등록**](./darknet_ros_connect2github_ssh.md) 문서를 참조한다. 

```bash
cd ~/catkin_ws/src
```

```bash
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

#### 2-2 `CMakeList.txt` 수정

다음 작업 없이 빌드하면, CPU 를 사용하도록 빌드된다. 

```
gedit ./src/darknet-ros/darkenet-ros/CMakeList.txt &
```

`# Find CUDA` 섹션을  [**CUDA** 위키피디어 페이지](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs)  **'GPUs supported'** 챕터의 표( 아래는 해당 표의 일부를 발췌한 것임 )를 참조하여 편집해주어야 한다. 

사용하는 Graphic Card 가 GTX 1650 또는 MX450 인 경우 Compute Capability 가 7.5 에 해당한다는 것을 알 수 있다.

<table>
  <tr>
    <td><b>Compute<br>Capability</b></td>
    <td align="center">
        <b>Nvidia Graphic Card</b>
    </td>
  </tr>
  <tr>
    <td align="center"><b> 6.1 </b></td>
    <td>
      Nvidia TITAN Xp, Titan X,
GeForce GTX 1080 Ti, GTX 1080, GTX 1070 Ti, GTX 1070, GTX 1060, GTX 1050 Ti, GTX 1050, GT 1030, GT 1010,MX350, MX330, MX250, MX230, MX150, MX130, MX110
    </td>
  </tr>
  <tr>
    <td align="center"><b> 7.5 </b></td>
    <td>
      NVIDIA TITAN RTX, GeForce RTX 2080 Ti, RTX 2080 Super, RTX 2080, RTX 2070 Super, RTX 2070, TX 2060 Super, RTX 2060, GeForce GTX 1660 Ti, GTX 1660 Super, GTX 1660, GTX 1650 Super, <b>GTX 1650</b>, <b>MX450</b>
    </td>
  </tr>
  <tr>
    <td align="center"><b> 8.6 </b></td>
    <td>
      GeForce RTX 3090, RTX 3080 Ti, RTX 3080, RTX 3070 Ti, RTX 3070, RTX 3060 Ti, RTX 3060
    </td>
  </tr>
</table>

`# Find CUDA` 섹션에 다음과 같이 `-gencode arch=compute_75,code=sm_75` 를 추가해 준다.


```makefile
# Find CUDA
find_package(CUDA QUIET)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    # -gencode arch=compute_30,code=sm_30               # <---+
    # -gencode arch=compute_35,code=sm_35               #     |
    # -gencode arch=compute_50,code=[sm_50,compute_50]  #     +-- comment
    # -gencode arch=compute_52,code=[sm_52,compute_52]  #     |
    # -gencode arch=compute_61,code=sm_61               #     |
    # -gencode arch=compute_62,code=sm_62               # <---+
    -gencode arch=compute_75,code=sm_75 # <---- insert for MX450 or GTX1650
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()
```













---

 [튜토리얼 목록](../README.md) 
