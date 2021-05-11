## YOLO with ROS



------

## darknet_ros

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**자료원본(출처) :** <https://github.com/leggedrobotics/darknet_ros>

**튜토리얼 목록 :** [README.md](../README.md)

------

영상속의 객체( Object )들을 실시간으로 검출( Detection )하는 라이브러리로 요 몇 년 사이 가장 유명한 YOLO( You Only Look Once )의 ROS 패키지인 `darknet_ros` 를 설치하고, 간단한 example 노드를 만들어 구동해본다.



### 1. 설치



#### 1.1 의존성(dependencies)

- [**OpenCV**](https://opencv.org/) ( computer vision library )

- [**Boost**](https://opencv.org/) ( C++ library )

이 의존성들은 별도로 설치해 둘 필요는 없다. 빌드과정에서 설치되어있지 않을 경우 자동으로 다운로드된다.



#### 1.2 빌드(Build)

`darknet_ros` GitHub repository 에서 소스코드를 `git clone` 하려면 SSH 키를 자신의 GitHub 계정에 등록해 두어야만 한다. SSH 키를 만들고, 자신의 GitHub 에 등록하는 방법은 [**SSH 를 이용한 GitHub 연결**](./darknet_ros_connect2github_ssh.md) 을 참고한다. 



`gitclone` 명령으로 `darknet_ros` `github` 로부터  소스코드 복사한다. 

```bash
$ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

`catkin_make` 명령으로 빌드하기위해 `~/catkin_ws` 폴더로 경로를 변경한다. 

```bash
$ cd ~/catkin_ws
```

다음 명령으로 빌드한다. 

```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

CUDA( Nvidia에서 만든 병렬 컴퓨팅 플랫폼 및 API 모델 )가 지원되는 Nvidia GPU 를 이용할 경우 CPU만을 이용할 경우보다 약 500배 빠르다. 이를 이용하기위해서는 Nvidia GPU가 있어야하며 CUDA를 설치해야한다. CMakeLists.txt 파일은 CUDA 설치 여부를 자동으로 감지하도록 작성되었으며, 시스템에 CUDA가없는 경우 빌드 프로세스가 YOLO의 CPU 버전으로 전환됩니다.



#### 1.3 `weights` 파일 다운로드

`weights` 파일을 위한 폴더로 경로 변경

```bash
$ cd catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
```

다음 명령을 실행하여 COCO 데이터 세트에서 사전 학습 된 `weights` 파일 2개를 다운로드한다. 

```bash
$ wget http://pjreddie.com/media/files/yolov2.weights
$ wget http://pjreddie.com/media/files/yolov2-tiny.weights
```

다음 명령을 실행하여 VOC 데이터 세트에서 사전 학습 된 `weights` 파일 2개를 다운로드한다. 

```bash
$ wget http://pjreddie.com/media/files/yolov2.weights
$ wget http://pjreddie.com/media/files/yolov2-tiny.weights
```

YOLO v3 사전 학습 된 `weights` 파일 2개는 다음 명령으로 다운로드할 수 있다. 

```bash
$ wget http://pjreddie.com/media/files/yolov2.weights
$ wget http://pjreddie.com/media/files/yolov2-tiny.weights
```

[이 외의 미리 학습된 weights 파일 찾아보기](https://pjreddie.com/darknet/yolo/)



#### 1.4 







---

 [튜토리얼 목록](../README.md) 
