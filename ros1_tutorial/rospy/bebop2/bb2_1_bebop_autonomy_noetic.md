## bebop_autonomy 설치 (우분투 20.04 LTS + ROS Melodic) 



참고문서 https://developer.parrot.com/docs/SDK3



우분투 20.04 + ROS Melodic 환경에서 `bebop_autonomy` 를 빌드하려면 `parrot-arsdk` 에대한 의존성 문제가 발생하므로 우선 [`parrot_arsdk`](https://github.com/AutonomyLab/parrot_arsdk) 를 빌드해야한다. 하지만 이 역시 ARDroneSDK3 에 의존성을 가지므로 작업순서는

1. ARDroneSDK3 설치
2. parrot_arsdk 설치
3. bebop_autonomy 설치 

와 같은 순서로 이루어진다.

### 1. ARDroneSDK3 설치

ARDroneSDK3 에 대한 자료는 https://developer.parrot.com/docs/SDK3 를 참고한다.

#### 1.1 미리 설치해 둘 패키지

```
sudo apt install git build-essential autoconf libtool libavahi-client-dev libavcodec-dev libavformat-dev libswscale-dev libncurses5-dev mplayer
```

#### 1.2 `repo` 설치 및 설정

```
mkdir -p ~/.bin
PATH="${HOME}/.bin:${PATH}"
curl https://storage.googleapis.com/git-repo-downloads/repo > ~/.bin/repo
chmod a+rx ~/.bin/repo
```

우분투 20.04 에는 파이썬3 가 기본으로 설정되어 있으므로 파이썬으로 된 소스코드의 `#!/usr/bin/env python` 부분( 셔뱅: shebang )에서 에러가 나지않도록 `python-is-python3` 을 설치해준다. 

```
sudo apt install python-is-python3
```

`repo` 명령 사용을 위해 다음 명령을 통해 자신의 `github` 에서 사용하는 이메일주소와 이름을 등록한다. 

```
git config --global user.email "your@email.address"
git config --global user.name "yourname"
```

#### 1.3 소스코드 다운로드

ARDroneSDK3 소스코드를 복사할 폴더 생성 및 해당 폴더로 작업경로 변경

```
mkdir sdk3
cd sdk3
```

`arsdk_manifests.git` 파일의 `url` 에 대해 `repo init` 명령을 실행한다. 

```
repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git
```

이제 `repo sync` 명령을 실행하면 필요한 여러 다른 `repository` 들을 모두 다운로드한다. 

```
repo sync
```

```
Fetching: 100% (31/31), done in 18.702s
Garbage collecting: 100% (31/31), done in 0.034s
repo sync has successfully
```

#### 1.4 빌드

다음 명령을 실행하여 `ARDroneSDK3` 를 빌드한다.

```
./build.sh -p arsdk-native -t build-sdk -j
```



### 2. parrot-arsdk 설치

`~/catkin_ws/src` 로 경로를 변경한다.

```
cd ~/catkin_ws/src
```

`git clone` 명령으로 `parrot-arsdk` 패키지의 소스코드를 복사한다. ( 원래 https://github.com/AutonomyLab/parrot_arsdk 가 오리지널이지만 우분투 20.04 이 후 파이썬 2.x 지원 중단에 따른 python3 문법으로 변경처리가 되어있지 않으므로 그 처리가 되어 있는 https://github.com/larics/parrot_arsdk.git 의 코드로 빌드한다. )

```
git clone https://github.com/larics/parrot_arsdk.git
```

작업 경로를 `~/catkin_ws` 로 변경한다.

```
cd ~/catkin_ws
```

`catkin_make` 로 빌드한다.

```
catkin_make
```

`source`  명령으로 `~/catkin_ws/devel/setup.bash`  파일의 변경 사항을 반영한다. 

```
source ~/catkin_ws/devel/setup.bash
```

`rospack profile` 실행

```
rospack profile
```



### 3. bebop_autonomy 설치

catkin 워크스페이스의 src 폴더로 작업경로 변경

```bash
$ cd ~/catkin_ws/src
```

`bebop_autonomy` Github 로부터 소스코드 복제

```bash
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

gedit 으로 `bebop_driver`  패키지의`bebop_video_decoder.cpp` 파일을 열어 93, 95, 97 행의 코드를 수정한다.

```
gedit ~/catkin_ws/src/bebop_autonomy/bebop_driver/src/bebop_video_decoder.cpp
```
수정 전
```c
if (codec_ptr_->capabilities & CODEC_CAP_TRUNCATED)
    {
      codec_ctx_ptr_->flags |= CODEC_FLAG_TRUNCATED;
    }
    codec_ctx_ptr_->flags2 |= CODEC_FLAG2_CHUNKS;
```
수정 후

```c
if (codec_ptr_->capabilities & AV_CODEC_CAP_TRUNCATED)
    {
      codec_ctx_ptr_->flags |= AV_CODEC_FLAG_TRUNCATED;
    }
    codec_ctx_ptr_->flags2 |= AV_CODEC_FLAG2_CHUNKS;
```

catkin 워크스페이스로 작업경로 변경

```bash
$ cd ~/catkin_ws
```

빌드

```bash
$ catkin_make
```





