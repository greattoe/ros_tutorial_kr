## AR Marker(Python)



------

## ar_marker_alvar

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**튜토리얼 목록 :** [README.md](../README.md)

------

증강 현실( Augmented Reality )에서 사용하는 AR Marker 는 마커 자체에 tf 와 pose 를 가지고 있다. ROS 에서 많이 사용되는 ar_track_alvar 패키지를 이용하여 AR 마커 인식과 그 pose 값을 이용한 트래킹 구현에 필요한 기본을 알아보자.



### 1. 'ar_track_alvar' 패키지

[ROS Wiki](http://wiki.ros.org/)에 설치 방법 및 [github 주소](https://github.com/ros-perception/ar_track_alvar/tree/kinetic-devel), 그 밖의 정보가 잘 정리되어 있다. 

**ROS Wiki 'ar_track_alvar' 페이지:** <http://wiki.ros.org/ar_track_alvar>



#### 1.1 설치

**Binary 설치**

설치할 패키지는 `ros-kinetic-ar-track-alvar` 와  `ros-kinetic-ar-track-alvar-msgs` 이다.

```bash
sudo apt-get install ros-kinetic-ar-track-alvar*
```

**소스 코드 빌드**

소스코드를 `catkin_make` 로 빌드하여 사용하길 원한다면 먼저 catkin 워크스페이스의 'src' 폴더로 경로 변경 후, 소스코드를 가져다 빌드한다. ( 빌드할 때에는 먼저 `~/catkin_ws` 폴더로 경로 변경한다. )

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/ar_track_alvar/tree/kinetic-devel
cd ~/catkin_ws
catkin_make
```



#### 1.2 실행을 위한 launch 파일 준비

`ar_track_alvar` 패키지 구동 `launch` 파일을 하나 만들어 보자. AR Marker를 인식하는 노드가 작동하려면 

1. 영상
2. 카메라 영상을 ROS 네트워크에 스트리밍하는 노드(`uvc_camera_node`, `usb_cam_node` 등)가 구동 중이어야 한다.
3. `ar_track_alvar` 패키지가 1번 항목의 노드가 발행하는 토픽(영상)을 입력으로 구동 중이어야 한다.
4. 사용자 작성 노드가 2번 항목의 노드가 발행하는 토픽(마커에 대한 토픽:`/ar_pose_maeker`)을 구독하여 처리한다.

작성할 `launch` 파일은 위의 2~4 항목 중 3번에 해당하는 기능을 구동하는 `launch` 파일이다. 당장은 `launch` 파일 하나만 필요하지만 나중을 위해 사용자 패키지를 하나 만들어 작성하자.

별다른 의존성이나 작업이 필요없지만, 나중에 이 패키지에 marker tracking 같은 기능의 소스코드를 추가할 때를 위해 `rospy` 에 대해 의존성을 가지는 `ar_marker` 패키지를 생성한다. 

패키지 생성은 `catkin` 워크스페이스의 하위 폴더인 `src` 폴더에서 한다.

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg ar_marker rospy
```

경로를 새로 만든 패키지 폴더(`ar_marker`)로 변경하고, `launch` 폴더를 만든다.

```bash
$ cd ar_marker
$ mkdir launch
```

 다시 경로를 지금 만든 `launch` 폴더로 변경한다.

```bash
$ cd launch
```

`ar_track_alvar` 가 설치된 곳에서 `launch` 파일 하나를 좀 전에 만든 `launch` 폴더로 복사한다. ( `pr2_indiv_no_kinect.launch` 파일을 `uvc_track_marker.launch` 로 이름을 바꿔 복사 )

```bash
$ cp /opt/ros/kinetic/share/ar_track_alvar/launch/pr2_indiv_no_kinect.launch ./uvc_track_marker.launch
```

복사한 `uvc_track_marker.launch` 를 편집한다. 

```bash
$ cd launch
$ gedit uvc_track_marker.launch &
```

아래는 복사해온 `launch` 파일의 내용과 변경할 값에 대한 설명이다.


```xml
<launch>
  <!-- 1. 자리에 사용하려는 마커의 한 변의 길이를 cm단위의 숫자로 "4.4" 위치에 바꿔 적는다.-->
  <arg name="marker_size" default="4.4" />
  <arg name="max_new_marker_error" default="0.08" />
  <arg name="max_track_error" default="0.2" />
  <!-- 2. AR 마커 인식에 사용할 '카메라 영상 스트리밍 토픽명'을 default 항목 값으로  -->
  <arg name="cam_image_topic" default="/wide_stereo/left/image_color" />
  <!-- 3. 2번의 영상을 취득하는 '카메라 정보 토픽명'을 default 항목 값으로  -->
  <arg name="cam_info_topic" default="/wide_stereo/left/camera_info" />
  <!-- 4. 영상 출력 프레임 이름을 default 항목 값으로  -->
  <arg name="output_frame" default="/torso_lift_link" />

  <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
    <param name="marker_size"           type="double" value="$(arg marker_size)" />
    <param name="max_new_marker_error"  type="double" value="$(arg max_new_marker_error)" />
    <param name="max_track_error"       type="double" value="$(arg max_track_error)" />
    <param name="output_frame"          type="string" value="$(arg output_frame)" />

    <remap from="camera_image"  to="$(arg cam_image_topic)" />
    <remap from="camera_info"   to="$(arg cam_info_topic)" />
  </node>
</launch>
```

1. **marker_size**

   프린터로 출력한 AR Marker의 한 변의 길이( 검은 색 정사각형의 한변의 길이 )를 실측하여 cm 단위의 숫자를 ""안에 입력한다.

2. **cam_image_topic** 및 

3. **cam_info_topic**

   마커를 인식할 카메라의 구동 노드로 uvc_camera 패키지의 uvc_camera_node를 구동한 경우, rostopic list 명령을 실행해보면 다음과 같은 화면 출력을 볼 수 있다.
   
   ```bash
   $ rostopic list
      /camera_info
      /image_raw
      /image_raw/compressed
      /image_raw/compressed/parameter_descriptions
      (이하 생략 / Omitted below)
   ```
   
   출력된 목록 중 `/camera_info` 가 **3. cam_info_topic** 에 해당하고, `/image_raw` 가 **2. cam_image_topic** 에 해당한다.

4. **output_frame**

   rviz에 표시될 카메라 기준위치를 나타낼 프레임 이름으로 아래와 같이 `$ rostopic echo /camera_info` 명령 실행 결과 중 `frame_id` 항목 값인 `"camera"` 가 **4. output_frame** 에 해당한다.
   
   ```bash
   $ rostopic echo /camera_info
   ---
   header: 
     seq: 525531
     stamp: 
       secs: 1569793369
       nsecs: 758004531
     frame_id: "camera"
   height: 480
   width: 640
   (이하 생략 / Omitted below)
   ```

이 같은 내용을 바탕으로 track_marker.launch 파일을 완성시켜 보면 다음과 같다.

```xml
<launch>
  <arg name="marker_size" default="5.2" />
  <arg name="max_new_marker_error" default="0.08" />
  <arg name="max_track_error" default="0.2" />
  <arg name="cam_image_topic" default="/image_raw" />
  <arg name="cam_info_topic" default="/camera_info" />
  <arg name="output_frame" default="camera" />

  <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
    <param name="marker_size"           type="double" value="$(arg marker_size)" />
    <param name="max_new_marker_error"  type="double" value="$(arg max_new_marker_error)" />
    <param name="max_track_error"       type="double" value="$(arg max_track_error)" />
    <param name="output_frame"          type="string" value="$(arg output_frame)" />

    <remap from="camera_image"  to="$(arg cam_image_topic)" />
    <remap from="camera_info"   to="$(arg cam_info_topic)" />
  </node>
</launch>
```

`~/catkin_ws`로 경로를 변경하고 `catkin_make`를 실행하여 `uvc_track_marker.launch` 패키지를 작성한 `ar_marker` 패키지를 빌드한다.



### 2. 구동 및 마커인식 확인

`roscore` 실행

```bash
$ roscore
```

USB 카메라 구동

```bash
$ rosrun uvc_camera uvc_camera_node
```

`uvc_track_marker.launch` 실행

```bash
$ roslaunch ar_marker uvc_track_marker.launch
```

토픽 리스트에 `/ar_pose_marker`가 존재하는 지 확인

```bash
$ rostopic list
/ar_pose_marker
/ar_track_alvar/enable_detection
/ar_track_alvar/parameter_descriptions
(이하 생략 / Omitted below)
```

카메라 시야에 AR 마커를 가져다 놓고 `$ rostopic echo /ar_pose_marker` 명령을 실행하여 아래 결과와 같이 마커가 제대로 인식되는 지 확인한다. 

```bash
$ rostopic /ar_pose_marker
---
header: 
  seq: 329
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1569796037
        nsecs: 237993750
      frame_id: "camera"
    id: 3
    confidence: 0
    pose: 
      header: 
        seq: 0
        stamp: 
          secs: 0
          nsecs:         0
        frame_id: ''
      pose: 
        position: 
          x: -0.0091884005952
          y: 0.0549410532295
          z: 0.415940946221
        orientation: 
          x: 0.998650018797
          y: -0.00480358428624
          z: 0.0392601699902
          w: 0.0336705299524
---
```



---

이전 튜토리얼 &nbsp; &nbsp; / &nbsp; &nbsp; [튜토리얼 목록](../README.md) &nbsp; &nbsp; / &nbsp; &nbsp; [다음 튜토리얼](./ar_2_analysis_marker.md)
