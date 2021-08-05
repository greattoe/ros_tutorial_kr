## ROS1 Tutorial (Korean)

#### ROS 설치


>1. ROS Kinetic
>     1. [Ubuntu 16.04 설치 및 설정](./ros1_tutorial/ubuntu/install_ubuntu_1604_lts.md) 
>     2. [ROS Kinetic 설치 및 설정](./ros1_tutorial/install_n_config/install_ROS_Kinetic.md) 
>2. ROS Melodic
>     1. [Ubuntu 18.04 설치 및 설정](./ros1_tutorial/ubuntu/install_ubuntu_1804_lts.md) 
>     2. [ROS Melodic 설치 및 설정](./ros1_tutorial/install_n_config/install_ROS_Melodic.md) 

---


#### 터틀봇3  운영환경 설정 팁


>  1. [PC 핫스팟 설정](./ros1_tutorial/turtlebot3/tb3_1_set_hotspot_on_1804.md) 
>  2. [SBC 시리얼 콘솔](./ros1_tutorial/turtlebot3/tb3_2_RPi_serial_console.md)  
>  3. [udev rules](./ros1_tutorial/turtlebot3/tb3_3_RPi_udev_rules.md)

---


####  rospy 튜토리얼

##### 1. Beginner level

>1. [catkin + rospy 사용법(1/2)](./ros1_tutorial/rospy/rospy_1_How2UsePythonWithCatkin_1.md) 
>2. [Topic Publisher / Subscriber 작성](./ros1_tutorial/rospy/rospy_2_WritingSimplePubSub.md) 
>3. [Service Server / Client 작성(1/2)](./ros1_tutorial/rospy/rospy_3_WritingServiceServerClient1.md) 
>4. [catkin + rospy 사용법(2/2) (setup.py)](./ros1_tutorial/rospy/rospy_4_How2UsePythonWithCatkin_2.md) 
>5. [Service Server / Client 작성(2/2)](./ros1_tutorial/rospy/rospy_5_WritingServiceServerClient2.md) 
>6. [간단한 Parameter 사용 예제](./ros1_tutorial/rospy/rospy_6_How2UseParameter.md) 
>7. 파이썬 경로 설정 
>8. numpy 사용법

##### 2. Intermediate level
>1. 메세지 발행
>2. Compressed Image Publisher/Subscriber

##### 3. 로봇 제어
>**3.1 turtlesim**
>
>>1. [직선 이동](./ros1_tutorial/rospy/mv_tutle_1_MoveInStraightLine.md) 
>>2. [좌, 우 회전](./ros1_tutorial/rospy/mv_tutle_2_RotateLeftRight.md) 
>>3. [목표장소로 이동](./ros1_tutorial/rospy/mv_tutle_3_Go2Goal.md) 
>
>**3.2 Turtlebot 3**
>
>>1. [직선 이동](./ros1_tutorial/rospy/turtlebot3/tb3_1_Move_in_Straight_Line.md) 
>>2. [좌, 우 회전](./ros1_tutorial/rospy/turtlebot3/tb3_2_Rotate_Left_n_Right.md) 
>>3. [목표지점으로 이동 1](./ros1_tutorial/rospy/turtlebot3/tb3_3_Go2Goal.md) 
>>4. [목표지점으로 이동 2](./ros1_tutorial/rospy/turtlebot3/tb3_4_GoToGoal.md) 
>>5. [Odometry 토픽을 이용한 2D Pose 토픽 발행](./ros1_tutorial/rospy/turtlebot3/tb3_5_Sub_Odom_Pub_Pose2D.md)
>>6. [2D Pose 토픽에 의한 직선 이동](./ros1_tutorial/rospy/turtlebot3/tb3_6_Straight_by_Pose2D.md)
>>7. [2D Pose 토픽에 의한 회전](./ros1_tutorial/rospy/turtlebot3/tb3_7_Rotate_by_Pose2D.md)
>>8. [2D Pose 토픽에 의한 회전 및 직선이동 라이브러리 작성](./ros1_tutorial/rospy/turtlebot3/tb3_8_Rotate_n_Straight_Library.md)
>>9. [목표지점으로 이동 3](./ros1_tutorial/rospy/turtlebot3/tb3_9_move2xy.md)
>>10. [AR 마커 탐색](./ros1_tutorial/rospy/turtlebot3/색tb3_10_search_ar_marker.md)
>>11. 마커가 화면 중앙에 오도록 터틀봇3 제어 
>>12. 마커가 화면 중앙에 위치한 경우의 '/tb3pose' 토픽 값 저장 및 경로 계산
>>13. 계산된 경로에 따라 주행하여 마커와 마주보기
>>14. 마커앞 10 ~ 15(cm) 영역에 멈추기
>
>**3.3 Parrot Bebop2**
>
>>1. [bebop_autonomy](./ros1_tutorial/rospy/bebop2/bb2_1_bebop_autonomy.md) 
>>2. [Parrot-Sphinx](./ros1_tutorial/rospy/bebop2/bb2_2_parrot_sphinx.md) 
>>3. [teleop_key 노드 작성](./ros1_tutorial/rospy/bebop2/bb2_3_teleop_key.md)
>>4. [Odometry 토픽을 참조한 이동](./ros1_tutorial/rospy/bebop2/bb2_4_move_by_odom.md)
>>5. [실시간 드론 위치표시 웹페이지 작성](./ros1_tutorial/rospy/bebop2/bb2_5_mark_bebop2_on_web.md) 
>>6. [darknet_ros_without_cuda(1/2)](./ros1_tutorial/darknetROS/darknet_ros_1_install_n_example.md)
>>7. [darknet_ros_with_cuda(2/2)](./ros1_tutorial/darknetROS/darknet_ros_2_using_cuda.md)
>>8. [GPS 좌표를 이용한 드론 이동](./ros1_tutorial/rospy/bebop2/bb2_6_move_by_gps.md) 
>>9. [우분투 20.04 bebop_autonomy 설치](./ros1_tutorial/rospy/bebop2/bb2_1_bebop_autonomy_noetic.md) 
>
>**[3.4 Multi Robot 제어](./ros1_tutorial/multimaster_fkie/multimaster_fkie.md)**

##### 4. tf 튜토리얼
>0. [tf 는...](./ros1_tutorial/rospy/tf_0_Instroduction.md)
>1. [tf 브로드캐스터](./ros1_tutorial/rospy/tf_1_broadcaster.md)
>2. [tf 리스너](./ros1_tutorial/rospy/tf_2_listener.md)
>3. [tf 프레임 추가](./ros1_tutorial/rospy/tf_3_adding_frame.md)
>4. [tf 와 Time](./ros1_tutorial/rospy/tf_4_tf_n_time.md)
>5. [tf 와의 시간여행](./ros1_tutorial/rospy/tf_3_adding_frame.md)

##### 5. AR Marker 튜토리얼
>1. [카메라 Calibration](./ros1_tutorial/camera_calibration/how_to_calibrate_monocular_camera.md)
>2. [ar_track_alvar 구동](./ros1_tutorial/rospy/ar_1_ar_track_alvar.md)
>3. [ AR 마커 정보 해석](./ros1_tutorial/rospy/ar_2_analysis_marker.md)

##### 6. rqt 튜토리얼
>1. [rqt 플러그인 패키지 생성](./ros1_tutorial/rospy/rqt_1_create_rqt_plugin_pkg.md)
>2. [파이썬 rqt 플러그인 작성](./ros1_tutorial/rospy/rqt_2_writing_python_plugin.md)

##### 7. OpenCV 관련 튜토리얼
>1. [cv_bridge](./ros1_tutorial/rospy/open_cv/opencv_1_cv_bridge.md)
>2. [동영상으로 토픽발행](./ros1_tutorial/rospy/open_cv/opencv_2_image_publisher.md)

---

#### roscpp 튜토리얼

##### 1 Beginner level
>1. Publisher/Subscriber 작성
>2. Service/Client 작성
>3. Parameter 사용
>4. NodeHandle 을 통한 Private Names 접근
>5. 콜백 함수로서의 클라스 멤버함수 사용

##### 2 Intermediate level
>###### 1. Timers 의 이해
>
>###### 2. Dynamic Reconfigure
>
>###### 3. turtlesim 튜토리얼
>
>>1. [turtlesim 원격조종 노드 작성](./ros1_tutorial/roscpp/ts1_teleop_turtlesim.md)
>>2. [turtlesim pose 구독 노드 작성](./ros1_tutorial/roscpp/ts2_sub_turtlesim_pose.md)
>>3. [turtlesim 직선 이동](./ros1_tutorial/roscpp/ts3_straight_move_turtlesim.md)
>
>###### 4. tf 튜토리얼
>
>>1. [tf 는... ](./ros1_tutorial/roscpp/tf_1_Instroduction.md)
>>2. [tf 브로드캐스터](./ros1_tutorial/roscpp/tf_2_broadcaster.md)
>>3. [tf 리스너](./ros1_tutorial/roscpp/tf_3_listener.md)

---

#### 그 외의 ROS 튜토리얼
##### SLAM & Navigation 튜토리얼

>1. [터틀봇3 Gazebo 를 이용한 SLAM & Navigation](./ros1_tutorial/slam_n_nav/)
>
>2. [터틀봇3 SLAM & Navigation](./ros1_tutorial/slam_n_nav/)

##### URDF 튜토리얼

>1. [시각적 로봇 모델 URDF 작성](./ros1_tutorial/urdf/urdf_1_building_visual_robot_model.md)
>
>2. [이동할 수 있는 로봇 모델 URDF 작성](./ros1_tutorial/urdf/urdf_2_building_movable_robot_model.md)
>
>3. [URDF에 물리적 특성과 충돌 속성 추가](./ros1_tutorial/urdf/urdf_3_adding_physical_n_collision.md)
>
>4. Xacro를 이용한 URDF 간략화
>
>5. Gazebo에서의 URDF 사용

   

#### 미분류

>###### [콜백( callback )과 스핀( spin )](./ros1_tutorial/roscpp/callback_n_spin.md)
>
>###### [스핀( spin )과 슬립( sleep )](./ros1_tutorial/roscpp/spin_n_sleep.md)
>
>###### [노드렛( nodelet )](./ros1_tutorial/roscpp/nodelet.md)