## rospy_tutorial/ Tutorials/ WritingPublisherSubscriber

[TOC]

---

## 1. 간단한 퍼블리셔와 서브스크라이버 작성 

**출처 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber>

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---

std_msgs 와 rospy 에 의존성을 가진 rospy_tutorial 패키지 생성

```
user@computer:~$ cd ~/catkin_ws/src
user@computer:~/catkin_ws/src$ catkin_create_pkg rospy_tutorial std_msgs rospy
```

생성된 rospy_tutorial 패키지 폴더로 경로 변경

```
user@computer:~/catkin_ws/src$ cd rospy_tutorial
user@computer:~/catkin_ws/src/rospy_tutorial$
```



### 1. 퍼블리셔 노드 작성

노드( Node )는 ROS 네트워크 환경 안에서 동작하는 실행파일 이다. 이제 지속적으로 토픽을 ROS 네트워크로 발행하는 퍼블리셔 노드를 만들어보자.

#### 1.1 코드

먼저 파이썬 스크립트 코드를 위한 scripts 폴더를 만들고 경로를 만들어진 scripts 폴더로 변경한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial$ mkdir scripts
user@computer:~/catkin_ws/src/rospy_tutorial$ cd scripts
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ _
```

다음 코드를 작성하여 ~/catkin_ws/src/rospy_tutorial/scripts/talker.py 로 저장한다.   [talker.py](https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py)

```python
#!/usr/bin/env python
# 위 내용은 셔뱅(Shebang)으로, 이 스크립트 해석기의 위치를 지정한다. 모든 파이썬으로 작성된 노드는 반드시 이
# 셔뱅으로 시작해야 한다.

import rospy					# roscpp 코드의 "#include <ros.h>"에 해당하는 구문
from std_msgs.msg import String	# ROS 표준 메세지의 String 모듈 import

def talker():					# talker 함수 정의 시작
    # String 형식 토픽 'chatter'를 발행하는 퍼블리셔 'pub' 선언
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 'talker' 노드 초기화 
    rospy.init_node('talker', anonymous=True)
    # 1초당 10회의 빈도로 토픽을 발행하기 위한 rate 객체 선언 
    rate = rospy.Rate(10) # 10hz
    
    # rospy가 종료되지 않았으면 반복할 루프
    while not rospy.is_shutdown(): # roscpp 코드의 "while(ros::ok())"에 해당하는 구문
        # "hello world " 문자열 뒤에 현재 시간을 덧붙인 문자열을 변수 hello_str에 치환 
        hello_str = "hello world %s" % rospy.get_time()
        # time stamp가 표시되는 화면출력으로 hello_str 출력
        rospy.loginfo(hello_str)
        # 퍼블리셔 pub로 hello_str 발행
        pub.publish(hello_str)
        # 루프 시작 부터 1/10초가 지날 때까지 시간지연(토픽 발행 빈도 10회/초)
        rate.sleep()

if __name__ == '__main__':	# main() 함수를 명시적으로 표시하는 파이썬 문법
    try:                    # 뒤의 예외처리가 고려된 처리 시작
        talker()			# talker() 함수 호출
    except rospy.ROSInterruptException: # ROS 인터럽트 예외 발생시
        pass                            # Do Nothing
```







### 2. 서브스크라이버 노드 작성

 다음 코드를 작성하여 ~/catkin_ws/src/rospy_tutorial/scripts/listener.py 로 저장한다.   [listener.py](https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py)

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

