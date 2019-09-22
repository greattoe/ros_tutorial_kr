## rospy_tutorial/ Tutorials/ WritingPublisherSubscriber



---

## 1. 간단한 퍼블리셔와 서브스크라이버 작성 

**출처 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber>

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---

[이전 튜토리얼](./rospy_0_How2UsePythonWithCatkin.md)에서 만든 `rospy_tutorial` 에 퍼블리셔 노드 `example_pub` 과  그 서브스크라이브 노드 `example_sub` 을 추가하려 한다. 그러나 작은 문제가 있다. 이전 튜토리얼에서 `rospy_tutorial` 패키지 생성을 위해 실행한 명령은 `$ catkin_create_pkg rospy_tutorial geometry_msgs rospy` 였다. 즉  `rospy_tutorial` 패키지의 의존성은 `geometry_msgs` 와  `rospy`  2가지라는 것이다. 

이번 튜토리얼에서 작성할 퍼블리셔와 서브스크라이버 노드들은  `std_msgs` 와  `rospy` 에 대해 의존성을 가진다. 따라서   `std_msgs` 에 대한 의존성 설정을 추가하기 위해 `package.xml` 파일과  `CMakeLists.txt` 파일을 편집해야 한다.

**package.xml** 편집

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>std_msgs</build_depend><!-------------------- 이 줄 추가 -->
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend><!------ 이 줄 추가 -->
  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend><!---------------------- 이 줄 추가 -->
```

**CMakeLists.txt** 편집

```bash
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs	# ----------------------------------------------- 이 줄 추가
)
```



### 1. 퍼블리셔 노드 작성

노드( Node )는 ROS 네트워크 환경 안에서 동작하는 실행파일 이다. 이제 지속적으로 토픽을 ROS 네트워크로 발행하는 퍼블리셔 노드를 만들어보자.

먼저 작업 경로를 `scripts` 폴더로 변경한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial$ cd scripts
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ _
```

다음 코드를 작성하여 `~/catkin_ws/src/rospy_tutorial/scripts/example_pub.py` 로 저장한다. 

```python
#!/usr/bin/env python
# 위 내용은 셔뱅(Shebang)으로, 이 스크립트 해석기의 위치를 지정한다. 모든 파이썬으로 작성된 ROS 노드는 반드
# 시 이 셔뱅으로 시작해야 한다.

import rospy                    # roscpp 코드의 "#include <ros.h>"에 해당하는 구문
from std_msgs.msg import String # ROS 표준 메세지의 String 모듈 import

def simple_pub():               # simple_pub() 함수 정의 시작
    # 'sample_pub' 노드 초기화 
    rospy.init_node('sample_pub', anonymous=True)
    # String 형식 토픽 'hello'를 발행하는 퍼블리셔 'pub' 선언
    pub = rospy.Publisher('hello', String, queue_size=10)
    # 1초당 10회의 빈도로 토픽을 발행하기 위한 rate 객체 선언 
    rate = rospy.Rate(10) # 10hz
    
    # rospy가 종료되지 않았으면 반복할 루프
    while not rospy.is_shutdown(): # roscpp 코드의 "while(ros::ok())"에 해당하는 구문
        # "hello~ " 문자열 뒤에 현재 시간을 덧붙인 문자열을 String 변수 str에 치환 
        str = "hello~ %s" % rospy.get_time()
        # time stamp가 표시되는 화면출력으로 str 출력
        rospy.loginfo(str)
        # 퍼블리셔 'pub'으로 'str'의 내용을 토픽명 'hello'로 발행
        pub.publish(str)
        # 루프 시작 부터 1/10초가 지날 때까지 시간지연(토픽 발행 빈도 10회/초)
        rate.sleep()

if __name__ == '__main__':  # 모듈명이 저장되는 전역변수 __name__에 저장된 값이 '__main__'이면
    try:                    # 뒤에 나오는 예외처리(except ... :)를 고려한 실행 구간 시작
        simple_pub()        # simple_pub() 함수 호출
    except rospy.ROSInterruptException: # ROS 인터럽트 예외 발생시
        print "Program terminated"      # 프로그램 종료 메세지 화면출력
```

작성한 코드를 저장, 종료 후 실행 속성을 부여한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x example_pub.py
```



### 2. 서브스크라이버 노드 작성

 다음 코드를 작성하여 `~/catkin_ws/src/rospy_tutorial/scripts/example_sub.py` 로 저장한다. 

```python
#!/usr/bin/env python
# 위 내용은 셔뱅(Shebang)으로, 이 스크립트 해석기의 위치를 지정한다. 모든 파이썬으로 작성된 ROS 노드는 반드
# 시 이 셔뱅으로 시작해야 한다.

import rospy                    # roscpp 코드의 "#include <ros.h>"에 해당하는 구문
from std_msgs.msg import String	# ROS 표준 메세지형식 중 String 모듈 import. 
                                # roscpp 코드 "#include <std_msgs/String.h>"에 해당한다.
# 메세지 수신 이벤트 발생 시 호출될 콜백함수 callback() 정의
def cb_func(msg):
    # time stamp + subscribe 노드명 + ' msg: ' + subscribe data 를 화면 출력 
    rospy.loginfo(rospy.get_caller_id() + 'subscribed message: %s', msg.data)

def simple_sub():               # "simple_sub()"함수 정의 시작
    # ROS 환경에서 노드는 중복되지 않는 유일한 이름을 가져야만 한다. 이름이 같은 두 개의 노드가
    # 실행된다면, 먼저 실행된 노드는 두 번째 노드가 실행될 때 강제종료된다.
    # 'anonymous=True' 플래그는 이 노드의 이름을 rospy가 관리할 수 있도록 하여 이 노드와 
    # 같은 이름의 노드가 실행되어 있더라도 rospy가 적당한 중복없는 이름으로 노드명을 변경함으로써
    # 같은 토픽을 구독하는 다수의 구독노드를 실행할 수 있게 한다는 의미이다.
    # rospy.init_node('sample_sub', anonymous=True)
	rospy.init_node('sample_sub') # 수신 메세지 String 길이를 짧게 하기위해 수정
    
    # 토픽명:'hello', 토픽형식:String, 콜백함수명:cb_func인 서브스크라이버 설정
    rospy.Subscriber('hello', String, cb_func)

    # roscpp의 'ros::spin();'에 해당하는 코드. 프로그램 종료 시점까지 제어가 유지되도록 한다.
    rospy.spin()                # Ctrl-C 입력이 있을 때까지 새 메세지가 발행되면 콜백함수를 호출한다.

if __name__ == '__main__':      # 인터프리터 전역변수 __name__ 의 값이 '__main__' 이면
    simple_sub()                # 'simple_sub()' 함수 호출
```

작성한 코드를 저장, 종료 후 실행 속성을 부여한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x example_sub.py
```



### 3. 작성한 노드 실행

`catkin_make` 실행을 위해 작업 경로를 catkin workspace 로 사용하고 있는 `~/catkin_ws` 로 변경한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ cd ~/catkin_ws
```

`catkin_make` 실행.

```
user@computer:~/catkin_ws$ catkin_make
```

변경된  `~/catkin_ws/devel/setup.bash` 의 내용을 `source` 명령을 이용하여 반영시킨다.

```
user@computer:~/catkin_ws$ source ./devel/setup.bash
```



`roscore` 실행

```
user@computer:~/catkin_ws$ roscore
```



`Ctrl+Alt+T` 를 입력하여 새창을 열고 `example_pub` 노드를 실행한다.

```
user@computer:~$ rosrun rospy_tutorial example_pub.py
[INFO] [1569133145.461737]: hello~ 1569133145.46
[INFO] [1569133145.562266]: hello~ 1569133145.56
[INFO] [1569133145.662338]: hello~ 1569133145.66
[INFO] [1569133145.762315]: hello~ 1569133145.76
[INFO] [1569133145.862293]: hello~ 1569133145.86
[INFO] [1569133145.962392]: hello~ 1569133145.96
[INFO] [1569133146.062291]: hello~ 1569133146.06
[INFO] [1569133146.162301]: hello~ 1569133146.16
```



`Ctrl+Alt+T` 를 입력하여 새창을 열고 `example_sub` 노드를 실행한다.

```
user@computer:~$ rosrun rospy_tutorial example_sub.py
[INFO] [1569133145.563670]: /sample_sub msg: hello~ 1569133145.56
[INFO] [1569133145.663827]: /sample_sub msg: hello~ 1569133145.66
[INFO] [1569133145.764018]: /sample_sub msg: hello~ 1569133145.76
[INFO] [1569133145.864110]: /sample_sub msg: hello~ 1569133145.86
[INFO] [1569133145.964654]: /sample_sub msg: hello~ 1569133145.96
[INFO] [1569133146.064125]: /sample_sub msg: hello~ 1569133146.06
[INFO] [1569133146.164177]: /sample_sub msg: hello~ 1569133146.16
```



[튜토리얼 목록 열기](../README.md)

[다음 튜토리얼](./rospy_2_WritingServiceClient.md)







