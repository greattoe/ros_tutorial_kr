## rospy_tutorial/ Tutorials/ WritingServiceClient



---

## 간단한 서비스와 클라이언트 작성

**출처 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber>http://wiki.ros.org/rospy_tutorials/Tutorials/WritingServiceClient

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---



### 1. 서비스 노드 작성

두 정수의 합을 구해 반환해 주는 서비스  "add_two_ints_server" 노드를 만들어보자.

이 전 튜토리얼에서 사용했던 ~/catkin_ws/src/rospy_tutorial 폴더로 경로를 변경한다.

```
user@computer:~$ roscd rospy_tutorial
```

서비스 파일( .srv )을 작성해 넣을 srv 폴더를 만든다.

```
user@computer:~/catkin_ws/src/rospy_tutorial$ mkdir srv
```

srv 폴더로 이동하여 서비스파일 AddTwoInts.srv 를 아래와 같이 작성 후, 저장한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial$ cd srv
user@computer:~/catkin_ws/src/rospy_tutorial/srv$ gedit AddTwoInts.srv &
```

```
int64 a
int64 b
---
int64 sum
```



경로를  ~/catkin_ws/src/rospy_tutorial/scripts 폴더로 변경하고, add_two_ints_server.py 를 작성한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/srv$ cd ../scripts
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ gedit add_two_ints_server.py &
```

```python
#!/usr/bin/env python

# rospy_tutorial 폴더의 하위폴더 srv 에서 AddTwoInts.srv 서비스와 그 응답서비스 import
from rospy_tutorial.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def svc_cb(req):	# 클라이언트 노드의 서비스 요청 발생시 호출될 콜백함수 svc_cb() 정의
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():   # 
    rospy.init_node('add_two_ints_server')	# 노드 초기화 (노드명:'add_two_ints_server')
    # 서비스명:'add_two_ints', 서비스타입:AddTwoInts.srv, 콜백함수: svc_cb()인 서비스 s 선언
    s = rospy.Service('add_two_ints', AddTwoInts, svc_cb)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

작성된 add_two_ints_server.py 파일에 실행 속성 부여하는 것을 잊지 말자.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x add_two_ints_server.py
```



### 2. 클라이언트 노드 작성

서비스노드 작성과 마찬가지로 ~/catkin_ws/src/rospy_tutorial/scripts 폴더에  add_two_ints_client.py 를 작성한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/srv$ cd ../scripts
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ gedit add_two_ints_client.py &
```

```python
#!/usr/bin/env python

import sys
import rospy
# rospy_tutorial 폴더의 하위폴더 srv의 모든 서비스 import
from rospy_tutorial.srv import *

def add_two_ints_client(x, y):	# 
    # 서비스를 호출하는 클라이언트 노드 코드는 간단한 편이다. init_node()를 호출할 필요도 없다.
    # 제일 먼저 할 일은 wait_for_service()를 호출하는 것인데, 이 함수는 편리하게도 'add_two_ints'
    # 서비스가 사용가능할 때까지 block 시킨다.
    rospy.wait_for_service('add_two_ints')
    try:
        # 서비스호출을 위한 handle 생성
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # 생성한 handle을 이용한 서비스 호출
        resp1 = add_two_ints(x, y)
        return resp1.sum
    # 서비스 호출이 실패할 경우의 예외처리
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:		# 서비스에 필요한 인수의 개 수가 맞게 입력된 경우
        x = int(sys.argv[1])	# x에 첫번째 인수를,
        y = int(sys.argv[2])	# y에 두번째 인수를 치환.
    else:						# 인수의 개 수가 틀린 경우
        print usage()			# usage() 함수를 호출하고,
        sys.exit(1)				# 프로그램 종료.
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```

작성된 add_two_ints_client.py 파일에 실행 속성 부여하는 것을 잊지 말자.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x add_two_ints_client.py
```

