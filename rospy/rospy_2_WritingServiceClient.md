## rospy_tutorial/ Tutorials/ WritingServiceClient



---

## 간단한 서비스 서버와 클라이언트 작성

**출처 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingServiceClient>

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---

이 번 튜토리얼에서는 이 전 튜토리얼에서 언급했던 사용자 정의 서비스( .srv 파일 )가 사용된다. 따라서 약간의 추가 작업이 필요하다. 이 작업들은 사용자 정의형식 메세지( .msg 파일 ) 을 사용하는 경우에도 거의 비슷하다.



**AddTwoInts.srv** 서비스 파일 작성 

이 전 튜토리얼에서 사용했던 `~/catkin_ws/src/rospy_tutorial` 폴더로 경로를 변경한다.

```
$ roscd rospy_tutorial
```

서비스 파일( .srv )을 작성해 넣을 `srv` 폴더를 만든다. ( 메세지 파일의 경우 ` .msg` 파일을 넣을 `msg` 폴더를 만든다. )

```
$ mkdir srv
```

`srv` 폴더로 이동하여 서비스파일 `AddTwoInts.srv` 를 아래와 같이 작성 후, 저장한다.

```
$ cd srv
$ gedit AddTwoInts.srv &
```

```
int64 a
int64 b
---
int64 sum
```



**package.xml**

우선 `package.xml` 파일에 다음을 추가한다.

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```



**CMakeLists.txt**

다음으로 `CMakeLists.txt` 파일 차례다.

- `find_package( ... )` 항목에  `message_generation` 을 추가한다.

```shell
find_package(catkin REQUIRED COMPONENTS
   rospy
   geometry_msgs
   std_msgs
   message_generation	# 여기에 추가
)
```

* `add_service_files( ... )` 항목의 `FILES` 에 서비스파일 `AddTwoInts.srv` 을 추가한다.

  ( 메세지 사용 경우: `add_message_files( ... )` 항목의 `FILES` 에 메세지파일 `xxxxxx.msg` 을 추가한다. )

```shell
add_service_files(
   FILES
   AddTwoInts.srv
#   Service1.srv
#   Service2.srv
)
```

- `generate_messages( ... )` 항목의  `DEPENDENCIES` 에 `std_msgs` 를 추가한다.

```shell
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

- `catkin_package( ... )` 항목에 `message_runtime` 를 `CATKIN_DEPENDS` 에 추가한다.

```shell
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rospy_tutorial
#  CATKIN_DEPENDS rospy std_msgs
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
```



### 1. 서비스 노드 작성

두 정수의 합을 구해 반환해 주는 서비스  "add_two_ints_server" 노드를 만들어보자.

경로를  ~/catkin_ws/src/rospy_tutorial/scripts 폴더로 변경하고, add_two_ints_server.py 를 작성한다.

```
$ roscd rospy_tutorial/scripts
$ gedit add_two_ints_server.py &
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

작성된  `add_two_ints_server.py` 파일에 실행 속성 부여.

```
$ chmod +x add_two_ints_server.py
```



### 2. 클라이언트 노드 작성

서비스노드 작성과 마찬가지로 `~/catkin_ws/src/rospy_tutorial/scripts` 폴더에  `add_two_ints_client.py` 를 작성한다.

```
$ roscd rospy_tutorial/scripts
$ gedit add_two_ints_client.py &
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

작성된 `add_two_ints_client.py` 파일에 실행 속성 부여.

```
$ chmod +x add_two_ints_client.py
```



### 3. 빌드 및 실행

빌드를 위해 작업경로를 `~/catkin_ws` 로 변경한다.

```
$ cd ~/catkin_ws
```

`catkin_make` 명령으로 빌드한다.

```
$ catkin_make
```

빌드 결과가 반영되어 변경된 `~/catkin_ws/devel/setup.bash` 의 내용을 `source` 명령을 이용하여 작업중인 터미널 창에 반영한다. 

```
$ source ~/catkin_ws/devel/setup.bash
```



`roscore` 실행

```
$ roscore
```



`Ctrl+Alt+T` 를 입력하여, 새로운 터미널을 열어서 서비스 서버 노드를 실행한다. 

**1. 서비스 서버 실행, 서비스 요청 대기**

```
$ rosrun rospy_tutorial add2ints_server.py 
Ready to add two ints.
```



다시 `Ctrl+Alt+T` 를 입력하여, 새로운 터미널을 하나 더 열어서 서비스 클라이언트 노드를 실행한다.

**2.  서비스 클라이언트에서 1, 2의 합을 구하는 서비스 요청** 

```
$ rosrun rospy_tutorial add_two_ints_client.py 1 2
```



**3. 서비스 서버가 클라이언트 요청에 응답하여 서비스 요청에 대한 결과 반환 후 다시 서비스 요청 대기** 

```
user@computer:~$ rosrun rospy_tutorial add2ints_server.py 
Ready to add two ints.
Returning [1 + 2 = 3]
```



**4. 서비스 클라이언트 서비스 응답 확인 후 종료**

```
$ rosrun rospy_tutorial add_two_ints_client.py 1 2
Requesting 1+2
1 + 2 = 3
```





---

​                                    [튜토리얼 목록 열기](../README.md)                                                           [다음 튜토리얼](./rospy_5_WritingROS_pythonMakefile.md) 
