# How to use python scripts with catkin 



---

## Catkin 환경 Python 사용법

**참조1 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile>

**참조2 :**  <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber>http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

**튜토리얼 레벨 :**  초급

**선수 학습 :**  ROS 튜토리얼

**빌드 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---



### 1. 패키지 생성

작업경로를 ```~/catkin_ws/src``` 로 변경 후,  ```catkin_create_pkg``` 명령으로  ```std_msgs``` 와 ```rospy``` 에 의존성을 가진 ```rospy_tutorial``` 패키지 생성.

```
user@computer:~$ cd ~/catkin_ws/src
user@computer:~/catkin_ws/src$ catkin_create_pkg rospy_tutorial std_msgs rospy
Created file rospy_tutorial/package.xml
Created file rospy_tutorial/CMakeLists.txt
Created folder rospy_tutorial/src
Successfully created files in /home/ground0/catkin_ws/src/rospy_tutorial. Please adjust the values in package.xml.
user@computer:~/catkin_ws/src$ _
```

생성된 ```rospy_tutorial``` 패키지 폴더로 경로 변경 후, 폴더 내용 확인.

```
user@computer:~/catkin_ws/src$ cd rospy_tutorial
user@computer:~/catkin_ws/src/rospy_tutorial$ ls
CMakeLists.txt  package.xml  src
user@computer:~/catkin_ws/src/rospy_tutorial$ _
```



### 2. setup.py 작성

다음 내용과 같이 ```~/catkin_ws/src/rospy_tutorial/setup.py``` 파일을 작성.

```python
## ! 절대로 이 "setup.py" 파일을 '$ python setup.py'처럼 구동하지 마시오!!!( catkin을 사용하시오!! )

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rospy_tutorial'],	# packages=['패키지명'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```



### 3. package.xml 파일과 CMakeList.txt 편집

파이썬 스크립트를 이용하여 패키지를 작성할 경우  ```package.xml``` 파일과 ```CMakeList.txt``` 파일에서 확인할 내용은 다음과 같다.

#### 3.1 package.xml

```package.xml``` 파일은 다음 내용을 반드시 포함하고 있어야 한다. 

```
<buildtool_depend>catkin</buildtool_depend>
```

#### 3.2 CMakeList.txt

```CMakeList.txt``` 파일은 최소한 다음 내용을 반드시 포함하고 있어야 한다. 

```
cmake_minimum_required(VERSION 2.8.3)
project(rospy_tutorial)

find_package(catkin REQUIRED COMPONENTS std_msgs rospy)
catkin_package()
```

위에 언급된 내용 외의 추가적인 편집이 필요한 경우에 대해서는 ["ROS 파이썬 Makefile 작성"](rospyWriteROS_PythonMakefile.md) 에서 설명하고 있다.



### 4. 스크립트 작성

```scripts``` 폴더를 만들고 그 안에 필요한 스크립트를 작성 후 저장한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial$ mkdir scripts
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ gedit cmd4turtlesim.py &
```

```python
#!/usr/bin/env python

import rospy
import geometry_msgs.msg

def move_turtle():
    rospy.init_node("move_turtle")
    pub = rospy.Publisher("turtle1/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
    tw  = geometry_msgs.msg.Twist()
    tw.linear.x = tw.angular.z = 0.25
    pub.publish(tw)
   
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            move_turtle()
    except rospy.ROSInterruptException:
        print "Program terminated by"
```


작성한 스크립트 확인.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ ls *.py -al
-rw-rw-r-- 1 user user  453  9월 21 10:55 cmd4turtlesim.py
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ _
```

작성한 스크립트에 실행 속성을 부여.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x cmd4turtlesim.py
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ _
```

결과 확인.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ ls *.py -al
-rwxrwxr-x 1 user user  453  9월 21 10:55 cmd4turtlesim.py
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ _
```



### 5. 빌드 및 실행

#### 5.1 빌드

작업 경로를 ```~/catkin_ws``` 로 변경한 후, ```catkin_make``` 를 실행하여 패키지를 빌드한다. 

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ cd ~/catkin_ws
user@computer:~/catkin_ws$ catkin_make
```

빌드가 성공하면 ```~/catkin_ws/devel/setup.bash``` 파일의 변경 사항을 ```source``` 명령으로 반영한다.

```
user@computer:~/catkin_ws/src/rospy_tutorial/scripts$ chmod +x cmd4turtlesim.py
```

#### 5.2 실행

```Ctrl``` + ```Alt``` + ```T``` 를 눌러 새로 터미널 창을 열고,  ```roscore``` 를 실행한다.

```
user@computer:~$ roscore
```

```Ctrl``` + ```Alt``` + ```T``` 를 눌러 새로 터미널 창을 열고, ```turtlesim_node``` 를 실행한다.

```
user@computer:~$ rosrun turtlesim turtlesim_node
```

```Ctrl``` + ```Alt``` + ```T``` 를 눌러 새로 터미널 창을 열고, 지금 작성한 ```cmd4turtlesim.py``` 스크립트 실행한다.

```
user@computer:~$ rosrun rospy_tutorial cmd4turtlesim.py
```

#### ![](../img/cmd4turtlesim.png)