## How to use python scripts with catkin 



---

## Catkin + Python 사용법 2/2

**참조 :**

- <http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile>
- <http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber>http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

**튜토리얼 레벨 :**  Beginner(초급)

**이전 튜토리얼 :** [catkin 빌드환경에서 rospy 사용법(1/2)](./rospy_0_How2UsePythonWithCatkin_1.md) 

**다음 튜토리얼 :** [Simple Publisher & Subscriber](./rospy_1_WritingPubSub.md)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

---



이 번 튜토리얼에서는 [catkin 빌드환경에서 rospy 사용법(1/2)](./rospy/rospy_0_How2UsePythonWithCatkin_1.md) 튜토리얼의  ```rospy_tutorial``` 패키지에 키보드로 `turtlesim_node` 의 거북이를 제어하는 노드(`turtle_teleop_key` 와 같이 작동하는 `remote_ctrl_turtle.py`)를 추가해보려 한다. 

이를 구현하려면 키보드 입력을 받아야 한다. 파이썬의 `input()` 함수는 키를 입력한 후 `Enter` 키를 입력해야만 키입력이 전달된다. 이것은 로봇을 제어하기에는 적당하지 않으므로, 키보드를 누를 때마다 입력을 받을 수 있는 코드가 필요하다. 

다음은 리눅스에서 키보드 입력을 처리하기 위해 작성한 `GetChar.py` 코드이다. 

```python
#! /usr/bin/env python
 
import os, time, sys, termios, atexit, tty
from select import select
  
# class for checking keyboard input
class GetChar:
    def __init__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
  
        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
  
        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)
      
      
    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
  
    def getch(self):        # get 1 byte from stdin
        """ Returns a keyboard character after getch() has been called """
        return sys.stdin.read(1)
  
    def chk_stdin(self):    # check keyboard input
        """ Returns True if keyboard character was hit, False otherwise. """
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr

```

위 `GetChar.py` 파일을 사용자 라이브러리로서 등록하여 이 같은 필요가 있을 때 마다 소스 코드에서 단순히 `import` 하여 사용하기 위해서 필요한 작업들을 알아보자. 



#### setup.py 작성

작업경로를 ```rospy_tutorial``` 로 변경

```bash
$ roscd rospy_tutorial
```

`setup.py` 작성

```bash
$ gedit setup.py &
```

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['my_lib'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```

이  `setup.py` 파일을 일반적인 파이썬 스크립트처럼 `$ python setup.py` 또는,  `$ setup.py install` 와 같이 실행해서는 절대 않된다. 이  `setup.py` 파일은 `catkin_make` 가 실행될 때, `catkin` 워크스페이스의 `devel` 폴더안에 `Makefile` 을 생성할 때 사용되는 파일이기 때문이다. 

- `package_dir={'': 'src'},`

  이 ROS 노드 패키지 폴더의 `src` 폴더에 사용자 정의 파이썬 라이브러리가 있음을 알려준다.

- `packages=['my_lib'],`

  라이브러리 이름은`my_lib` 이다.

`catkin_create_pkg` 명령이 실행 되었을 때, 이미 `src` 폴더가 생성되어 있으므로 바로 해당 폴더로 이동한다.

```bash
$ cd src
```

`my_lib` 라이브러리 폴더를 생성하고 생성된 `my_lib` 폴더로 작업경로를 변경한다.

```bash
$ mkdir my_lib && cd my_lib
```

현재 폴더에 있는 `*.py` 파일들이 '파이썬 라이브러리' 라는 것을 나타내는 `__init__.py` 파일 생성.

```bash
$ touch __init__.py
```

이 문서 시작 부분에 언급한 `GetChar.py` (키 입력 처리 사용자 라이브러리) 작성.

```bash
$ gedit GetChar.py &
```



#### CMakelist.txt 편집

```rospy_tutorial``` 패키지 폴더로 경로 변경

```bash
$ roscd rospy_tutorial
```

```CMakelist.txt``` 편집

```bash
$ roscd rospy_tutorial
```

```bash
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()  # <------------ uncomment this line

################################################
## Declare ROS messages, services and actions ##
################################################
```

변경된 `CMakelist.txt` 를 반영하기 위해 `rospy_tutorial` 패키지를 다시 빌드해야한다.

```bash
$ cd ~/catkin_ws && catkin_make
```

변경된 패키지 빌드 정보 반영.

```bash
$ source ./devel/setup.bash
```



### 스크립트 작성

```rospy_tutorial/scripts``` 폴더로 작업 경로 변경.

```
$ roscd rospy_tutorial/scripts
```

`remote_ctrl_turtle.py` 작성.

```
$ gedit remote_ctrl_turtle.py &
```

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from my_lib.GetChar import GetChar  # <----- this works by 'setup.py'

msg = """
---------------------------------------
              (forward)
                 'w'

  (left)'a'      's'       'd'(right)
              (backward)
---------------------------------------
type 'Q' for quit program...
---------------------------------------
"""

if __name__ == '__main__':

    rospy.init_node('remote_turtle')

    pub  = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    tw   = Twist()
    rate = rospy.Rate(10)
    kb   = GetChar()

    tw.linear.x  = tw.linear.y  = tw.linear.z  = 0.0
    tw.angular.x = tw.angular.y = tw.angular.z = 0.0

    count = ch = 0

    print msg

    while not rospy.is_shutdown():
        ch = kb.getch()

        if   ch == 'w':
            tw.linear.x  =  2.0;    print "forward"
        elif ch == 's':
            tw.linear.x  = -2.0;    print "backward"
        elif ch == 'a':
            tw.angular.z =  2.0;    print "turn left"
        elif ch == 'd':
            tw.angular.z = -2.0;    print "turn right"
        elif ch == 'Q':             break
        else:                       pass

        pub.publish(tw)
        tw.linear.x  =  tw.angular.z = 0.0

        count = count + 1
        if count == 15:
            count = 0;    print msg

        rate.sleep()
```

작성한 스크립트에 실행 속성을 부여.

```
$ chmod +x remote_ctrl_turtle.py
```



#### 스크립트 동작 확인

`roscore` 실행

```bash
$ roscore
```

새로운 터미널 창( `Ctrl` + `Alt` + `T` )에서 `turtlesim` 노드 실행

```bash
$ rosrun turtlesim turtlesim_node
```

새로운 터미널 창( `Ctrl` + `Alt` + `T` )에서 `remote_ctrl_turtle.py` 스크립트 실행

```bash
$ rosrun rospy_tutorial remote_ctrl_turtle.py
```

`turtlesim` 노드 실행 시`remote_ctrl_turtle.py` 스크립트 실행창에서 `w` ,  `s` ,  `a` ,  `d` 를 입력하여 `turtlesim` 구동 시 열린 창의 거북이가 제어되는 지 확인한다. 



---

 [튜토리얼 목록 열기](../README.md)                                                                   [다음 튜토리얼](./rospy_1_WritingPubSub.md)


