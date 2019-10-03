## rqt/ Tutorials/ Create your new rqt plugin



------

## Create your new rqt plugin

**튜토리얼 레벨 :**  Beginner(초급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [Writing_Python_Plugin](./rqt_2_writing_python_plugin.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin>

------



### 1. Intro

이 튜토리얼에서는 사용자 지정 UI( User Interface )를 ROS의 GUI 프레임워크인 rqt에 통합하기 위한 플러그인을 생성하는 방법을 보여 줄 것이다.

코드 작성 후 작성한 코드의 실행 및 기타 사항은 [rqt/User Guide](https://github.com/ros-visualization/rqt_common_plugins)의 내용을 따른다.

이 튜토리얼에서 사용하는 모든 코드 세트는 [github](https://github.com/lucasw/rqt_mypkg) 에서 찾아볼 수 있다. 이들 [rqt 플러그인](https://github.com/ros-visualization/rqt_common_plugins)들은 매우 쓸모가 많은데, 그 이유는 이 플러그인들이 비어있는 위젯을 불러내기 때문이다.  [rqt_bag](https://github.com/ros-visualization/rqt_common_plugins/tree/groovy-devel/rqt_bag) 가 바로 그 사례이다.

설계 지침은 [사용 가능 리소스](http://wiki.osrfoundation.org/Usability)를 참조하십시오.



### 2. Prerequisite & assumption( 전제조건과 가정 )

- rqt 가 이미 설치되어 있다. ( [rqt 설치](http://wiki.ros.org/rqt/UserGuide#Installation) )
- rqt를 사용하여 ROS에 통합할 QWidget 기반 GUI가 확보되어 있다.
- ROS Groovy 이상의 ROS 버전과 및 catkin을 빌드 시스템으로 가정한다.
- 이 튜토리얼은 우분투 12.10( 2013. 03. 12 )를 기본으로 작성되었다.
- ![(!)](../img/idea.png)이 튜토리얼의 예제는 `python`을 사용한다. `C++` 사용을 위해서는 `rqt_gui_py`를  `rqt_gui_cpp`로 교체한다.



### 3. Steps to create rqt plugin pkg

#### 3.1 내용 없는 빈 패키지의 생성

시작하기 전에 `rqt_mypkg` 라는 이름의 빈 패키지를 `~/catkin_ws/src` 에 하나 만들자.

```xml
catkin_create_pkg rqt_mypkg rospy rqt_gui rqt_gui_py
```

#### 3.2 'package.xml' 편집

##### 3.2.1 'export' 태그 추가

catkin 빌드 시스템이 작성할 플러그인을 검색할 수 있도록 `package.xml` 에 플러그인을 선언한다. 

```xml
<package>
  :
  <!-- all the existing tags -->
  <export>
    <rqt_gui plugin="${prefix}/plugin.xml"/>
  </export>
  :
</package>
```

[전체 'package.xml' 파일 예](https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_bag/package.xml) 

##### 3.2.2 build_depend 제거( 선택 )

플러그인의 소스코드를 파이썬으로 작성한다면 파이썬은 빌드 자체가 필요하지 않으므로 [build_depend](http://wiki.ros.org/catkin/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.dependencies_tags) 태그를 생략할 수 있다.

#### 3.3 'plugin.xml' 파일 생성

그런 다음 플러그인에 대한 추가 메타 정보를 제공할 참조 파일 plugin.xml을 생성한다.

```xml
<library path="src">
  <class name="My Plugin" type="rqt_mypkg.my_module.MyPlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
      An example Python GUI plugin to create a great user interface.
    </description>
    <qtgui>
      <!-- optional grouping...
      <group>
        <label>Group</label>
      </group>
      <group>
        <label>Subgroup</label>
      </group>
      -->
      <label>My first Python Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>Great user interface to provide real value.</statustip>
    </qtgui>
  </class>
</library>
```

작성한 플러그인의 배치는 사용가능한 플러그인과 그들의 그룹화를 고려하여 결정한다.

[전체 예제](https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_bag/plugin.xml) 

##### 3.3.1 'plugin.xml' 파일의 라이브러리 속성 요소

플러그인 작동을 위해 필요한 경우 어디에서나 단지 example을 복사하고 수정하면 된다. 다음은 좀더 자세히 알고자하는 사람들을 위한 몇 가지 라이브러리 요소( element )의 xml 속성에 대한 설명이다.

- `/library@path` 

  시스템 경로에 추가되는 패키지의 상대경로

- `/library/class@name` 

  플러그인 명(이름)으로, 같은 패키지 안에서 유일한(중복되지 않는) 이름이어야 한다.

- `/library/class@type` 

  import 문에 사용되는 패키지, 모듈 및 클래스의 연결된 이름. (예 : package.module.class)

- `/library/class@base_class_type` 

  rospy 클라이언트 라이브러리를 사용하는 Python 플러그인의 경우 그 값은 `rqt_gui_py::Plugin` 이다.

- `/library/description` 

  플러그인에 대한 설명.

- `/library/qtgui` 

  이 태그에는 패키지에 대한 추가 옵션 정보가 포함되어 있다. 제공되지 않을 경우 플러그인 이름이 메뉴에서 플러그인의 레이블로 사용되고 설명이 상태 팁으로 표시된다.

  - `/library/qtgui/group` 

    플러그인을 그롭으로 묶는 기능을 활성화 하라. 그룹 태그는  `label`, `icon` 과 `statustip` 태그를 포함할 수 있다. 그룹은 플러그인이 나무가지의 잎사귀처럼 추가되는 메뉴의 계층 구조를 형성한다.

    다른 플러그인 그룹은 플러그인의 레이블에 기초하여 병합된다.  ( `icon` 과 `statustip` 은 그들이 다르게 정의 될 때, 다른 플러그인에 의해 재정의( override )될 수 있다. )

  - `/library/qtgui/label` 

    레이블을 메뉴에 표시되는 플러그인으로 바꾼다.

  - `/library/qtgui/icon` 

    레이블 옆에 표시될 `icon` 을 정의한다. ( 타입과 속성에 따라 달라질 수 있다. )

  - `/library/qtgui/icon@type` 
  
    - `file` (default): 패키지 경로에 대한 `icon` 으로 사용할 이미지 파일의 상대 경로
  - `theme`:  `icon`  [Icon Naming Specification](http://standards.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html) 에 의해 정해진 이름
    
- `resource`:  `icon`  Qt 리소스에 대한 명칭
    
  - /library/qtgui/statustip
  
    마우스 포인터가 플러그인 레이블 위에 위치할 때 나타나는 상태 팁 재정의



### 4. Write a plugin code

[파이썬을 이용한 rqt 플러그인 제작](http://wiki.ros.org/rqt/Tutorials/Writing a Python Plugin)

#### 4.1 Coding rule for rqt

- 대부분 일반적인 [ROS coding style guide python](http://wiki.ros.org/PyStyleGuide) 의 내용을 따른다.
- 의존성 목록은 알파벳 순으로 import 한다.
- [파이썬을 이용한 rqt 플러그인 제작에 대한 몇 가지 규칙들](http://wiki.ros.org/rqt/Tutorials/Writing a Python Plugin#python_coding_style_for_rqt)

[여기](http://wiki.ros.org/rqt/Tutorials/Writing a Python Plugin#python_coding_style_for_rqt)에도  `rqt` 플러그인 제작에  `python` 을 사용하는 경우 지켜야 할 것들에 대해 정의해 놓은 문서가 있다.

#### 4.2 Choice of programming language in rqt

유지보수의 편의성 때문에 많은 rqt 플러그인들이 python으로 씌여졌다. 또 매우 강하게 새로운 플러그인은 python으로 작성할 것을 권고하고 있다. `C++` 또한 `rqt` 플러그인 제작을 완벽히 지원하지만, 다음의 경우에 한해서만 사용하길 바란다.

- `C++` 에서만 가능한 성능이 발휘되어야 하거나, `C++` 에서만 지원하는 라이브러리를 사용해야 하는 경우

  ( 예: [rqt_image_view](http://wiki.ros.org/rqt_image_view) )

-  `Python` 에 비해 `C++` 이 훨씬 더 익숙할 경우

기존의 `rqt plugin`들이 어떤 언어를 사용하여 만들어 졌는지는 [rqt/Plugins](http://wiki.ros.org/rqt/Plugins) 페이지에서 확인할 수 있다. 또한 각 플러그인 페이지마다 repository 가 표시되어 있으므로, 필요한 경우 해당 플러그인의 소스코드도 살펴 볼 수 있다. 



### 5. Install & Run your plugin

제작한 플러그인을 실행하는 방법은 [Running rqt](http://wiki.ros.org/rqt/UserGuide#Running_rqt) 섹션에 설명되어 있다.

`catkin`을 사용하면 위의 링크의 어떤 방법을 사용하든 플러그인을 실행하고 싶을 때는 CMake를 통해 설치해야만 한다. CMake가 PATH가 설정되어 있지 않은 특정 폴더의 스크립트를 패키지에 포함시켜 줄 수 있기 때문이다.

`setup.py`( [참조](http://answers.ros.org/question/50661/catkin-setuppy-installation-into-devel-space-not-working/) )에 매크로를 추가한다.  다음은 매크로가 추가된`setup.py` 파일의 예이다. 

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    package_dir={'': 'src'},
)

setup(**d)
```

[CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)의 다음 라인의 주석을 해제도 잊은면 않된다.

```
catkin_python_setup()
```

다음 예 처럼, ROS에서 실행 가능하다고 선언된 위치에 스크립트를 위치시키는 `install` 매크로를 추가한다.

```xml
install(PROGRAMS scripts/rqt_mypkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

다음 행을 추가하여 필요한 리소스와 `plugin.xml` 을 호출한다.

```xml
install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

이제 다음 명령으로 작성한 스크립트를 실행할 수 있다.

```
$ rosrun rqt_mypkg rqt_mypkg
```

![<!>](../img/attention.png) 혹시 rqt를 실행할 때 플러그인 메뉴에 플러그인이 표시되지 않을 경우, 아래 명령을 실행해야 할 수도 있다. 

```
rqt --force-discover
```



### 6. Option

- 모든 ROS 패키지에서 플러그인이 동작할 수 있도록  `package.xml` 에 충분한 정보를 추가하는 것을 잊지 말자.

#### 6.1 Unit testing rqt plugins

말할 필요도없이, 하지 않으면 안되는 것은 아니지만 단위 테스트 코드를 만들고 유지하는 것을 강력히 권고한다. 

- 단위 테스트코드를 저장할 최적의 장소는 패키지 폴더 바로 아래의 `/test` 폴더이다.
- 단위 테스트에서 다루는 영역은 비즈니스와 응용 프로그램 로직에 한정될 수 있다. 
- visual components ([ref](http://www.theregister.co.uk/2007/10/22/gui_unit_testing/))에 대한 단위테스트 수행에 있어 별 다른 편리한 방법은 아직까지는 없는 것으로 보인다.
  - 좀 더 나은 아이디어가 있다면 [rqt community](http://ros.org/wiki/rqt/#Community)에 토론 게시판을 개설해 주길 바란다. 이 주제에 대해서는 많은 사람들이 관심을 가지고 있기 때문이다. 
- 단위 테스트 완료 후, 통합 테스트는 [rosunit](http://wiki.ros.org/rosunit)을 이용할 것을 강력히 추천한다. 

#### 6.2 To run your plugin directly

모든 테스트를 완료한 `rqt` 플러그인을 시스템 `PATH` 에 추가하면,  `rosrun`, `rqt_gui` 등 과 같은 추가도구 없이도 어디서나 실행가능하게 만들 수 있다. 하지만 이 방법은 권장하지 않는다. 전체 시스템이 같이 사용하는 리소스 공간을 깔끔하게 유지하기 위해서이다. 그럼 추가도구 없이 어디서나 `rqt` 플러그인을 실행할 방법은 없는가? 한 가지 방법응 소개한다.

![<!>](../img/attention.png) 커스텀 `rqt` 플러그인을 직접 실행하는 것은 권장하지 않는다. ( 관련 토론 게시물 [1](https://github.com/ros-visualization/rqt_common_plugins/issues/1#issuecomment-11919157), [2](https://groups.google.com/d/msg/ros-sig-rqt/QkqYjMJ0dZk/A9R7FPdsFpEJ), [3](https://groups.google.com/d/msg/ros-sig-rqt/lIvOLIChRzo/3tccjWJ_0yMJ) ) 실제로 필요한 경우에만 이 방법을 사용하기 바랍니다.

 `setup.py` [(reference)](http://answers.ros.org/question/50661/catkin-setuppy-installation-into-devel-space-not-working/) 에 아래 한 줄을 추가한다.

```
scripts=['%RELATIVE_PATH_TOYOUR_EXECUTABLE%']
```

그 한 줄이 추가된 `setup.py` 파일의 내용은 다음 내용과 비슷할 것이다.

```python
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_mypkg']
)
```

 `setup.py` 파일 편집을 마쳤으면 다시 한 번 `catkin_make` 를 실행하여 다시 빌드한다.

```
$ cd %TOPDIR_YOUR_CATKIN_WS%
$ catkin_make
```

이 작업은 이미 `%TOPDIR_YOUR_CATKIN_WS%devel/setup.bash%` 에 대해 `source` 명령을 적용했다면 호출할수있는 연계 스크립트를 `%TOPDIR_YOUR_CATKIN_WS%devel/bin` 경로에 생성한다.

#### 6.3 Practices to follow on making rqt plugins

- 일반적인 GUI 개발과 같이 어느 쓰레드로부터 GUI 업데이트가 발생하는 지 주의를 기울여야 한다.
  - `rqt` 플러그인에서 인스턴스화 된 노드는 동일한 프로세스에서 다른 스레드로 실행되지만 노드 핸들러( 예를 들자면`C ++`에서의  `NodeHandle` 와 `rospy.Subscriber` 같은 )에 제공된 콜백 함수는 메인 쓰레드에서 실행되면서도 GUI를 업데이트 할 수 있다. 
- [MVC](http://en.wikipedia.org/wiki/Model–view–controller)와 같은 공통 GUI SW 아키텍처를 적용하라.
  - 같은 이유로 플러그인 클라스( `rqt_gui_cpp::Plugin` or `rqt_gui_py.plugin.Plugin` )와 위젯 실행을 이상적으로 분리해야 한다.

---

​                                                                                [튜토리얼 목록 열기](../README.md)                                                         [다음 튜토리얼](./tf_4_tf_n_time.md)
