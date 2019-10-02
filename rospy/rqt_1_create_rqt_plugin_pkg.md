## rqt/ Tutorials/ Create your new rqt plugin



------

## Create your new rqt plugin

**튜토리얼 레벨 :**  Beginner(초급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic./tf_2_listener.md)

**다음 튜토리얼 :** [Writing_Python_Plugin](./rqt_2_writing_python_plugin.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin>

------



### 1. Intro

이 튜토리얼에서는 사용자 지정 UI( User Interface )를 ROS' GUI 프레임워크 rqt에 통합하기 위한 플러그인을 생성하는 방법을 보여 줄 것이다.

코드 작성 후 작성한 코드의 실행 및 기타 사항은 [rqt/User Guide](https://github.com/ros-visualization/rqt_common_plugins)의 내용을 따른다.

이 튜토리얼에서 사용하는 모든 코드 세트는 [github](https://github.com/lucasw/rqt_mypkg) 에서 찾아볼 수 있다. 이들 [rqt 플러그인](https://github.com/ros-visualization/rqt_common_plugins)들은 매우 쓸모가 많은데, 그 이유는 이 플러그인들이 비어있는 위젯을 불러내기 때문이다.  [rqt_bag](https://github.com/ros-visualization/rqt_common_plugins/tree/groovy-devel/rqt_bag) 가 바로 그 사례이다.

설계 지침은 [사용 가능 리소스](http://wiki.osrfoundation.org/Usability)를 참조하십시오.



### 2. Prerequisite & assumption( 전제조건과 가정 )

- rqt 가 이미 설치되어 있다. ( [rqt 설치](http://wiki.ros.org/rqt/UserGuide#Installation) )
- rqt를 사용하여 ROS에 통합할 QWidget 기반 GUI가 확보되어 있다.
- ROS Groovy 이상의 ROS 버전과 및 catkin을 빌드 시스템으로 가정한다.
- 이 튜토리얼은 우분투 12.10( 2013. 03. 12 )를 기본으로 작성되었다.
- 이 튜토리얼의 예제는 파이썬을 사용한다. C++을 사용하기 위해서는 `rqt_gui_py` 를  `rqt_gui_cpp` 로 교체한다.





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

[전체 'package.xml' 파일의 예](https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_bag/package.xml) 

##### 3.2.2 build_depend 제거( 선택 )

플러그인의 소스코드를 파이썬으로 작성한다면 파이썬은 빌드 자체가 필요하지 않으므로 [build_depend](http://wiki.ros.org/catkin/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.dependencies_tags) 태그를 생략할 수 있다.

#### 3.3 plugin.xml 파일 생성

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

플러그인 작동을 위해 필요한 경우 어디에서나 단지 example을 복사하고 수정하면 된다. 다음은 좀더 자세히 알고자하는 사람들을 위한 몇 가지 라이브러리 요소의 xml 속성에 대한 설명이다.

- /library@path

- /library/class@name

- /library/class@type

- /library/class@base_class_type

- /library/description

- /library/qtgui

  이 태그에는 패키지에 대한 추가 선택적 정보가 포함되어 있다. 아무것도 제공되지 않을 경우, 메뉴에서 플러그인의 이름을 플러그인의 라벨로 사용하고 설명은 상태 팁으로 표시된다.

  - /library/qtgui/group 

    Enables grouping of plugins. Group tags can contain a `label`, `icon` and `statustip` tag. The groups form a hierarchy of menus where the plugin is added as the leaf. The groups of different plugins are merged based on their label (icons and statustip may be overridden by other plugins when they are defined differently).

  - /library/qtgui/label

    Overrides the label with which the plugin appears in the menu.

  - /library/qtgui/icon

    Defines the icon that should be shown beside the label (depending on the type of attribute).

  - /library/qtgui/icon@type

    - `file` (default): the icon value is a package-relative path to an image.
    - `theme`: the icon value names an icon defined by the [Icon Naming Specification](http://standards.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html).
    - `resource`: the icon value names a Qt resource.

  - /library/qtgui/statustip

    Overrides the status tip that is shown when hovering over the plugin label.



### 4. Write a plugin code

Writing code is explained on separate pages for [python](http://wiki.ros.org/rqt/Tutorials/Writing a Python Plugin) | [C++](http://wiki.ros.org/rqt/Tutorials/Writing a C%2B%2B Plugin) respectively.

#### 4.1 Coding rule for rqt

- Mostly follow the general ROS coding style guide [C++](http://wiki.ros.org/CppStyleGuide) | [Python](http://wiki.ros.org/PyStyleGuide)
- List dependency, import in an alphabetical order
- rqt in python [defines some rules](http://wiki.ros.org/rqt/Tutorials/Writing a Python Plugin#python_coding_style_for_rqt)

You can find out in which language the existing `rqt plugin`s are written at [rqt/Plugins](http://wiki.ros.org/rqt/Plugins) page. Go to the page of each plugin and find its source repository where you can look at the source code.



### 5. Install & Run your plugin

See the [Running rqt](http://wiki.ros.org/rqt/UserGuide#Running_rqt) section for how to run your plugin.

With `catkin`, no matter which method in the link above you want to run your plugin, you need to install it via `CMake` which puts the script into a package-specific folder which is not on the `PATH`.

Add macros to your `setup.py` [(reference)](http://answers.ros.org/question/50661/catkin-setuppy-installation-into-devel-space-not-working/). For example, after adding the line the section that contains it might look like :

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    package_dir={'': 'src'},
)

setup(**d)
```

Also make sure in your [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt), to uncomment a line:

```
catkin_python_setup()
```

Add `install` macro that puts the script into a location where it is rosrun-able is declared. For example:

```xml
install(PROGRAMS scripts/rqt_mypkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Add the following lines to call the resource and plugin.xml

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

These scripts can be run by:

```
$ rosrun rqt_mypkg rqt_mypkg
```

![<!>](../img/attention.png) If your plugin does not show up under the `Plugins` menu when you launch rqt, you may need to run:

```
rqt --force-discover
```



### 6. Option

- Do not forget to add enough info into `package.xml`, just as with every `ROS` package.

#### 6.1 Unit testing rqt plugins

Needless to say, making & maintaining `unit test` codes is strongly recommended, but not required.

- Unit test codes can be best stored under `/test` folder on the root directory of a package.
- Area covered by unit testing can only be business & application logic. There seems to be no convenient way to do unit test for visual components ([ref](http://www.theregister.co.uk/2007/10/22/gui_unit_testing/)).
  - If you have better idea, please open a discussion in [rqt community](http://ros.org/wiki/rqt/#Community). This is very interesting topic.
- For integrated test, using [rosunit](http://wiki.ros.org/rosunit) is highly recommended.

#### 6.2 To run your plugin directly

You can add your `rqt` plugin to the system `PATH` so that you can run it on-the-fly without using other tools such as `rosrun`, `rqt_gui` and so on. It is not, however, recommended to put it to `PATH` in order to keep system common space cleaner. Only if you dare to do so, there's a way.

![<!>](../img/attention.png) Running custom `rqt` plugins directly is NOT recommended (discussion [1](https://github.com/ros-visualization/rqt_common_plugins/issues/1#issuecomment-11919157), [2](https://groups.google.com/d/msg/ros-sig-rqt/QkqYjMJ0dZk/A9R7FPdsFpEJ), [3](https://groups.google.com/d/msg/ros-sig-rqt/lIvOLIChRzo/3tccjWJ_0yMJ)). Do this only when you're really in need.

Add 1 line:

```
scripts=['%RELATIVE_PATH_TOYOUR_EXECUTABLE%']
```

to your `setup.py` [(reference)](http://answers.ros.org/question/50661/catkin-setuppy-installation-into-devel-space-not-working/). For example after adding the line the section that contains it might look like :

```python
d = generate_distutils_setup(
    packages=['rqt_mypkg'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_mypkg']
)
```

Once you're done, run:

```
$ cd %TOPDIR_YOUR_CATKIN_WS%
$ catkin_make
```

This will yield a relay script to `%TOPDIR_YOUR_CATKIN_WS%devel/bin`, which you can call if you're already sourced `%TOPDIR_YOUR_CATKIN_WS%devel/setup.bash%` (or similar, as you wish).

#### 6.3 Practices to follow on making rqt plugins

- Same as general GUI development, you should pay attention to from which thread you're updating GUI.
  - Although nodes that are instantiated from `rqt` plugins run as different thread in the same process, callback function that is given to node handler (`NodeHandle` in `C++` / `rospy`.`Subscriber` (for example)) runs in the main thread, which enables to update GUI from there.
- Apply common GUI software architecture (eg. [MVC](http://en.wikipedia.org/wiki/Model–view–controller))
  - For the same reason, you should ideally separate `Plugin` class (`rqt_gui_cpp::Plugin` or `rqt_gui_py.plugin.Plugin`) and your widgets' implementation.

---





[튜토리얼 목록 열기](../README.md)



[다음 튜토리얼](./tf_4_tf_n_time.md)