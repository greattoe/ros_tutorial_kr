## rqt/ Tutorials/ Writing a Python Plugin



------

## Writing a Python Plugin

**튜토리얼 레벨 :**  Beginner(초급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [Creating a_rqt_Plugin pkg](./rqt_1_create_rqt_plugin_pkg.md)

**다음 튜토리얼 :** [Writing_Python_Plugin](./rqt_3_adding_physics_n collisons.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin>

------



### 1. Intro

이 튜토리얼에서는 사용자 지정 UI( User Interface )를 ROS의 GUI 프레임워크인 rqt에 통합하기 위한 플러그인을 생성하는 방법을 보여 줄 것이다. [rqt 플러그인 패키지 생성 튜토리얼](./rqt_1_create_rqt_plugin_pkg.md)을 마치고 이 튜토리얼을 시작하는 것을 전제로 진행한다.

이 튜토리얼에서 사용하는 예제 코드는 [github](https://github.com/lucasw/rqt_mypkg) 에서 찾아볼 수 있다.  ( 예: [rqt_bag](https://github.com/ros-visualization/rqt_common_plugins/tree/groovy-devel/rqt_bag) )



### 2. The code

이 전 튜토리얼에서 만든 패키지( rqt_mypkg ) 폴더의 src 폴더 안에 패키지 이름과 같은 rqt_mypkg 폴더를 만든다. 

```
% roscd rqt_mypkg
% mkdir -p src/rqt_mypkg
```

만들어진 `~/catkin_ws/src/rqt_mypkg/src/rqt_mypkg` 폴더 안에 파일명이 `__init__.py` 인 빈 파일을 하나 생성한다.

```
% cd src/rqt_mypkg
% touch __init__.py
```

그리고나서, 같은 폴더에 `my_module.py` 파일을 만들고 다음 코드를  `my_module.py` 의 내용으로 작성하고 저장한다.

```python
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
```

This simple plugin only adds a single widget using `add_widget`. But a plugin can contribute multiple widgets by calling that method for each widget. The method can be called at any time to dynamically contribute more widgets. Internally each widget is embedded into a new QDockWidget which itself is added to the QMainWindow.



#### 2.1 Adding the script file

시작하기 전에 `rqt_mypkg` 라는 이름의 빈 패키지를 `~/catkin_ws/src` 에 하나 만들자.

```xml
% roscd rqt_mypkg
% mkdir -p src/rqt_mypkg
```

Then in the same folder, create `rqt_mypkg` file and paste all of the following code inside it:

```python
#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_mypkg'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
```



#### 2.2 Using qt_binding_helper

The `python_qt_binding` enables to write code which is agnostic to the used Python bindings for Qt. You should import all Qt modules with the `python_qt_binding` prefix instead of the `PyQt4`/`PySide` prefix. For more information about licensing regarding Qt-python binding, see wiki page of [python_qt_binding](http://wiki.ros.org/python_qt_binding) package.



#### 2.3 Using a UI file

The example uses an Qt Designer UI file to describe the widget tree. Instead of hand-coding the widget tree the `python_qt_binding.loadUi` function creates the widgets at runtime based on the description from the file. See [this tutorial](http://wiki.ros.org/rqt/Tutorials/Using .ui file in rqt plugin) for more info.

For this tutorial, we will assume that a .ui file created from QT Designer is available.

Create the `resource` folder inside the `rqt_mypkg` package directory.

```
% roscd rqt_mypkg
% mkdir -p resource
```

In the same `resource` folder, create `MyPlugin.ui` file and paste all of the following code inside it:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>Form</class>
 <widget class="QWidget" name="Form">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>400</width>
    <height>300</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>Form</string>
  </property>
  <widget class="QPushButton" name="Test">
   <property name="geometry">
    <rect>
     <x>120</x>
     <y>70</y>
     <width>98</width>
     <height>27</height>
    </rect>
   </property>
   <property name="text">
    <string>Original Name</string>
   </property>
  </widget>
 </widget>
 <resources/>
 <connections/>
</ui>
```



#### 2.4 trigger configuration

As a method shown in the code example above, each rqt plugin can have a configuration dialog (opened from gear-shape icon).

Examples from [rqt_service_caller](https://github.com/ros-visualization/rqt_common_plugins/blob/efc7c7e30533c598869943fb400a5744c7d30881/rqt_service_caller/src/rqt_service_caller/service_caller.py#L54) | [rqt_plot](https://github.com/ros-visualization/rqt_common_plugins/blob/efc7c7e30533c598869943fb400a5744c7d30881/rqt_plot/src/rqt_plot/plot.py#L180).



### 3. Using rospy

The plugin should not call `init_node` as this is performed by `rqt_gui_py`. The plugin can use any [rospy](http://wiki.ros.org/rospy)-specific functionality (like Publishers, Subscribers, Parameters). Just make sure to stop timers and publishers, unsubscribe from Topics etc in the `shutdown_plugin` method.

Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks, because they are running in a different thread. In the ROS callback you can:

- emit a Qt signal (which will bridge across the threads) and manipulate the widgets in the receiving slot

OR

- only operate on non-widget entities like `QAbstractItemModels`



### 4. Once code is done

You can run your plugin from the rqt main window's 'plugin' menu or standalone like this:

```
$ rqt --standalone rqt_mypkg
```

For more options you may go back to [Install & Run your plugin section in the previous tutorial](http://wiki.ros.org/rqt/Tutorials/Create your new rqt plugin#Install_.26_Run_your_plugin).



### 5. Misc

#### 5.1 python coding style for rqt

- Follow [PyStyleGuide](http://wiki.ros.org/PyStyleGuide) in general.
- `rqt` local custom is whether you follow 80-char-in-a-line rule from PEP8 is up to you when writing a plugin. Existing code should not be changed and must be consistent per package.

Also, `python` in `rqt` defines documenting rule ([discussion](https://github.com/ros-visualization/rqt/issues/55)):

- [PEP257](http://www.python.org/dev/peps/pep-0257/) & [PEP258](http://www.python.org/dev/peps/pep-0258/) leave up the detailed format to each project

- Most of the rqt plugins so far look like using [Sphinx](http://sphinx-doc.org/markup/desc.html) format

- For attributing, both `:` and `@` can be used. For example:

  ```
   @param argfoo: this is argfoo
  ```

  is equal to:

  ```
   :param argfoo: this is argfoo
  ```



---

[이전 튜토리얼](./rqt_1_create_rqt_plugin_pkg.md) &nbsp; &nbsp; / &nbsp; &nbsp; [튜토리얼 목록](../README.md) &nbsp; &nbsp; / &nbsp; &nbsp;  [다음 튜토리얼](./rqt_3_adding_physics_n collisons.md)