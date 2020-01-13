## bebop_autonomy / Parrot-Sphinx



---

## Parrot-Sphinx guide book

**튜토리얼 레벨 :**  Intermediate(중급)(수정)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [링크 수정 필요]()

**이전 튜토리얼 :** [링크 수정 필요]()

**자료원본(출처) :** <https://developer.parrot.com/docs/sphinx/>

**목록보기:** [README.md](../README.md)

---

이 문서는 [Parrot-Sphix guide book](https://developer.parrot.com/docs/sphinx/) 을 참조하여,

Parrot Bebop2 드론을 개발 타겟으로, 

ROS Kinetic Kame 및 bebop_autonomy 가 설치된 Ubuntu 16.04 (xenial) 로 운영되는 PC 에  `Parrot-Sphinx` 를 이용한 개발환경을  구축하는 방법을 설명한다. 



### 1. Parrot-Sphinx는...

`Parrot-Sphinx` 는 원래 드론 SW 를 개발하는 Parrot 社의 엔지니어를 위해 만든 시뮬레이션 툴이다. 

호스트시스템과 격리( 분리 )된 환경의 PC 에서 Parrot 드론 펌웨어를 실행하는 것이 주요개념으로, 이를 위해 `ROS` 의 `Gazebo` 를 통해 드론의 물리, 시각적 환경을 시뮬레이션하며 다음과 같은 기능을 제공한다.

* 웹브라우져를 통한 비행 데이터 시각화( Visualization )
* 동작 중 드론 제어
* 터미널을 통한 
* 실제 드론에 사용되는 조종앱( Freefligt 등 )을 이용한 제어
* PC에 연결된 카메라( 웹캠 등 )를 이용한 비디오 스트리밍
* 원격 서버를 통한 실행



### 2. 시스템 요구사양 

* `Parrot-Sphinx` 는 리눅스( 64bit )에서만 실행될 수 있으며, 그 중에서도 다음 배포판에서 보다 정확히 작동한다. 
  
  * Ubuntu 16.04 (xenial) - 이 문서의 작성환경
  * Ubuntu 18.04 (bionic) - 권장 사항
  * Debian 9 (stretch)
  * Debian 10 (buster)
  
* 최소 1GB의 디스크 공간

* 버전 3.0 이상의 OpenGL 지원

  ```
  $ glxinfo | grep "OpenGL version"
  OpenGL version string: 3.0 Mesa 18.0.5
  ```

* WiFi Adapter 

  시뮬레이션 된 Parrot 드론 연결을 위한 AP모드를 지원하는 무선 랜카드가 필요하다. 
  
  실제 드론에 연결할 때에도 AP모드로 동작하는 드론의 WiFi에 연결한다. 그처럼 이 무선랜카드를 이용하여 시뮬레이션된 드론에 연결하여 실제 드론대신에 작성한 코드로 동작시켜 볼 수 있다. 
  
  따라서 AP모드( `hostapd` )가 지원되는 무선랜카드를 사용해야 하는데, AP모드( `hostapd` )가 지원되더라도 실제로 동작하지 않는 경우가 있으므로 다음 호환 제품목록을 참조하여 준비할 것을 권한다.
  
  * **Parrot-Sphinx 구동이 테스트된 WiFi어댑터 목록 링크**
    * [USB 방식](https://developer.parrot.com/docs/sphinx/system-requirements.html#id3)
    * [PCI 방식](https://developer.parrot.com/docs/sphinx/system-requirements.html#id4) 
    * 위에는 없지만 테스트 결과 RTL8188CU, MT7610U 칩셋을 탑재한 USB 방식 제품들의 동작 확인.

* 인터넷 연결( 온 라인 )

  위에 설명한 시뮬레이션된 Parrot 드론( ex: Bebop2, ANAFI 등 ) 연결을 위한 WiFi 어댑터와는 별도의 인터넷연결이 필요하다. 시뮬레이션을 위해 인터넷을 통해 펌웨어를 다운받아 실행하기 위함이다.





### 3. 설치

#### 3.1 repository 주소 및 key 추가

`/ets/apt/sources.list.d` 파일에 `Parot-Sphinx` 저장소 주소 추가

```
$ echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
```

`Parot-Sphinx` 저장소 접근을 위한 키 등록

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
```

#### 3.2 Parrot-Sphinx 패키지 설치

변경된 `/ets/apt/sources.list.d` 파일 반영을 위한 업데이트

```
$ sudo apt-get update
```

`Parot-Sphinx` 바이너리 패키지 설치

```
$ sudo apt-get install parrot-sphinx
```



### 4. 구동을 위해 필요한 작업

#### 4.1 'bebop2.drone' 파일 수정

적당한 편집기를 이용하여 `/opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone` 파일을 수정해야 한다. 이 작업을 위해서는 `Parrot-Sphinx` 로 시뮬레이션된 드론에 연결하기위한 WiFi 네트워크 인터페이스  이름이 필요하다. `ifconfig` 명령으로 알아낸다.

```
$ ifconfig
en0123456 Link encap:Ethernet  HWaddr 00:11:22:33:44:55  
          inet addr:xxx.xxx.xxx.71  Bcast:xxx.xxx.xxx.255  Mask:255.255.255.0
                 .
                 .
                 .
                 
lo        Link encap:Local Loopback  
          inet addr:127.0.0.1  Mask:255.0.0.0
      |          .
      |          .
      v          .
_______________  
wlxaabbccddeeff Link encap:Ethernet  HWaddr aa:bb:cc:dd:ee:ff  <--- 앞에 표시한 인터페이스 이름
          UP BROADCAST MULTICAST  MTU:1500  Metric:1
                 .
                 .
                 .
```

위의 `ifconfig` 명령 실행 결과를 보면 `en0123456` , `lo` , `wlxaabbccddeeff` 모두 3개의 네트워크 인터페이스가 보인다. 이 중 `w` 문자로 시작하는 `wlxaabbccddeeff` 가 WiFi 네트워크 인터페이스이름이다. 

 `/opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone` 파일 편집. 

```
$ sudo nano /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
```

`bebop2.drone` 파일의 내용은 다음과 같다.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<drone
  name="bebop2"
  firmware="http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/latest/images/
  ardrone3-milos_pc.ext2.zip"
  hardware="milosboard">
  <machine_params
    low_gpu="0"
    with_front_cam="1"
    with_hd_battery="0"
    with_flir="0"
    flir_pos="tilted"/>
  <pose>default</pose>
  <interface>eth1</interface>
  <!-- 'wlan0' may need to be replaced the actual wifi interface name ------------------->
  <stolen_interface>wlan0:eth0:192.168.42.1/24</stolen_interface><!-- modify this line -->
  <!-- stolen_interface>wlxaabbccddeeff:eth0:192.168.42.1/24</stolen_interface ---------->
</drone>
```

위에 주석으로 표시한 것 중 `<stolen_interface>wlan0:eth0:192.168.42.1/24</stolen_interface>` 부분을  `<stolen_interface>wlxaabbccddeeff:eth0:192.168.42.1/24</stolen_interface>` 와 같이 자신이 사용하는 네트워크 인터페이스 이름으로 변경 후 저장, 종료한다. 

시뮬레이션이 원활하지 않을 경우 

 `low_gpu="0"` 를 `low_gpu="1"` 로,  `with_front_cam="1"` 은 `with_front_cam="0"` 로 변경하면 효과가 있다. 

`with_hd_battery="0"` 는 시스템 성능과 상관없이 `with_hd_battery="1"` 로 변경하는 것이 유리하다.



#### 4.2 'bebop_node.launch' 수정

`~/catkin_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node.launch` 파일의 `name` 속성이 `ip` 인 `<arg>` 태그의 `default` 속성의 값을 `192.168.42.1 (실제 드론의 IP )` 에서 `10.202.0.1(시뮬레이션 드론의 ip` 로 변경한다.

```
$ gedit ~/catkin_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node.launch
```

`bebop_node.launch` 파일의 내용은 다음과 같다.

```xml
<?xml version="1.0"?>
<launch>
    <arg name="namespace" default="bebop" />
    <arg name="ip" default="192.168.42.1" /> <!-- change this to "10.202.0.1" 로 변경-->
    <arg name="drone_type" default="bebop2" /> 
    <arg name="config_file" default="$(find bebop_driver)/config/defaults.yaml" />
    <arg name="camera_info_url" default="package://bebop_driver/data/$(arg drone_type)_camera_calib.yaml" />
    <group ns="$(arg namespace)">
        <node pkg="bebop_driver" name="bebop_driver" type="bebop_driver_node" output="screen">
            <param name="camera_info_url" value="$(arg camera_info_url)" />
            <param name="bebop_ip" value="$(arg ip)" />
            <rosparam command="load" file="$(arg config_file)" />
        </node>
        <include file="$(find bebop_description)/launch/description.launch" />
    </group>
</launch>
```

```<arg name="ip" default="192.168.42.1" />``` 부분을 ```<arg name="ip" default="10.202.0.1" />``` 와 같이 ( 주석에 표시해둔 것과 같이 ) 변경, 저장한다.



### 5. Parrot-Sphinx 구동

#### 5.1 펌웨어 서비스 구동

스핑크스에서 드론 펌웨어 파일을 구동하려면 리눅스 펌웨어 서비스가 필요하다. 다음과 같이 실행하면된다.

```
$ sudo systemctl start firmwared.service
```

PC 를 껏다 켰거나, 리부팅한 경우 다시 실행 주어야 한다.



#### 5.2 스핑크스 구동

먼저 인터넷 연결을 확인한 후 다음과 같이 스핑크스를 구동한다.

```
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
```

정상적으로 구동된 경우 아래와 같은 화면을 볼 수 있다. 하지만 아직 드론을 움직일 수 없다. 드라이버 노드가 아직 구동되지 않았기 때문이다.

![](../img/running_sphinx.png)



#### 5.3 'bebop_node.launch' 파일 실행

4.2 에서 `bebop_node.launch` 파일을 수정해 두었었다. 이  `bebop_node.launch` 파일을 구동한다. 

```
$ roslaunch bebop_driver bebop_node.launch
```

에러없이 구동에 성공했다면 `rostopic` 명령으로 `/bebop/takeoff` 라는 토픽명으로 `std_msgs/Empty` 형식의 토픽을 발행하여, 화면의 `bebop2` 드론을 이륙시켜 보자.

```
$ rostopic pub /bebop/takeoff std_msgs/Empty
```

아래 화면과 같이 화면 속의 드론이 이륙한 것을 볼 수 있다. ( 착륙은 토픽명만  `/bebop/land`로 변경 실행한다. )

![](../img/sphinks_takeoff.png)

시뮬레이션이 아닌 실제 bebop2 드론에 대한 코드를 작성하고 테스트하기 위해서는

1. **드론을 켜고,** 
2. **WiFi를 통해 드론과 연결하고,**
3. **`roscore` 를 구동한다. 그리고 나서,**
4. **`bebop_driver`  패키지의 `bebop_node.launch` 파일을 구동한 후에** 
5. **작성한 코드를 실행하여 드론을 동작시켜 본다.**

와 같은 순서로 그 동안 작업해 왔다. 

스핑크스를 이용하여 같은 작업을 할 경우에는

1. **리눅스 펌웨어 서비스를 구동하고,**
2. **스핑크스를 구동한다. 그리고 나서,**
3.  **`bebop_driver`  패키지의 `bebop_sphinx.launch` 파일을 구동한 후에** 
4. **작성한 코드를 실행하여 드론을 동작시켜본다.**

의 순서로 한다.



[튜토리얼 목록 열기](../README.md)


