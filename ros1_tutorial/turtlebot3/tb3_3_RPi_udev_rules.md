## Tuetlebot3

---

## SBC(Raspberry Pi) udev rules



**튜토리얼 목록 :** [README.md](../../README.md) 

---

터틀봇3의 SBC인 라즈베리파이에서 `ls /dev/tty*` 명령을 실행하면 아래와 같은 결과를 얻을 수 있다.

```
$ ls /dev/tty*
/dev/tty    /dev/tty18  /dev/tty28  /dev/tty38  /dev/tty48  /dev/tty58    /dev/ttyAMA0
/dev/tty0   /dev/tty19  /dev/tty29  /dev/tty39  /dev/tty49  /dev/tty59    /dev/ttyprintk
/dev/tty1   /dev/tty2   /dev/tty3   /dev/tty4   /dev/tty5   /dev/tty6     /dev/ttyS0
/dev/tty10  /dev/tty20  /dev/tty30  /dev/tty40  /dev/tty50  /dev/tty60    /dev/ttyUSB0
/dev/tty11  /dev/tty21  /dev/tty31  /dev/tty41  /dev/tty51  /dev/tty61    
/dev/tty12  /dev/tty22  /dev/tty32  /dev/tty42  /dev/tty52  /dev/tty62    
/dev/tty13  /dev/tty23  /dev/tty33  /dev/tty43  /dev/tty53  /dev/tty63    
/dev/tty14  /dev/tty24  /dev/tty34  /dev/tty44  /dev/tty54  /dev/tty7
/dev/tty15  /dev/tty25  /dev/tty35  /dev/tty45  /dev/tty55  /dev/tty8
/dev/tty16  /dev/tty26  /dev/tty36  /dev/tty46  /dev/tty56  /dev/tty9
/dev/tty17  /dev/tty27  /dev/tty37  /dev/tty47  /dev/tty57  /dev/ttyACM0
```

`tty` 는 단말기를 나타내는데, 이들 중 `/dev/ttyACM0` ,  `/dev/ttyAMA0` ,  `/dev/ttyUSB0` 는 시리얼 포트 들이다. 

- `/dev/ttyACM0` : OpenCR 보드가 연결된 USB to Serial 포트
- `/dev/ttyAMA0` : 시리얼 콘솔 연결 시 사용했던 라즈베리파이 GPIO 포트의 UART
- `/dev/ttyUSB0` : 터틀봇3 의 LIDAR 센서가 연결된 시리얼 포트 

Arduino Pro Micro (Leonardo 호환 보드) 를 연결 후, 다시 실행해보면 `/dev/ttyACM0` 와 함께  `/dev/ttyACM1` 이 나타난 것을 볼 수 있다. 항상 OpenCR 보드가  `/dev/ttyACM0` , Arduino Pro Micro 가  `/dev/ttyACM1` 에 할당되면 상관없지만 둘다 연결된 상태에서 전원을 켜면 어떤 보드가 어떤 이름을 할당 받을지 알 수가 없다. 이런 경우 구분 가능한 장치 속성을 이용하여 심볼릭 링크를 작성하면 편리하다.

다음은 리부팅 후 `udevadm info /dev/ttyACM0` 명령을 수행한 결과이다. 

```bash
$ udevadm info --name=/dev/ttyACM0 --attribute-walk

Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices. It prints for every device
found, all possible attributes in the udev rules key format.
A rule to match, can be composed by the attributes of the device
and the attributes from one single parent device.

  looking at device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.2/1-1.2:1.0/tty/ttyACM0':
    KERNEL=="ttyACM0"
    SUBSYSTEM=="tty"				# <------------
    DRIVER==""

  looking at parent device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.2/1-1.2:1.0':
    KERNELS=="1-1.2:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="cdc_acm"
    ATTRS{iad_bFunctionProtocol}=="00"
    ATTRS{iad_bInterfaceCount}=="02"
    ATTRS{bInterfaceNumber}=="00"
    ATTRS{bInterfaceClass}=="02"
    ATTRS{iad_bFirstInterface}=="00"
    ATTRS{bAlternateSetting}==" 0"
    ATTRS{bNumEndpoints}=="01"
    ATTRS{bInterfaceSubClass}=="02"
    ATTRS{supports_autosuspend}=="1"
    ATTRS{bmCapabilities}=="6"
    ATTRS{bInterfaceProtocol}=="00"
    ATTRS{authorized}=="1"
    ATTRS{iad_bFunctionSubClass}=="02"
    ATTRS{iad_bFunctionClass}=="02"

  looking at parent device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.2':
    KERNELS=="1-1.2"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{removable}=="unknown"
    ATTRS{rx_lanes}=="1"
    ATTRS{idVendor}=="2341"				# <------------		
    ATTRS{version}==" 2.00"
    ATTRS{maxchild}=="0"
    ATTRS{manufacturer}=="Arduino LLC"
    ATTRS{bDeviceClass}=="ef"
    ATTRS{bDeviceProtocol}=="01"
    ATTRS{bcdDevice}=="0100"
    ATTRS{quirks}=="0x0"
    ATTRS{tx_lanes}=="1"
    ATTRS{product}=="Arduino Leonardo"
    ATTRS{authorized}=="1"
    ATTRS{devnum}=="3"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{bDeviceSubClass}=="02"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{busnum}=="1"
    ATTRS{configuration}==""
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{bmAttributes}=="a0"
    ATTRS{bMaxPower}=="500mA"
    ATTRS{devpath}=="1.2"				# <------------
    ATTRS{idProduct}=="8036"			# <------------
    ATTRS{ltm_capable}=="no"
    ATTRS{bNumInterfaces}==" 2"
    ATTRS{urbnum}=="40"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{speed}=="12"
    ATTRS{devspec}=="  (null)"
    
	### omitted bellow ###
```

이 전과는 다르게 아두이노 보드에 `/dev/ttyACM0` 가 할당된 것을 알 수 있다. (위 내용 중 심볼릭 링크 작성시 필요해 보이는 속성에 `# <-----` 와 같이 표시를 해 두었으며, 뒷 부분은 생략하였다. )

다음은 `udevadm info /dev/ttyACM1` 명령을 수행한 결과이다. 


```bash
$ udevadm info --name=/dev/ttyACM1 --attribute-walk

Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices. It prints for every device
found, all possible attributes in the udev rules key format.
A rule to match, can be composed by the attributes of the device
and the attributes from one single parent device.

  looking at device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.4/1-1.4:1.0/tty/ttyACM1':
    KERNEL=="ttyACM1"
    SUBSYSTEM=="tty"				# <------------
    DRIVER==""

  looking at parent device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.4/1-1.4:1.0':
    KERNELS=="1-1.4:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="cdc_acm"
    ATTRS{authorized}=="1"
    ATTRS{bmCapabilities}=="2"
    ATTRS{bInterfaceClass}=="02"
    ATTRS{bInterfaceSubClass}=="02"
    ATTRS{bNumEndpoints}=="01"
    ATTRS{supports_autosuspend}=="1"
    ATTRS{bAlternateSetting}==" 0"
    ATTRS{bInterfaceNumber}=="00"
    ATTRS{bInterfaceProtocol}=="01"

  looking at parent device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.4':
    KERNELS=="1-1.4"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{busnum}=="1"
    ATTRS{product}=="OpenCR Virtual ComPort in FS Mode"
    ATTRS{idProduct}=="5740"		# <------------
    ATTRS{configuration}==""
    ATTRS{bNumInterfaces}==" 2"
    ATTRS{serial}=="FFFFFFFEFFFF"
    ATTRS{removable}=="unknown"
    ATTRS{authorized}=="1"
    ATTRS{devspec}=="  (null)"
    ATTRS{speed}=="12"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{quirks}=="0x0"
    ATTRS{tx_lanes}=="1"
    ATTRS{idVendor}=="0483"			# <------------
    ATTRS{bmAttributes}=="c0"
    ATTRS{devnum}=="5"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{version}==" 2.00"
    ATTRS{bcdDevice}=="0200"
    ATTRS{manufacturer}=="ROBOTIS"
    ATTRS{ltm_capable}=="no"
    ATTRS{bDeviceClass}=="02"
    ATTRS{maxchild}=="0"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{devpath}=="1.4"			# <------------
    ATTRS{bMaxPower}=="100mA"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{rx_lanes}=="1"
    ATTRS{bDeviceProtocol}=="00"
    ATTRS{bDeviceSubClass}=="00"
    ATTRS{urbnum}=="13"
    
	### omitted bellow ###
```

역시 이 전과는 다르게 OpenCR 보드에 `/dev/ttyACM1` 가 할당된 것을 알 수 있다. (이번에도 역시 내용 중 심볼릭 링크 작성시 필요해 보이는 속성에 `# <-----` 와 같이 표시를 해 두었으며, 뒷 부분은 생략하였다. )

이 정보를 토대로 장치에 대한 심볼릭 링크를 작성하기 위해 `/etc/udev/rules.d/99-usb-serial.rules` 파일을 편집해보자.

```bash
$ sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttyOpenCR"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", SYMLINK+="ttyLIFT"
```

편집한 내용을 저장, 종료하였으면 해당 내용을 반영하기 위해 다음 명령을 실행한다. 

```bash
$ sudo udevadm trigger
```

이제 `ls /dev/[symbolic link of device]` 명령으로 장치명을 확인한다. (이 문서의 경우 장치명이 `tty` 로 시작하도록 지정하였으므로 `ls /dev/tty*` 명령으로 확인하였다.)

```
$ ls /dev/tty*
/dev/tty    /dev/tty18  /dev/tty28  /dev/tty38  /dev/tty48  /dev/tty58    /dev/ttyACM1
/dev/tty0   /dev/tty19  /dev/tty29  /dev/tty39  /dev/tty49  /dev/tty59    /dev/ttyAMA0
/dev/tty1   /dev/tty2   /dev/tty3   /dev/tty4   /dev/tty5   /dev/tty6     /dev/ttyLIFT
/dev/tty10  /dev/tty20  /dev/tty30  /dev/tty40  /dev/tty50  /dev/tty60    /dev/ttyOpenCR
/dev/tty11  /dev/tty21  /dev/tty31  /dev/tty41  /dev/tty51  /dev/tty61    /dev/ttyprintk
/dev/tty12  /dev/tty22  /dev/tty32  /dev/tty42  /dev/tty52  /dev/tty62    /dev/ttyS0
/dev/tty13  /dev/tty23  /dev/tty33  /dev/tty43  /dev/tty53  /dev/tty63    /dev/ttyUSB0
/dev/tty14  /dev/tty24  /dev/tty34  /dev/tty44  /dev/tty54  /dev/tty7     
/dev/tty15  /dev/tty25  /dev/tty35  /dev/tty45  /dev/tty55  /dev/tty8
/dev/tty16  /dev/tty26  /dev/tty36  /dev/tty46  /dev/tty56  /dev/tty9
/dev/tty17  /dev/tty27  /dev/tty37  /dev/tty47  /dev/tty57  /dev/ttyACM0
```

새로 만들어진 심볼릭 링크를 통해 장치와 시리얼 통신을 테스트 해보았다.

```bash
$ tio /dev/ttyLIFT
[tio 20:48:05] tio v1.32
[tio 20:48:05] Press ctrl-t q to quit
[tio 20:48:05] Connected
up
lift  up  complete!
down
lift down complete!

[tio 20:49:29] Disconnected
```











[튜토리얼 목록](../../README.md) 

