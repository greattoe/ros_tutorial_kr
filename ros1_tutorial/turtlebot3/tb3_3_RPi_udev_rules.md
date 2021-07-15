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













[튜토리얼 목록](../../README.md) 

