

## 6. GPS 위치정보에 의한 이동 

**튜토리얼 레벨 :**  Intermediate(중급)(수정)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** ROS Kinetic

**다음 튜토리얼 :** [링크 수정 필요]() 

**이전 튜토리얼 :** [링크 수정 필요]() 

**목록보기:** [README.md](../README.md) 



GPS 로부터 수신된 위도, 경도 정보를 바탕으로 드론이 최초 이륙한 GPS 위치정보를 이용한 RTB( Return to Base ) 기능 및, 목적지 GPS 좌표를 전달받아 해당 위치로 이동하는 기능을 구현한다. 

---

**필요기능 정의**

- GPS Positiion 정보 구독
- 목표 GPS 좌표 방향을 향한 회전
- 목표 GPS 좌표까지의 이동
- 이동 중 위치확인 및 보정



### 1. 두 지점의 GPS 좌표로부터 두 지점 사이의 거리 및 방위각 계산 라이브러리

두 지점의 GPS 좌표로부터 두 지점 사이의 거리 및 방위각 계산하는 라이브러리 `GPS.py` 파일을 작성을 위해 작업 경로를`bb2_pkg/src/bb2_pkg` 로 변경한다.

```bash
$ roscd bb2_pkg/src/bb2_pkg
```

`GPS.py`  파일 생성.

```bash
$ touch GPS.py
```

`GPS.py`  파일 생성.

```bash
$ touch GPS.py
```

`GPS.py`  파일 편집.

```bash
$ gedit GPS.py &
```



```python
#!/usr/bin/env python
 
from math import sqrt, sin, cos, tan, acos, asin, atan
'''-------------------------------------------------------------------------------------

                     longitude 1 degree 
                   |<- 90075.833903 m -->|   (latitude)
                   |                     |
       ----------  +----------+----------+ 34.444029
         ^         |          |          |   input latitude  of Point1:  35.944029
         |         |          |          |   input longitude of Point1: 126.184297 --+
         |         |          |          |   input latitude  of Point2:  35.944029   |
                   |          |          |   input longitude of Point2: 127.184297 --+
      latitude     |          |          |  distance = 90075.293451, bearing = 89.706518
      1 degree     +----------+----------+ 35.444029
   111016.503262 m |          |          |   input latitude  of Point1:  35.444029 --+
                   |          |          |   input longitude of Point1: 126.684297   |
         |         |          |          |   input latitude  of Point2:  36.444029 --+
         |         |          |          |   input longitude of Point2: 126.684297
         v         |          |          |  distance = 111016.503262, bearing = 0.000018
       ----------  +----------+----------+ 36.444029

   (longuitude) 126.184297    126.684297    127.184297

   longitude 1.0000000 = 90075.833903(m)       latitude 1.000000000 = 111016.503262(m)
             0.1000000 =  9007.583390(m)                0.100000000 =  11101.650326(m)
             0.0100000 =   900.758339(m)                0.010000000 =   1110.165033(m)
             0.0010000 =    90.075834(m)                0.001000000 =    111.016503(m)
             0.0001000 =     9.007583(m)                0.000100000 =     11.101650(m)
             0.0000100 =     0.900758(m)                0.000010000 =     1.1101650(m)
             0.0000010 =     0.090076(m)                0.000001000 =     0.1110165(m)
             0.0000001 =     0.009008(m)                0.000000100 =     0.0111017(m)
               
--------------------------------------------------------------------------------------'''
# class for getting distance & azimuth between P1 & P2
class GPS:

    def __init__(self):
        self.c15 = 6378137.000000000
        self.c16 = 6356752.314140910
        self.c17 =       0.0033528107
      
    def get_distance(self, P1_latitude, P1_longitude, P2_latitude, P2_longitude):
        
        # convert from degree to radian
        e10 = P1_latitude  * 3.1415926535 / 180.
        e11 = P1_longitude * 3.1415926535 / 180.
        e12 = P2_latitude  * 3.1415926535 / 180.
        e13 = P2_longitude * 3.1415926535 / 180.
        
        # GRS80
        f15 = self.c17 + self.c17 * self.c17
        f16 = f15 / 2.
        f17 = self.c17 * self.c17 /  2.
        f18 = self.c17 * self.c17 /  8.
        f19 = self.c17 * self.c17 / 16.
        
        c18 = e13 - e11
        c20 = (1. - self.c17) * tan(e10)
        c21 = atan(c20)
        c22 = sin(c21)
        c23 = cos(c21)
        c24 = (1. - self.c17) * tan(e12)
        c25 = atan(c24)
        c26 = sin(c25)
        c27 = cos(c25)
        c29 = c18
        c31 = (c27 * sin(c29) * c27 * sin(c29)) + (c23 * c26 - c22 * c27 * cos(c29)) * (c23 * c26 - c22 * c27 * cos(c29))
        c33 = (c22 * c26) + (c23 * c27 * cos(c29))
        c35 = sqrt(c31) / c33
        c36 = atan(c35)
        c38 = 0.0
        c40 = 0.0
        c41 = cos(asin(c38)) * cos(asin(c38)) * (self.c15 * self.c15 - self.c16 * self.c16) / (self.c16 * self.c16)
        c43 = 1 + c41 / 16384 * (4096 + c41 * (-768 + c41 * (320 - 175 * c41)))
        c45 = c41 / 1024 * (256 + c41 * (-128 + c41 * (74 - 47 * c41)))
        c47 = c45 * sqrt(c31) * (c40 + c45 / 4 * (c33 * (-1 + 2 * c40 * c40) - c45 / 6 * c40 * (-3 + 4 * c31) * (-3 + 4 * c40 * c40)))
        c50 = self.c17 / 16 * cos(asin(c38)) * cos(asin(c38)) * (4 + self.c17 * (4 - 3 * cos(asin(c38)) * cos(asin(c38))))
        c52 = c18 + (1 - c50) * self.c17 * c38 * (acos(c33) + c50 * sin(acos(c33)) * (c40 + c50 * c33 * (-1 + 2 * c40 * c40)))
        c54 = self.c16 * c43 * (atan(c35) - c47)
    
        if ((P1_latitude == P2_latitude) and (P1_longitude == P2_longitude)):
            return 0
        
        if (c31 == 0):
            c38 = 0.0
            
        else:
            c38 = c23 * c27 * sin(c29) / sqrt(c31)
        
        if ((cos(asin(c38)) * cos(asin(c38))) == 0):
            c40 = 0.0
            
        else:
            c40 = c33 - 2 * c22 * c26 / (cos(asin(c38)) * cos(asin(c38)))
        
        return c54
        
  
    def get_bearing(self, P1_latitude, P1_longitude, P2_latitude, P2_longitude):
    
        Cur_Lat_radian  = P1_latitude  * (3.1415926535 / 180)
        Cur_Lon_radian  = P1_longitude * (3.1415926535 / 180)
        
        Dest_Lat_radian = P2_latitude  * (3.1415926535 / 180)
        Dest_Lon_radian = P2_longitude * (3.1415926535 / 180)
        
        # radian distance
        radian_distance = 0;
        radian_distance = acos(sin(Cur_Lat_radian) * sin(Dest_Lat_radian) + cos(Cur_Lat_radian) * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian))
        
        radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian) * cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)))
        true_bearing = 0
        
        if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0):
            true_bearing = radian_bearing * (180 / 3.1415926535)
            true_bearing = 360 - true_bearing
        else:
            true_bearing = radian_bearing * (180 / 3.1415926535)
            
        return true_bearing
```



[튜토리얼 목록 열기](../README.md)
