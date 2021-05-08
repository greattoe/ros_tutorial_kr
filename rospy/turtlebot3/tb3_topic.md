## Turtlebot3/ Topics



---

## Turtlebot3 발행 토픽

**이 문서의 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**관련 자료 :** 

**REPOSITORY :** 

**문서 목록 :** [README.md](../README.md)

---



### /tf

```
---
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1569341995
        nsecs: 118904935
      frame_id: "odom"
    child_frame_id: "base_footprint"
    transform: 
      translation: 
        x: 0.0918072909117
        y: 0.00205412087962
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0393077991903
        w: 0.999227166176
---
```



### /odom

```
---
header: 
  seq: 1296
  stamp: 
    secs: 1569340595
    nsecs: 252484956
  frame_id: "odom"
child_frame_id: "base_footprint"
pose: 
  pose: 
    position: 
      x: 0.091345705092
      y: 0.00217283004895
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0129650924355
      w: 0.999915957451
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.000215363950701
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```



### /scan

```
---
header: 
  seq: 3457
  stamp: 
    secs: 1569341237
    nsecs: 819157708
  frame_id: "base_scan"
angle_min: 0.0
angle_max: 6.26573181152
angle_increment: 0.0174532923847
time_increment: 2.98699997074e-05
scan_time: 0.0
range_min: 0.119999997318
range_max: 3.5
ranges: [2.3570001125335693, 2.36299991607666, 2.367000102996826, 2.381999969482422,  2.367000102996826, 2.4000000953674316, 2.3989999294281006, 2.4059998989105225, 1.690999984741211, 1.715999960899353, 1.6260000467300415, 1.5809999704360962, 1.4259999990463257, 1.3940000534057617, 1.3550000190734863, 1.3630000352859497, 1.2050000429153442, 1.2230000495910645, 1.152999997138977, 1.1260000467300415, 1.0770000219345093, 1.0190000534057617, 1.0360000133514404, 1.0099999904632568, 1.0230000019073486, 0.3779999911785126, 0.3779999911785126, 0.3720000088214874, 
...... 중략 ......., 0.5220000147819519, 0.5249999761581421, 0.5260000228881836, 0.527999997138977, 0.5299999713897705, 0.5329999923706055, 0.5360000133514404, 0.5379999876022339, 0.5410000085830688, 0.5440000295639038, 0.546999990940094, 0.5509999990463257, 0.5550000071525574, 0.5590000152587891, 0.5630000233650208, 0.5690000057220459, 0.5740000009536743, 0.5789999961853027, 0.5839999914169312, 0.5910000205039978, 0.597000002861023, 0.6039999723434448, 0.6100000143051147, 0.6159999966621399, 0.6209999918937683, 0.6240000128746033, 0.6330000162124634, 0.640999972820282, 0.6510000228881836]
intensities: [854.0, 776.0, 688.0, 637.0, 575.0, 547.0, 503.0, 497.0, 580.0, 364.0, 126.0, 91.0, 136.0, 109.0, 139.0, 172.0, 176.0, 183.0, 333.0, 360.0, 426.0, 434.0, 382.0, 445.0, 575.0, 640.0, 743.0, 817.0, 959.0, 1148.0, 1534.0, 4220.0, 3367.0, 3933.0, 3889.0, 4118.0, 4383.0, 4079.0, 3738.0, 3853.0, 3396.0, 3874.0, 4184.0, 4001.0, 3632.0, 3505.0, 3521.0, 3360.0, 3801.0, 3074.0, 3515.0, 3304.0, 3163.0, 3147.0, 2023.0, 3124.0, 3307.0, 3853.0, .... 중략 ..., 247.0, 492.0, 296.0, 340.0, 476.0, 238.0, 329.0, 328.0, 346.0, 359.0, 340.0, 365.0, 378.0, 382.0, 259.0, 117.0, 173.0, 218.0, 254.0, 258.0, 305.0, 216.0, 188.0, 201.0, 214.0, 125.0, 429.0, 479.0, 267.0, 604.0, 347.0, 289.0, 283.0, 155.0, 318.0, 334.0, 584.0, 330.0, 374.0, 323.0, 192.0, 237.0, 262.0, 202.0, 250.0, 355.0, 388.0, 384.0, 381.0, 3270.0, 1016.0]
```



### /imu

```
---
header: 
  seq: 253877
  stamp: 
    secs: 1569342277
    nsecs: 897385944
  frame_id: "imu_link"
orientation: 
  x: 0.00422589248046
  y: -0.00159128580708
  z: 0.244152396917
  w: 0.969721853733
orientation_covariance: [0.0024999999441206455, 0.0, 0.0, 0.0, 0.0024999999441206455, 0.0, 0.0, 0.0, 0.0024999999441206455]
angular_velocity: 
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.019999999552965164, 0.0, 0.0, 0.0, 0.019999999552965164, 0.0, 0.0, 0.0, 0.019999999552965164]
linear_acceleration: 
  x: 0.0808043032885
  y: 0.0646434426308
  z: 10.2938699722
linear_acceleration_covariance: [0.03999999910593033, 0.0, 0.0, 0.0, 0.03999999910593033, 0.0, 0.0, 0.0, 0.03999999910593033]
---
```







[튜토리얼 목록 열기](../README.md)

[이 링크도 수정]()






