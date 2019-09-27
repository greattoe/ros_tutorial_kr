## tf/ Tutorials/ tf and Time (Python)



------

## tf and Time (Python)

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**이전 튜토리얼 :** [adding frame](./tf_3_adding_frame.md)

**다음 튜토리얼 :** [tf time_travel](./tf_5_time _travel.md)

**튜토리얼 목록 :** [README.md](../README.md)

**튜토리얼 원문 :** <http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29>

------



### 1. tf and Time

이전 튜토리얼들에서 어떻게 tf 가 좌표 프레임 tree를 쫓아가는지 배웠다. 이 tree는 시간에 따라 변하며, tf 는 매 transform 마다 time snapshot을 저장( 최대 10초까지가 기본값 )한다. 

지금까지는  `lookupTransform()` 함수를 그 tf tree 의 접근 가능한 최근 변환( 그 변환이 녹화된 정확한 시간을 모르는 채로 )에 접근하는 용도로만 사용했다. 이 튜토리얼은 어떻게 변환이 이루어진 시간을 특정하는지를 학습시켜 준다.

`~/catkin_ws/src/learning_tf/nodes/turtle_tf_listener.py` 를 아래 코드와 같이  `lookupTransform()` 함수를 호출하도록 편집, 수정한다.

```python
        try:
             now = rospy.Time.now()
            (trans,rot) = listener.lookupTransform("/turtle2", "/carrot1", now)
        except (tf.LookupException, tf.ConnectivityException):
```

갑자기 모든  `lookupTransform()` 함수가 실패하면서 다음과 같은 메세지가 나타난다.

```
Traceback (most recent call last):
  File "~/ros/pkgs/wg-ros-pkg-trunk/sandbox/learning_tf/nodes/turtle_tf_listener.py", line 25, in <module>
    (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', now)
tf.ExtrapolationException: Extrapolation Too Far in the future: target_time is 1253830476.460, but the closest tf  data is at 1253830476.435 which is 0.024 seconds away.Extrapolation Too Far in the future: target_time is 1253830476.460, but the closest tf  data is at 1253830476.459 which is 0.001 seconds away.Extrapolation Too Far from single value: target_time is 1253830476.460, but the closest tf  data is at 1253830476.459 which is 0.001 seconds away. See http://pr.willowgarage.com/pr-docs/ros-packages/tf/html/faq.html for more info. When trying to transform between /carrot1 and /turtle2. See http://www.ros.org/wiki/tf#Frequently_Asked_Questions
```

또는 `electric` 을 사용 중이라면 아래와 같은 메세지일 수도 있다.

```
Traceback (most recent call last):
  File "/home/rosGreat/ROS_tutorial/learning_tf/nodes/turtle_tf_listener.py", line 28, in <module>
    (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', now)
tf.ExtrapolationException: Lookup would require extrapolation into the future.  Requested time 1319591145.491288900 but the latest data is at time 1319591145.490932941, when looking up transform from frame [/carrot1] to frame [/turtle2]
```

왜 이런 일이 발생할까? 각각의 tf 리스너들은 서로 다른 tf 브로드캐스터들로 부터 오는 좌표 transform을 임시로 넣어둘 버퍼를 가지고 있다. tf 브로드캐스터가 transform을 전송할 때, 그 transform이 버퍼에 들어갈 때까지 보통 수ms 정도의 시간이 소요된다. 그래서 프레임 transform 호출을 "now"라는 시간으로 할 경우 몇 ms를 기다려야만 한다.



### 2. Wait for transforms

tf 는 transform이 사용가능할 때까지 기다리는 매우 훌륭한 도구를 제공한다.  다음 코드가 어떤 코드와 비슷한 지 한 번 살펴보자.

```python
    listener.waitForTransform("/turtle2", "/carrot1", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/turtle2", "/carrot1", now)
```

`waitForTransform()` 함수는 4개의 인수(argument)가 필요하다

transform 전송 대기를 

1. 어떤 프레임부터...
2. ... 어떤 프레임까지 하는가 , 그리고
3. 이 번에, 와
4. 타임아웃( 최대 대기 시간 ) 들이다.

그러므로 `waitForTransform()` 함수는 두 거북이가 전송가능한 상태가 될 때까지( 수 ms에 불과한 시간이지만 ) 실제로 block당하게 될 것이다.  또는 타임아웃 시간이 경과할 때까지 transform이 사용가능 상태로 바꾸지 않을 것이다.

그럼 왜 2개의 `waitForTransform()` 함수를 호출했을까? 코드 시작 부분에서 turtle2를 소환했지만 transform을 전송을 위한 대기시간을 주지않으면 /turtle2 프레임은 나타나지 않을 것이다. 최초의 `waitForTransform()` 함수는 /turtle2 프레임이 tf 상에 브로드캐스트되기 전까지는 `waitForTransform()` 함수를 "time now"라는 시간으로 호출하면서 기다리게 될 것이다.



### 3. 결과 확인

이제 다시 한 번 첫 번 째 거북이를 조종해보면, 두 번 째 거북이가 따라오는 것을 볼 수 있을 것이다.

이제 거북이의 움직임에 눈에 띄는 차이는 없다는 것을 알았을 것이다. 실제 발생한 시간 차이가 겨우 수 ms에 지나지 않기 때문이다. 그렇다면 이 튜토리얼에서 무엇 때문에 코드를 `Time(0)` 에서 `now` 로 바꾸게 한 것인가? 바로 tf buffer 와 time delay 가 서로 관련 있다는 것을 알기 위해서다. 실제로 tf 를 사용하는 경우, 대부분 `Time(0)` 를 사용해도 완벽하게 동작한다.

이제 다음 튜토리얼인 'time travel in tf' 로 넘어가자.



[튜토리얼 목록 열기](../README.md)



[다음 튜토리얼](./tf_5_time _travel.md)

[이전 튜토리얼](./tf_3_adding_frame.md)

