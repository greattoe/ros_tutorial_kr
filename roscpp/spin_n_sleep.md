

## ros::spin() 과 rate.sleep() 의 차이



---

### ros::spin()

메세지 구독 또는 서비스 처리는 해당 이벤트( 메세지 구독, 서비스 처리 )에 대한 `callback` 함수를 호출에 의해 수행된다.  `ros::spin()` 또는 `ros::spinOnce()` 는 이러한 `callback` 처리에 필요한 `roscpp`  함수이다. 

코드 끝에서  `ros::spin()`  또는 `ros::spinOnce()` 이 호출되면 `Ctrl-C` 입력이 발생할 때 까지 프로그램이 종료되지 않도록 대기시키면서 `subscribe` 할 새 토픽이  `publish` 되거나 처리할  `service`  요청이 발생할 경우 해당  `callback` 함수를 호출한다. 

따라서 다음에 설명하고 있는 Rate, Duration 하고는 전혀 무관한 개념이다.

하지만 간 혹 작성한 노드가 `Ctrl-C` 입력에 응답하지 않는 경우를 방지하는 차원에서 `callback` 처리가 필요 없음에도 코드 마지막에  `ros::spin()`  또는 `ros::spinOnce()` 를 호출하는 경우에는 하는 `callback`  함수를 호출해야 하는 이벤트가 발생할 때까지 특별히 처리하는 작업없이 시간을 보낸다는 점에서만 조금 비슷하다.



---

### ros::Rate::sleep() ###

다음 코드에서 Rate 객체 rate는 루프를 초당 24회 반복시키기 위해 선언했다. 루프 1회 수행에 소요되는 시간은 그 내용에 따라 다르지만 루프 마지막의 rate.sleep( ) 에서 1/24초에서 모자란 시간만큼 sleep 한다. ( [ros::Rate::sleep( )](http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Rate.html#ae5664d27cda1b17a103347560259e945) 참조 )

```c++
ros::Rate rate(24.); // 24Hz
while(ros::ok())
{
       .
       .
       .
    rate.sleep();
}
```

같은 효과를 같는 다른 방법으로는 [ros::Duration::sleep( )](http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Duration.html#a39708cc9b2871f6b3715023ab9610043) 이 있다.

### ros::Duration::sleep()

이 코드의 Duartion 객체 duration 또한 1/24초 동안 루프를 1회 반복시키기 위해 선언 했다는 점에서는 위 코드의 Rate 객체 rate 와 다를 것이 없다. 다만 rate는 매개변수로 '1초에 몇 번 반복할 것인가?' 라는 주파수 관점의 Hz를 사용하고, duration은 '1회 반복에 얼만큼의 시간이 소요되는가?' 라는 주기를 관점으로 초(sec)를 사용한다는 것이 다를 뿐이다.

```c++
ros::Duration duration(1./24.); // 1/24초마다 한 번 ( once on every 1/24 sec )
while(ros::ok())
{
          .
          .
          .
    duration.sleep();
}
```



---

