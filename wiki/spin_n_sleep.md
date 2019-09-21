## ros::spin() 과 rate.sleep() 의 차이



### ros::spin()

메세지 구독 또는 서비스 처리는 해당 이벤트( 메세지 구독, 서비스 처리 )에 대한 `Callback` 함수 호출에 의해 수행된다.  `ros::spin()`  또는 `ros::spinOnce()` 는 `Callback`을 처리할 수 있도록 해준다. 



### ros::Rate::sleep() ###

다음 코드에서 `Rate` 객체 `rate` 는 루프를 초당 24회 반복시키기 위해 선언했다. 루프 1회 수행에 소요되는 시간은 그 내용에 따라 다르지만 루프 마지막의 `rate.sleep()` 에서 1/24초에서 모자란 시간만큼 sleep 한다. ( [ros::Rate::sleep( )](http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Rate.html#ae5664d27cda1b17a103347560259e945) 참조 )

```c++
ros::Rate rate(24.);
while(ros::ok())
{
    rate.sleep();
}
```

같은 효과를 같는 다른 방법으로는 [ros::Duration::sleep( )](http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Duration.html#a39708cc9b2871f6b3715023ab9610043) 이 있다.



### ros::Duration::sleep()

다음은 위에 설명한 `ros::Rate::sleep()` 과 같은 효과를 갖는 `ros::Duration::sleep()` 예제 코드이다.  `ros::Rate::sleep()` 이 주파수( 1초당 반복횟수 Hz ) 측면의 접근이라면,   `ros::Duration::sleep()` 은 주기( 1회 소요시간 ) 측면의 접근이라는 것만 다르고, 효과는 같다. 

```c++
ros::Duration duration(1./24.);
while(ros::ok())
{
    duration.sleep();
}
```

