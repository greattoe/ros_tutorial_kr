## Callbacks and Spinning

<http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning>

```roscpp``` 는 당신의 어플리케이션에 스레딩 모델을 지정하려 하지 않는다. 이는  ```roscpp``` 가 네트워크 관리, 스케줄링 등을 수행하기 위해 백그라운드에서 스레드를 사용할 수 있지만, 그 스레드를 어플리케이션에 절대 노출시키지 않는다는 것을 뜻한다. 그렇지만 ```roscpp``` 는 당신이 원하는 숫자의 스레드에서 당신의 ```Callback``` 이 호출되는 것을 허용한다.

그 결과 적절한 사용자 작업이 없이는 메세지 구독, 서비스, 다른 어떤 ```Callback`` 도 호출될 수 없을 것이다. 

**Note:** ```Callback queue``` 또는 ```Callback spinning``` 은 보통 ```roscpp``` 의 내부 네트워크 통신에는 영향을 미치지 않는다.  사용자 ```Callback```이 발생할 때에만 영향을 미친다. ```Callback``` 처리 속도와 메시지 도착 속도에 따라 메시지 삭제 여부가 결정되므로 구독 대기열에 영향을 미치게 된다.



### 1. Single-threded Spinning

가장 단순하면서도 가장 일반적인 ```Single-threaded Spinning```은 ```ros::spin()``` 이다. :

```C++
   1 ros::init(argc, argv, "my_node");
   2 ros::NodeHandle nh;
   3 ros::Subscriber sub = nh.subscribe(...);
   4 ...
   5 ros::spin();
```

이 애플리케이션에서 모든 사용자 콜백은 ```ros::spin()``` 호출 내에서 호출된다.  ```ros::spin()``` 은 ```ros::shutdown()``` 을 호출하거나 ```Ctrl-C``` 입력에 의해 노드가 종료 될 때까지 반환되지 않습니다.

흔히 사용되는 또 다른 방법은 주기적으로 ```ros::spinOnce()```를 호출하는 것이다. :

```C++
   1 ros::Rate r(10); // 10 hz
   2 while (should_continue)
   3 {
   4   ... do some work, publish some messages, etc. ...
   5   ros::spinOnce();
   6   r.sleep();
   7 }
```

```ros::spinOnce()``` 는 자신이 호출되는 그 시점에서 대기중인 모든 ```Callback``` 들을 호출하게 된다.

```spin()``` 을 사용하여 코드를 구현하는 것은 매우 간단하다. :

```c++
   1 #include <ros/callback_queue.h>
   2 ros::NodeHandle n;
   3 while (ros::ok())
   4 {
   5   ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
   6 }
```

그리고 spinOnce( )의 경우 간단히 다음처럼 사용한다. :

```c++
   1 #include <ros/callback_queue.h>
   2 
   3 ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
```

**Note:** ```spin()``` 과  ```spinOnce()``` 는 실제로 단일 스레드 어플리케이션이라는 것을 의미하며, 여러 스레드에서 동시에 호출되는 상황을 위해 최적화 되어 있지 않다. 다중 스레드 스피닝에 대한 사항은 다음의 다중 스레드 스피닝을 참조하시오.



### 2. Multi-threaded Spinning

roscpp는 다중 스레드로부터의 Callback 호출을 지원하는 기능을 제공한다. 두 가지 방법이 제공되는데 그 하나는 ros::MultiThreadedSpinner 이고 다른 하나는 ros::AsyncSpinner 이다.

[**ros::MultiThreadedSpinner**](http://docs.ros.org/diamondback/api/roscpp/html/classros_1_1MultiThreadedSpinner.html) :

- ```MultiThreadedSpinner``` 는 ```ros::spin()``` 과 유사한 일종의 ```blocking spinner``` 이다. 생성자에서 코어당 스레드 수를 지정할 수 있는데, 0으로 지정하거나, 숫자를 지정하지 않을 경우 CPU 코어당 1개의 스레드가 사용된다.

  ```c++
     1 ros::MultiThreadedSpinner spinner(4); // Use 4 threads
     2 spinner.spin(); // spin() will not return until the node has been shutdown
     3 
  ```

[**ros::AsyncSpinner**](http://docs.ros.org/diamondback/api/roscpp/html/classros_1_1AsyncSpinner.html) :

- 보다 유용한 스레드 스피너는 ```AsyncSpinner``` 이다. ```spin()``` 호출을 ```blocking``` 하는 대신, ```AsyncSpinner``` 는 ```start()``` 와 ```stop()``` 을 호출할 수 있으며, 자신이 소멸될 때에는 자동으로 ```stop()``` 이 호출된다.  다음 코드는 위 `MultiThreadedSpinner` 예제와 같이 동작하는 ```AsyncSpinner``` 예제이다. :

  ```c++
   1 ros::AsyncSpinner spinner(4); // Use 4 threads
     2 spinner.start();
     3 ros::waitForShutdown();
  ```
  



### 3. CallbackQueue::callAvailable() and callOne()

