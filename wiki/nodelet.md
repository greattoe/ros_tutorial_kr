## nodelet

<http://wiki.ros.org/nodelet>



---

### 1. High Level

---

`nodelet` 은 프로세스 내부에서 메시지 전달 시 copy cost의 발생없이[^1] 단일 프로세스 내에서 하나의 머신에서 여러 알고리즘을 실행하는 방법을 제공하기 위해 설계되었다.  `roscpp` 는 같은 노드 내의 publisher 와 subscriber 호출 시 포인터 복사 없이 수행하도록 최적화 되어 왔다. 이를 위해 `nodelet` 은 동일 노드로의 동적 클라스 로딩을 허용하지만, 같은 프로세스 내에서 하나의 같은 `nodelet` 이 별개의 서로 다른 `nodelet` 처럼 동작하도록 간단한 별도의 네임스페이스를 제공한다. 이로 인해 플러그인 라이브러리( [pluginlib](http://wiki.ros.org/pluginlib) )를 사용하는 런타임( runtime )에서 동적으로 `nodelet` 로딩이 가능하도록 그 기능이 확대되었다.

#### 1.1 Application( 응용분야 )

- 처리량의 급증으로 인한 데이터 트래픽 문제는 많은 숫자의 `nodelet` 을 구성 후, 동일 프로세스에 로드하여 copying 및 network 트래픽을 피할 수 있다.

#### 1.2  Design Goals( 설계 목표 )

- 이미 만들어져 사용되고 있는 C++ ROS 인터페이스 사용
- `nodelet` 들 사이에 <u>zero copy</u>[^1]를 이용한 데이터 교환 허용
- 시간 의존성을 극복하기위한 플러그인으로서의 동적 로딩
- 성능 향상을 위한 경우를 제외한 위치 투명성
- node와 `nodelet` 사이의 최소한의 코드 상이성

#### 1.3 Technique( 적용기술 )

- 동적 로딩에 사용될 기본 클래스 `nodelet :: Nodelet` 을 정의하라. 모든 `nodelet` 은 이 기본 클래스에서 상속되며 [pluginlib](http://wiki.ros.org/pluginlib)를 사용하여 동적으로 로드될 수 있다.
- 마치 첫번째 클라스의 node처럼 동작할 수 있도록 네임스페이스를 제공하여 자동으로 arguments 와 parameters를 remapping 한다.
-  하나 이상의 `nodelet` 을 로드할 수 있는 `nodelet_manager` 프로세스가 존재한다. `nodelet` 사이의  통신은 <u>boost shared pointer</u>[^2]와 함께 <u>zero copy</u>[^1] 를 이용한 `roscpp` 의 publish 호출을 사용할 수 있다.

#### 1.4 Basic Usage( 기본 사용법 )

```
nodelet usage:
nodelet load pkg/Type manager - Launch a nodelet of type pkg/Type on manager manager
nodelet standalone pkg/Type   - Launch a nodelet of type pkg/Type in a standalone node
nodelet unload name manager   - Unload a nodelet a nodelet by name from manager
nodelet manager               - Launch a nodelet manager node
```

#### 1.5 API( Application Programming Interfaces )

##### 1.5.1 Nodelet Base Class

`nodelet::Nodelet`

Publish methods:

```c++
Nodelet()
void init (const std::string& name, const ros::M_string& remapping_args, const std::vector<std::string>& my_argv);
```

Protected members and methods for use in subclass:


```c++
std::string             getName()
ros::NodeHandle&        getNodeHandle()
ros::NodeHandle&        getPrivateNodeHandle()
ros::NodeHandle&        getMTNodeHandle()
ros::NodeHandle&        getMTPrivateNodeHandle()
ros::CallbackQueue&     getMTCallbackQueue()
std::vector<std::string> getMyArgv()
```

Initialization method used to start ROS API in subclass:


```c++
virtual void onInit () = 0
```









---

[^1]:**zero copy ** **:** TCP/IP 환경에서 클라이언트의 정적파일 요청에 대응하기 위해 서버 어플리케이션의 사용자 영역과 커널 영역 사이에서 **1.** 디스크로 부터 파일데이터 요청 , **2.** 복사된 파일 데이터 반환, **3.** 반환받은 데이터를 소켓에 넣기 요청, **4.** 데이터를 소켓에 복사 등과 같은 과정이 처리된 이 후 비로소 서버가 클라이언트의 요청에 응답하게된다. **zero copy**는 커널영역에서 파일을 읽어서 바로 소켓에 넣어 처리함으로써 불필요한 반복된 복사과정을 생략한 방법이다.
[^2]: **boost shared pointer** **:** 스마트 포인터의 일종으로 복사 동작이 일어날 때는 참조 카운트를 늘리고, 소멸이 일어날 때는 참조 카운트를 줄임으로서 자신이 가리키고 있는 객체에 대한 참조 카운트를 유지한다. ( 이 때 참조 카운트가 0이 되면 가리키고 있는 객체를 소멸시킨다. )

