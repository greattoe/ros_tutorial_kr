## image_transport/ Tutorials/ PublishingImages



**원문 : **http://wiki.ros.org/image_transport/Tutorials/PublishingImages



---



이 후의 두 튜토리얼에서 [tf introduction](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) 튜토리얼의 데모와 같이 동작하는 코드를 작성할 것이다. 그리고 나서 tf 의 고급 기능을 이용한 데모 코드의 기능 확장에 집중할 것이다.

시작하기 전에 이 프로젝트를 위한  tf, [roscpp](http://wiki.ros.org/roscpp), [rospy](http://wiki.ros.org/rospy) and [turtlesim](./turtlesim.md)에 대한 의존성을 갖는 learning_tf 라는 새로운 ROS 패키지를 만든다.

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
```

아무 내용도 작성하지 않은 빈 패키지만 roscd 명령으로 이동할 수 있도록 빌드해두자.

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash
```



### 1. 간단한 Image Publisher 작성 

이미지를 계속해서 발행하는 `publisher` 노드를 만들어보자

```bash
$ cd ~/catkin_ws

```



#### 1.1 코드







```c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
```



#### 1.2 





```c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "video_to_msg");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) return 1;

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
    
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
```

우선 앞 서 진행했던 튜토리얼의 launch 파일이 실행 중이라면 ctrl-c 를 입력하여 그것을 먼저 종료한다. 이제 직접 작성한 거북이 broadcaster를 시작할 준비가 완료되었다.

```bash
$ roslaunch learning_tf start_demo.launch
```

turtlesim 노드가 실행되고 거북이가 한 마리 보일 것이다.



### 3. 결과 확인

**tf_echo** 툴을 이용하여 실제로 거북이의 pose 가 tf 로 publish 되고 있는가 체크한다.

```bash
$ rosrun tf tf_echo /world /turtle1
```

위의 명령은 첫 번 째 거북이의 pose 값을 보여준다. 키보드를 이용하여 거북이를 이리저리 움직여 보라. 만일 같은 명령을 `/world` 와 `/turtle2` 에 대해 수행한다면 하나의 transform 도 보이지 않을 것이다. 왜냐하면 두 번 째 거북이는 아직 거기 없기 때문이다. 하지만 곧 다음 튜토리얼에서 두 번 째 거북이를 추가할 것이고, 두 번 째 거북이의 pose 값도 tf 로 broadcast 될 것이다.

[튜토리얼 목록 열기](../README.md)

[다음 튜토리얼](./tf_3_listener.md)