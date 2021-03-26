---

## 웹페이지에 드론 GPS 좌표 출력 

**튜토리얼 레벨 :**  Intermediate(중급)(수정)

**이 튜토리얼 작성 환경 :**  catkin **/** Ubuntu 16.04 **/** Kinetic

**다음 튜토리얼 :** [링크 수정 필요]()

**이전 튜토리얼 :** [링크 수정 필요]()

**목록보기:** [README.md](../README.md)

---

실시간으로 Parrot Bebop2 드론을 지도에 표시해보자. 파이썬 라이브러리 중 Selenium 라이브러리는 파이썬 코드와 웹서버 사이에 데이터를 전달할 수 있다. 

구현에는 Ubuntu 16.04, ROS Kinetic Kame, Python, Selenium, Node.js, Kakao Map API 를 사용하으며, 먼저 파이썬( rospy )으로 Bebop2 GPS 데이터를 구독하는 Subscriber 를 구현하고, Subscribe 한 Bebop2 드론의 GPS 정보를 Selenium 을 통해 웹서버로 전달하였다. 전달된 GPS 정보는 Kakao Map API를 통해 지도화면에 표시하도록 하였다.

---

### 1. node.js 를 이용한 웹서버

---

#### 1.1 node.js 설치

**node.js** 10.x 버전 설치를 위한 **repository** 등록 및 반영 

```
$ curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
```

( `curl` 또는 `build-essential` 에 관련된 의존성 문제 발생할 경우에는 `sudo apt-get install curl build-essential` 명령으로 의존성 설치 후 진행. )

**node.js** 10.x 버전 설치

```
$ sudo apt-get install -y nodejs
```

설치된 **node.js** 및 **npm** ( Node Package Manager ) 버전 확인. 

```
$ node -v
v10.19.0
$ npm -v
6.13.7
```

#### 1.2 express.js 설치

다음과 같이 **express** 와 **express-generator** 를 설치한다.

```
$ sudo npm install -g express
$ sudo npm install -g express-generator
```

#### 1.3 간단한 웹 서버 구현

* express 를 이용하여 웹 어플리케이션 ex_selenium 생성

  ```
  $ express ex_selenium
  ```

* 생성된 ex_selenium 어플리케이션의 기본 의존성 설치

  ```
  $ cd ex_selenium
  $ npm install
  ```

* `~/ex_selenium/public/index.html` 을 시작 페이지로 하는 웹 서버 코드 편집

  ```
  $ gedit ~/ex_selenium/app.js
  ```

  자동으로 생성되어 있는 `app.js` 의 내용을 다음코드로 변경한다.

  ```javascript
  var express = require('express');
  var app = express();
  
  app.use(express.static(__dirname + '/public'));
  
  const port = 8080;
  
  app.listen(port, function(){
      console.log('listening on *:' + port);
  });
  ```

* `~/ex_selenium/public/index.html` 작성

  ```
  $ gedit ~/ex_selenium/public/index.html
  ```

  index.html

  ```html
  <html>
      <head>
          <title> selenium example </title>
      </head>
      <body>
          <h1>
              selenium sample page
          </h1>
          <hr>
          <p>
              Lattitude:
              <input id="longi" type="text"> &nbsp;
              Longitude:
              <input id="latti" type="text">
          </p>
          <scripts>
              function update_gps(lat, lon)
              {
                  var lattitude, longitude;
                  
                  if(lat) document.getElementById('latti').value = lat;
                  else; // if(lat == undefined) do nothing
                  
                  if(lon) document.getElementById('longi').value = lon;
                  else; // if(lon == undefined) do nothing
              }
              
              let timerId = setInterval(update_gps, 500);
          </scripts>
      </body>
  </html>
  ```



---

### 2. Selenium 설치

---

#### 2.1 웹드라이버 설치

* **타겟 브라우저의 버전 확인**

**Selenium**은 웹브라우저를 제어할 수 있는 프레임워크( Framework )로 제어하고자하는 브라우저의 웹드라이버( web-driver )를 통해 제어를 수행한다. 이 웹드라이버는 브라우저의 종류와 버전에 따라  다르며, Selenium을 통해 제어하려는 대상 브라우저와 그 버전에 맞는 드라이버를 설치해야 한다.

**Chrome** 브라우저의 경우를 예로 들자면 사용중인 Chrome 브라우저를 실행 후 '<img src="../img/chrome_seeting_menu.gif" width="28" />'버튼을 클릭하여 열린 메뉴에서 **'Help' - 'About Google Chrome'** 메뉴를 선택한다.

![](../img/chk_chrome_ver.png)

열려진 **'Settings - About Chrome'** 탭에서 버전을 확인한다.

![](../img/chk_chrome_ver2.png)

크롬 브라우저의 버전을 알아냈으면 아래 링크에서 해당 버전의 드라이버를 다운받는다.
Chrome 웹드라이버 다운로드: https://sites.google.com/a/chromium.org/chromedriver/downloads

![](../img/chrome_drv_down.png)

크롬 브라우저의 버전과 시작번호가 같은 드라이버를 다운로드한다. 



#### 2.2 Selenium 설치

```
$ pip install selenium
```









### 3. Bebop2 GPS 좌표 Subscriber

#### 3.1 GPS 좌표 Subscriber



### 





[튜토리얼 목록 열기](../README.md)


