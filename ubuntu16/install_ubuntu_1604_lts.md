

### Ubuntu 16.04 LTS 설치

이 후 내용은 Microsoft Windows 10 이 설치되어있는 Intel x64 또는 AMD64 시스템을 기준으로 작성되었습니다.



#### 1. 설치전 준비

**설치 ISO 파일 다운로드 :** https://releases.ubuntu.com/16.04/ 

<img src="..\img\ubuntu16\_download_iso.png" width="100%">



**설치디스크 작성**

4GB 이상의 빈 USB 메모리를 준비한다. 다운받은 ISO 이미지는 리눅스 형식으로 기록된 디스크의 이미지이다.  따라서 이를 USB 메모리에 기록할 도구가 필요하다.  설치 디스크를 작성할 수 있는 프로그램은 여러가지(Ultra ISO, Universal USB Installer, UNetbooin, Rufus 등)가 있다.  다음은 그 중 Rufus를 이용하는 방법이다.

Rufus 다운로드 : https://rufus.ie/ 

<img src="..\img\ubuntu16\_download_rufus.png" width="100%">





<img src="..\img\ubuntu16\_rufus01.png">



**설치 공간 확보**

키보드에서 '<img src="..\img\ubuntu16\__win_key.png" width="20">' + ' **X** ' 키를 누르면 나타나는 메뉴에서 ' **K** '를 누르면 **'디스크 관리자'** 가  실행된다.

<img src="..\img\ubuntu16\disk_mgmt_01.png" width="50%"><img src="..\img\ubuntu16\disk_mgmt_02.png" width="50%">





<img src="..\img\ubuntu16\disk_mgmt_04.png" width="50%"><img src="..\img\ubuntu16\system_properties.png" width="50%">



#### 2. 설치

**'한국어'** 선택 후 **'Ubuntu 설치'** 클릭

네크워크 연결이나 업데이트 등은 설치 후에 진행하기로하고, **'계속'** 을 클릭한다.

<img src="..\img\ubuntu16\01_welcome.png" width="50%"><img src="..\img\ubuntu16\02_during_setup.png" width="50%">



윈도우에서 볼륨을 축소하여 확보한 할당되지 않은 공간에 설치할 것이므로 **'기타'** 를 선택하고 **'계속'** 을 클릭한다.

그림보다는 복잡한 상황이겠지만 <u>윈도우에서 확보한 크기</u>만큼의 **'남은 공간'** 을 확인, 선택하고 **'+'** 를 클릭한다.  

<img src="..\img\ubuntu16\03_setup_where.png" width="50%"><img src="..\img\ubuntu16\04_add_partition.png" width="50%">



**'크기'** 에 PC 의 메모리(RAM) 크기(4G: 4096, 8G: 8192, 16G: 16384)를 기입 후 용도에서 **'스왑 영역'** 을 선택한다. 

<img src="..\img\ubuntu16\05_swap_partition.png" width="100%">



다시 **'남은 공간'** 을 선택하고, **'+'** 를 클릭한 후,  **'마운트 위치'** 에서 **'/'** 를 선택한다.  

<img src="..\img\ubuntu16\06_root_partition.png" width="100%">



**'부트로더를 설치할 장치'** 가 윈도우가 설치된 디스크가 맞는지 확인 후, **'지금 설치'** 를 클릭한다.  

<img src="..\img\ubuntu16\07_partition_result.png" width="100%">



지금 까지의 파티션 테이블 작업을 최종 확인하는 메세지 상자에서 **'계속'** 을 클릭한다.

<img src="..\img\ubuntu16\08_apply_partition.png" width="100%">



**시간대역**(Time Zone)을 설정하는 화면에서 **'Seoul'** 을 선택 후, **'계속'** 을 클릭한다.  

<img src="..\img\ubuntu16\09_where_live.png" width="100%">



**키보드 레이아웃** 을 설정하는 화면에서 **'한국어'** , **'한국어-한국어(101/104키 호환)'** 을 선택 후, **'계속'** 을 클릭한다. 

<img src="..\img\ubuntu16\10_kb_layout.png" width="100%">



**계정**(Account)를 설정하는 과정이다. 

**이름**(Name)은 크게 중요하지 않다.  로그인 화면에 표시되는 이름이다. 심지어 비워 두어도 된다.  

**컴퓨터 이름**(Host Name)은 네트워크에 연결된 단말명으로 표시되는 이름이다. 영문 대소문자, 숫자, 하이픈( - )만을 사용할 수 있으며, 숫자로 시작할 수 없다.  

**사용자 이름**(User Name)은 해당 계정을 대표하는 명칭으로 **사용자 ID** 에 해당한다.  역시 영문 대소문자, 숫자, 하이픈( - )만을 사용할 수 있으며, 숫자로 시작할 수 없다.  

**'암호'**(Password)는 숫자, 특수문자, 대, 소문자를 섞어 보안에 취약하지 않도록 정하는 것이 원칙이지만, 중요한 순간에 생각이 나지 않는다면 낭패이므로, 이를 염두에 두어 신중하게 정하도록하자.  ( **'암호 선택'** 입력시 옆에 나타나는 '짧은 암호', '약한 암호' 같은 메세지는 무시해도 상관없다. )

계정에 대한 설정을 마쳤으면 **'계속'** 을 클릭하여 설치를 진행한다.  

<img src="..\img\ubuntu16\11_whoru.png" width="100%">



이제부터 **'설치 완료'** 대화상자가 나타날 때까지 기다린다. 

<img src="..\img\ubuntu16\12_start_install.png" width="100%">



**'지금 다시 시작'** 을 클릭한다. 

<img src="..\img\ubuntu16\13_finish_install.png" width="100%">



#### 3. 설치 후 필요한 추가작업


