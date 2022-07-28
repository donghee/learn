# ROS 2 Programming Day 2

 - 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
 - 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
 - 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

-   목표: 나만의 ROS 노드 만들고, 시뮬레이터를 이용하여 노드를 프로그래밍 할 수 있다.
-   교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day2](https://learn.dronemap.io/ros-workshop/ros2/#/day2)
-   코치: 박동희 dongheepark@gmail.com

1. ROS 기본 프로그래밍
 - 워크스페이스 만들기
 - ROS 노드 만들기

2. 자율 주행 소프트웨어 실습
 - 시뮬레이터 사용하기
 - SLAM 소개
 - ROS와 시뮬레이터를 이용하여 SLAM 실습

3. Raspberry Pi에 ROS 2 개발 환경 구성
 - 네트워크 설정
 - Raspberry Pi에 ROS 2 설치
 - 모터, IMU ROS 노드 만들기

---

## ROS 노드 만들기

### 새로운 노드 만들기

빌드 도구 colcon 설치하기
```
sudo apt install python3-colcon-common-extensions python3-rosdep2
```

#### 패키지 만들기
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

my_package 이름의 패키지 안에 my_node 이름 노드를 만들어 보자.
```
#ros2 pkg create --build-type ament_python my_package # my_package 만 만들경우
ros2 pkg create --build-type ament_python --node-name my_node my_package
```
 - python package(ament_python) ? cmake 패키지(ament_cmake) --dependencies 의존패키지
 - 의존 패키지 추가하기: ros2 pkg create --build-type ament_cmake --node-name my_node my_cpp_package --dependencies rclcpp

#### 패키지 빌드
```
cd ~/ros2_ws
colcon build --packages-select my_package
```

#### 패키지 노드 실행

환경 변수 설정
```
cd ~/ros2_ws
source install/setup.bash
```

노드 실행
```
ros2 run my_package my_node
```

#### 해보기: my_package 분석
 - ros2_ws/src/my_package 안의 디렉토리와 파일을 분석하고 설명해보자.

#### 해보기: c++ 노드 패키지 작성
 - c++ 코드로 구현된 my_cpp_package와 my_node 만들고, 그 노드를 실행해보자.
 - 힌트: 패키지 만들때 ament_cmake 옵션 사용
 - 참고: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

----

### 새로운 노드 만들기: Pub Sub 노드 만들기

#### 패키지 만들기
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

#### 노드 코드 작성

~/ros2_ws/src/py_pubsub/py_pubsub/minimal_publisher.py

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

~/ros2_ws/src/py_pubsub/py_pubsub/minimal_subscriber.py

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 패키지 노드 실행 위치지정

~/ros2_ws/src/py_pubsub/setup.py 의 entry_points에 다음줄 추가

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.minimal_publisher:main',
                'listener = py_pubsub.minimal_subscriber:main',
        ],
},
```
#### 패키지 빌드
```
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
```

```
colcon build --packages-select py_pubsub
```

```
source install/setup.bash
```

#### 패키지 노드 실행

```
ros2 run py_pubsub talker
```

```
ros2 run py_pubsub listener
```

#### 해보기: c++ 노드 작성
 - c++로 작성된 cpp_pubsub listener를 만들어서 py_pubsub talker가 보내는 토픽을 받아 보자.
 - ros2 run cpp_pubsub listener
 - 참고: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node
 - rqt를 이용하여 토픽을 모니터링 해보자.

![](https://i.imgur.com/ARCF7vx.png)

## SLAM

Simultaneous Localization and Mapping 동시적 위치추정 및 지도작성

자율주행 차량에 사용되어 주변 환경 지도를 작성하는 동시에 차량의 위치를 작성된 지도 안에서 추정하는 방법

미지의 장소의 지도 작성!

![](https://kr.mathworks.com/discovery/slam/_jcr_content/mainParsys3/discoverysubsection_158176500/mainParsys3/image.adapt.full.medium.png/1654866250910.png)

종류
 - vSLAM: 카메라 이용.
   - PTAM, ORB-SLAM 알고리즘 사용
 - 라이다 SLAM: 라이다 센서 이용. 포인트 클라우드 생성.
   - 포인트 클라우드 합치기 알고리즘: ICP, NDT

특징
 - 실시간 지도 생성
 - 누적 오차 발생
 - 높은 계산 비용.


## 시뮬레이션

Gazebo 시뮬레이터를 이용하여, SLAM과 자율 주행을 테스트 해보자.

<!-- https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros -->
<!-- https://ubuntu.com/blog/simulate-the-turtlebot3 -->

### 시뮬레이터 Gazebo

#### Gazebo 11 설치

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo11 libgazebo11-dev

gazebo
```

가상 머신에서 에러가 나는 경우 다음 실행후 다시 gazebo 실행
```
export SVGA_VGPU10=0
```

ROS Gazebo 패키지 설치

```
sudo apt install ros-foxy-gazebo-dev ros-foxy-gazebo-plugins ros-foxy-gazebo-msgs  ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros ros-foxy-ros-core ros-foxy-geometry2
```

SLAM, Navigation 패키지 설치
```
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

터틀봇 패키지 설치
```
sudo apt install python3-vcstool
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
```

turtlebot3.repos파일의 version을 ros2-devel을 foxy-devel로 수정. 단 ld08_driver는 ros2-devel로 남겨 둔다.

```
vcs import src < turtlebot3.repos
```

```
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

시뮬레이터 실행
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![](https://www.theconstructsim.com/wp-content/uploads/2022/01/Click-Open-Gazebo-to-view-the-Gazebo-simulation.png)

```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

![](https://www.theconstructsim.com/wp-content/uploads/2022/01/ros2-launch-turtlebot3_cartographer-cartographer.launch.py_.png)

키보드 입력
```
ros2 run turtlebot3_teleop teleop_keyboard
```

토픽, 서비스 확인
```
ros2 topic list
ros2 service list
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' -1
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}}' -1
ros2 run turtlebot3_teleop teleop_keyboard
```

맵과 로봇 프레임 좌표 연결
```
ros2 run tf2_ros tf2_echo base_footprint map
```

프레임 좌표 연결 확인
```
ros2 topic echo /odom
ros2 run tf2_tools view_frames.py
```

맵 저장
```
ros2 run nav2_map_server map_saver_cli -f map
```

네비게이션 실행. 자율 주행!
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=./map.yaml
```

![](https://i.imgur.com/8TLNJRc.png)


#### Gazebo 시뮬레이션 모델 만들기

1. 차동 주행 월드 로드

```
gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

![](https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive.png)

1. 월드 파일 읽어 보기

/opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

월드 파일에서 `gazebo_ros_pkgs` 확인

1. 로봇 조정
```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

![](https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive_lin_vel.gif)

<!-- --- -->
<!-- ## 가제보 시뮬레이션 로봇 만들기 -->

<!-- tank_gazebo -->

<!-- ros2 pkg create tank_gazebo --build-type ament_cmake --dependencies ament_cmake rclcpp rclpy -->

<!-- ros2 pkg create tank_description --build-type ament_cmake --dependencies ament_cmake urdf -->

<!-- colcon build --symlink-install --packages-ignore dolly_ignition -->

<!-- # Initial Setup ROS and colcon -->
<!-- # https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#prerequisites -->

<!-- source /opt/ros/foxy/setup.bash -->
<!-- # Only compile certian packages -->
<!-- colcon build --symlink-install --packages-select tank_gazebo tank_description -->
<!-- # Compile everything if you want -->
<!-- colcon build --symlink-install -->
<!-- source install/setup.bash -->

<!-- # Install Dependencies -->
<!-- # https://docs.ros.org/en/dashing/Installation/Linux-Development-Setup.html#install-dependencies-using-rosdep -->
<!-- sudo rosdep init -->
<!-- rosdep update -->
<!-- # rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers" -->
<!-- rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers" -->

<!-- # Launch single liner -->
<!-- ros2 launch tank_gazebo tank_launch.py -->
<!-- ros2 launch tank_gazebo multi_tank_launch.py -->

<!-- # If you want separted -->
<!-- ros2 launch tank_gazebo start_world_launch.py -->
<!-- ros2 launch tank_description start_world_launch.py -->

<!-- ros2 topic list -->

<!-- ros2 topic pub /tank/cmd_vel -->

<!-- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank/cmd_vel -->
<!-- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank2/cmd_vel -->

<!-- ros2 run tf2_tools view_frames.py -->

<!-- ########### -->

<!-- This part is missing libraries that cannot be satisfied with any available stage-packages known to snapcraft: -->
<!-- - libnddsc.so -->
<!-- - libnddscore.so -->
<!-- - libnddscpp.so -->
<!-- - librosidl_typesupport_connext_c.so -->
<!-- - librosidl_typesupport_connext_cpp.so -->
<!-- - librticonnextmsgcpp.so -->
<!-- - usr/lib/x86_64-linux-gnu/libpsm_infinipath.so.1 -->
<!-- These dependencies can be satisfied via additional parts or content sharing. Consider validating configured filesets if this dependency was built. -->

<!-- https://www.theconstructsim.com/ros2-how-to-2-create-a-ros2-action-server/ -->

<!-- 참고 -->
<!--  - https://www.udemy.com/course/ros2-for-beginners/?referralCode=18C75F99C1A868F0A7AB -->


## Raspberry Pi 3에 ROS 2 설치

준비물
 - [Ubuntu 20.04: Raspberry Pi Generic (64-bit ARM) preinstalled server image](https://cdimage.ubuntu.com/releases/20.04/release/)
 - balenaEther
 - SD카드 32GB

### ROS 2 설치 준비
 - 키보드, 모니터 연결하여 네트워크 설정
 - timezone 설정
 - swap 추가
 - ROS_DOMAIN_ID 설정: 노트북, Raspberry Pi 같은 도메인 ID로 지정.

#### WIFI 네트워크 설정

 - WIFI AP: "baribarilab"
 - WIFI PASSWORD: "1111100000"

/etc/netplan/50-cloud-init.yaml
```
 version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "YOUR_WIFI_NAME":
        password: "YOUR_WIFI_PASSWORD"
```

```
sudo netplan --debug apply
```

```
ip address
```

#### Timezone 설정

timezone asia/seoul 설정
```
timedatectl set-timezone asia/seoul
```

#### Swap 추가

Swap 추가

```
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
```

Swap 생성 확인
```
Setting up swapspace version 1, size = 1024 MiB (1073737728 bytes)
no label, UUID=09c9cd5c-f066-48f0-b988-7b000b69ed3d
```

Swap 추가
```
sudo swapon /swapfile

swapfile 파티션 테이블에 추가
```
sudo nano /etc/fstab
```
```
/swapfile swap swap defaults 0 0
```

swap 확인
```
sudo free -h

total        used        free      shared  buff/cache   available
Mem:           906M        144M        553M        6.6M        208M        740M
Swap:          1.0G          0B        1.0G
```

### ROS 2 Foxy 패키지 설치

```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```
sudo apt install ros-foxy-ros-base -y
```

base 패키지는 GUI툴이 포함되어 있지 않다.

```
echo "source /opt/ros/foxy/setup.bash" >>~/.bashrc
```

### 원격 접속하기

 - 아이디 ubuntu
 - 암호: ubuntu

```
ssh ubuntu@192.168.88.??
```

### 하드웨어 설치

#### IMU

mpu9250 연결
![](https://images.squarespace-cdn.com/content/v1/59b037304c0dbfb092fbe894/1573589401909-GKK8YB7UJ9FLCCYBDDRP/rpi_mpu9250_wiring_diagram.png?format=2500w)

https://abyz.me.uk/rpi/pigpio/download.html

연결 확인
```
i2cdetect -y 1
```

0x68

#### 모터 드라이버

L298n
![](https://i.imgur.com/omSvD8A.png)

![](https://i.imgur.com/Cn6y8qp.png)

연결
![](https://imgur.com/0dio7EM.png)

#### 카메라
raspberry pi camera v1

https://github.com/christianrauch/raspicam2_node

#### 라이다
rplidar a1

https://github.com/Slamtec/sllidar_ros2

<!-- https://www.theconstructsim.com/tag/ros2 -->

<!-- ## ROS2 패키지 만들기 -->
<!-- C++ -->

<!-- source /opt/ros/foxy/setup.bash -->

<!-- ros2 pkg create <package_name> --build-type <build_type> --dependencies <dependencies_separated_by_single_space> -->


<!-- ros2_ws/src -->
<!-- ros2 pkg create ros2_hello_cpp_pkg --build-type ament_cmake --dependencies rclcpp -->

<!-- cd ros2_hello_cpp_pkg/src -->

<!-- touch ros2_hello.cpp -->

<!-- #include "rclcpp/rclcpp.hpp" -->

<!-- int main(int argc, char *argv[]) { -->
<!--   rclcpp::init(argc, argv); -->
<!--   auto node = rclcpp::Node::make_shared("Hello"); -->

<!--   RCLCPP_INFO(node->get_logger(), -->
<!--               "Hello ROS 2 World"); -->

<!--   rclcpp::shutdown(); -->
<!--   return 0; -->
<!-- } -->



<!-- CMakeLists.txt -->
<!-- ``` -->
<!-- add_executable(hello src/ros2_hello.cpp) -->
<!-- ament_target_dependencies(hello rclcpp) -->

<!-- install(TARGETS -->
<!--     hello -->
<!--     DESTINATION lib/${PROJECT_NAME} -->
<!-- ) -->
<!-- ``` -->

<!-- 컴파일 -->

<!-- cd /home/user/ros2_ws -->

<!-- colcon build --symlink-install -->

<!-- source install/setup.sh -->

<!-- ros2 run ros2_hello_cpp_pkg hello -->


<!-- python 버전 -->

<!-- ``` -->

<!-- import rclpy -->
<!-- from rclpy.node import Node -->
<!-- from box_bot_perception.dummy_class import Dummy -->
<!-- from std_msgs.msg import String -->


<!-- class MinimalPublisher(Node): -->

<!--     def __init__(self): -->
<!--         super().__init__('minimal_publisher') -->
<!--         self.dummy_obj = Dummy() -->
<!--         self.publisher_ = self.create_publisher(String, '/box_bot_talker', 10) -->
<!--         timer_period = 0.5  # seconds -->
<!--         self.timer = self.create_timer(timer_period, self.timer_callback) -->


<!--     def timer_callback(self): -->
<!--         msg = String() -->
<!--         talk_text = self.dummy_obj.talk() -->
<!--         msg.data = "Dummy Said:"+str(talk_text) -->
<!--         self.publisher_.publish(msg) -->
<!--         self.get_logger().info('Publishing: "%s"' % msg.data) -->



<!-- def main(args=None): -->
<!--     rclpy.init(args=args) -->

<!--     minimal_publisher = MinimalPublisher() -->

<!--     rclpy.spin(minimal_publisher) -->

<!--     # Destroy the node explicitly -->
<!--     # (optional - otherwise it will be done automatically -->
<!--     # when the garbage collector destroys the node object) -->
<!--     minimal_publisher.destroy_node() -->
<!--     rclpy.shutdown() -->


<!-- if __name__ == '__main__': -->
<!--     main() -->
<!-- ``` -->


<!-- rpi2 cam on ros2 -->
<!-- https://www.youtube.com/watch?v=MlYWtDNsvgw -->
