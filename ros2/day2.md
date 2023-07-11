# ROS 2 Programming Day 2

- 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
- 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
- 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

- 목표: 시뮬레이터를 이용하여 노드를 프로그래밍 할 수 있다.
- 교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day2](https://learn.dronemap.io/ros-workshop/ros2/#/day2)
- 코치: 박동희 dongheepark@gmail.com

2. 자율 주행 소프트웨어 실습

- 시뮬레이터 사용하기
- SLAM 소개
- ROS와 시뮬레이터를 이용하여 SLAM 실습

3. Raspberry Pi에 ROS 2 개발 환경 구성

- 네트워크 설정
- Raspberry Pi에 ROS 2 설치
- 모터, IMU ROS 노드 만들기

## SLAM 소개

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

## Gazebo 시뮬레이터를 이용하여 SLAM 실습

Gazebo 시뮬레이터를 이용하여, SLAM과 자율 주행을 테스트 해보자.

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

#### ROS Gazebo 패키지 설치

```
sudo apt install ros-foxy-gazebo-dev ros-foxy-gazebo-plugins ros-foxy-gazebo-msgs  ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros ros-foxy-ros-core ros-foxy-geometry2
```

#### SLAM, Navigation 패키지 설치

```
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

#### 터틀봇 패키지 설치

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

#### 시뮬레이터 실행

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![](https://i.imgur.com/eImcWz6.png)

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

![](https://i.imgur.com/MDWDPgQ.png)

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
```

맵과 로봇 프레임 좌표 연결 확인

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

#### 해보기: turtlebot3_dqn_stage3.launch.py

- ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage3.launch.py 실행하여 자율 주행 해보자.

#### Gazebo를 이용하여 로봇 모델 만들기

1. 차동 주행 월드 로드

```
gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

![](https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive.png)

1. 월드 파일 읽어 보기

/opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world

월드 파일에서 플러그인과 출력 확인

1. 로봇 조정

```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

![](https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive_lin_vel.gif)

---

모델 구성 요소
 - LINK
 - JOINT
 - URDF: Unified Robot Description Format. 로봇의 geometry와 구성을 명세.
 - 관성모멘트: 주어진 축을 중심으로 일어나는 회전 운동을 변화시키기 어려운 정도


Link 

![](http://wiki.ros.org/urdf/XML/link?action=AttachFile&do=get&target=inertial.png)

```
 <link name="my_link">
   <inertial>
     <origin xyz="0 0 0.5" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
   </inertial>

   <visual>
     <origin xyz="0 0 0" rpy="0 0 0" />
     <geometry>
       <box size="1 1 1" />
     </geometry>
     <material name="Cyan">
       <color rgba="0 1.0 1.0 1.0"/>
     </material>
   </visual>

   <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <cylinder radius="1" length="0.5"/>
     </geometry>
   </collision>
 </link>
```

Joint

![](http://wiki.ros.org/urdf/XML/joint?action=AttachFile&do=get&target=joint.png)

```
 <joint name="my_joint" type="floating">
    <origin xyz="0 0 1" rpy="0 0 3.1416"/>
    <parent link="link1"/>
    <child link="link2"/>

    <calibration rising="0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
    <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
    <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
 </joint>
```

ros2-control gazebo-ros2-control 설치
```
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control ros-foxy-xacro
```

simple_joint 패키지 설치
```
cd ~/ros2_ws/src
git clone https://github.com/donghee/simple_joint
cd ~/ros2_ws
colcon build --packages-select simple_joint
```

simple joint gazebo 실행
```
cd ~/ros2_ws
source ./install/setup.bash
ros2 launch simple_joint_gazebo simple_joint_launch.py
```

ros2 control joint state와  trajectory controller 로드 (새로운 터미널에서)
```
cd ~/ros2_ws
source ./install/setup.bash
ros2 control load_controller --set-state start joint_state_broadcaster
ros2 control load_controller --set-state start joint_trajectory_controller
```

조인트 제어
```
python3 ~/ros2_ws/install/simple_joint_description/lib/simple_joint_description/wheel_steer.py -0.5
```

<!-- ``` -->
<!-- ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{ trajectory: { joint_names: [body_steering__wheel_stir_right], points: [ { positions: [0, 0, 0.5, 0.5] }]}, goal_time_tolerance: { sec: 1, nanosec: 0 } }" -->
<!-- ``` -->

## Raspberry Pi 4에 ROS 2 설치

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

sudo nano /etc/netplan/50-cloud-init.yaml

```
network:
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

timezone Asia/Seoul 설정

```
timedatectl set-timezone Asia/Seoul
```

#### hostname 변경

sudo nano /etc/hostname

```
ubuntu
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
```

swapfile 파티션 테이블에 /swapfile 추가

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

빌드 도구 설치하기

```
sudo apt install python3-colcon-common-extensions python3-rosdep2 build-essential libboost-dev
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

_mpu9250과 Raspberry Pi 연결하기_
![](https://images.squarespace-cdn.com/content/v1/59b037304c0dbfb092fbe894/1573589401909-GKK8YB7UJ9FLCCYBDDRP/rpi_mpu9250_wiring_diagram.png?format=2500w)

https://abyz.me.uk/rpi/pigpio/download.html

연결 확인

```
i2cdetect -y 1
```

0x68

```
ubuntu@doodoong:~$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

permission 문제가 있을경우, 다음 명령으로 i2c 그룹에 ubuntu 유저를 추가한다. 추가 후 다시 로그인 해야 그룹이 적용.

```
sudo usermod -G i2c "$USER"
```

#### 모터 드라이버

L298n

![](https://i.imgur.com/omSvD8A.png)

![](https://i.imgur.com/Cn6y8qp.png)

_L298n과 Raspberry Pi 연결하기_

![](https://imgur.com/0dio7EM.png)

![](https://www.raspberrypi-spy.co.uk/wp-content/uploads/2012/06/raspberry_pi_3_model_b_plus_gpio.jpg)

#### 카메라

raspberry pi camera v1

https://github.com/christianrauch/raspicam2_node

#### 라이다

rplidar a1

https://github.com/Slamtec/sllidar_ros2

<!-- ---- -->

<!-- ### 소프트웨어 드라이버 -->

<!-- Raspberry Pi GPIO 드라이버 -->

<!-- pigpio -->

<!-- http://abyz.me.uk/rpi/pigpio/download.html -->

<!-- 설치 -->

<!-- ``` -->
<!-- sudo apt install python-setuptools python3-setuptools -->
<!-- wget https://github.com/joan2937/pigpio/archive/master.zip -->
<!-- unzip master.zip -->
<!-- cd pigpio-master -->
<!-- make -->
<!-- sudo make install -->
<!-- ``` -->

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

---

diffdrive-arduino

필요한 패키지
ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-hardware-interface ros-foxy-xacro

---

icra2023 ros2 gz tutorial

- https://drive.google.com/drive/folders/1HeveECTM4RTbKhFgO0gieUqawBCQKZf0
- https://github.com/osrf/icra2023_ros2_gz_tutorial#quick-start
