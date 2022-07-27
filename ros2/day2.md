# ROS 2 Programming Day 2

 - 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
 - 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
 - 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

-   목표: 나만의 ROS 노드 만들고, 시뮬레이터를 이용하여 노드를 프로그래밍 할 수 있다.
-   교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day1](https://learn.dronemap.io/ros-workshop/ros2/#/day2)
-   코치: 박동희 dongheepark@gmail.com

1. ROS 기본 프로그래밍
 - ROS를 자율 주행 로봇을 제어하는 ROS 노드 만들기
 - 시뮬레이터 사용하기
 - 컴퓨터 비전 소개 및 카메라 사용 실습
2. 자율 주행 소프트웨어 실습
 - SLAM 소개
 - ROS와 시뮬레이터를 이용하여 SLAM 실습

---

## ROS 노드 만들기

워크스페이스 만들기
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

No 빌드 도구 colcon 설치하기
```
sudo apt install python3-colcon-common-extensions
```

**패키지 빌드하기**

1. examples 패키지 워크스페이스의 src 디렉토리 아래에 다운로드
```
git clone https://github.com/ros2/examples src/examples -b foxy
```
2. examples 패키지 빌드
```
colcon build --symlink-install
```

빌드한 패키지 실행
 1. 환경 설정
```
. install/setup.bash
```
 2. 노드 실행
구독 노드 실행
```
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```
출판 노드 실행
```
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

---

https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros

### 시뮬레이터 Gazebo

#### Gazebo 11 설치

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt key add -
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

#### Gazebo 사용하기

1. 차동 주행 월드 로드

```
gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

![]https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive.png)

1. 월드 파일 읽어 보기

```
gedit /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

월드 파일에서 `gazebo_ros_pkgs` 확인

1. 로봇 조정
```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

![](https://github.com/osrf/gazebo_tutorials/raw/master/ros2_installing/figs/gazebo_ros_diff_drive_lin_vel.gif)

---
## 가제보 시뮬레이션 로봇 만들기
https://bitbucket.org/theconstructcore/box_bot/src/galactic/

tank_gazebo

ros2 pkg create tank_gazebo --build-type ament_cmake --dependencies ament_cmake rclcpp rclpy

ros2 pkg create tank_description --build-type ament_cmake --dependencies ament_cmake urdf


colcon build --symlink-install --packages-ignore dolly_ignition

# Initial Setup ROS and colcon
# https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#prerequisites

source /opt/ros/foxy/setup.bash
# Only compile certian packages
colcon build --symlink-install --packages-select tank_gazebo tank_description
# Compile everything if you want
colcon build --symlink-install
source install/setup.bash

# Install Dependencies
# https://docs.ros.org/en/dashing/Installation/Linux-Development-Setup.html#install-dependencies-using-rosdep
sudo rosdep init
rosdep update
# rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"


# Launch single liner
ros2 launch tank_gazebo tank_launch.py
ros2 launch tank_gazebo multi_tank_launch.py

# If you want separted
ros2 launch tank_gazebo start_world_launch.py
ros2 launch tank_description start_world_launch.py

ros2 topic list

ros2 topic pub /tank/cmd_vel

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank/cmd_vel

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank2/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank2/cmd_vel

ros2 run tf2_tools view_frames.py

###########


This part is missing libraries that cannot be satisfied with any available stage-packages known to snapcraft:
- libnddsc.so
- libnddscore.so
- libnddscpp.so
- librosidl_typesupport_connext_c.so
- librosidl_typesupport_connext_cpp.so
- librticonnextmsgcpp.so
- usr/lib/x86_64-linux-gnu/libpsm_infinipath.so.1
These dependencies can be satisfied via additional parts or content sharing. Consider validating configured filesets if this dependency was built.



---
turtle3 slam

https://www.theconstructsim.com/ros2-how-to-2-create-a-ros2-action-server/

---

ROS 2 빌드툴 Colcon

ROS 2 워크스페이스 만들기

파이썬 패키지 만들기

C++ 패키지 만들기

Publisher and Subscriber

Client and Server

Launch File


![](https://roboticsbackend.com/wp-content/uploads/2020/10/ros2_multiple_machines-1024x554.png)

ROS_DOMAIN_ID=5

printenv


참고
 - https://www.udemy.com/course/ros2-for-beginners/?referralCode=18C75F99C1A868F0A7AB


## Raspberry Pi 3에 ROS2 설치

준비물
 - [Ubuntu 20.04: Raspberry Pi Generic (64-bit ARM) preinstalled server image](https://cdimage.ubuntu.com/releases/20.04/release/)
 - balenaEther
 - SD카드 32GB


WIFI 네트워크 설정

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

timezone 설정
```
timedatectl set-timezone asia/seoul
```

Swap 추가

```
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
```

```
Setting up swapspace version 1, size = 1024 MiB (1073737728 bytes)
no label, UUID=09c9cd5c-f066-48f0-b988-7b000b69ed3d
```

```
sudo swapon /swapfile
sudo vi /etc/fstab
sudo free -h
```

```
  total        used        free      shared  buff/cache   available
Mem:           906M        144M        553M        6.6M        208M        740M
Swap:          1.0G          0B        1.0G
```


```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```
sudo apt install ros-foxy-ros-base -y
```

base 패키지는 GUI 툴이 설치 안된다.


```
echo "source /opt/ros/foxy/setup.bash" >>~/.bashrc
```

테스트 패키지 설치

```
sudo apt install ros-foxy-demo-nodes-cpp
sudo apt install ros-foxy-demo-nodes-py
```

```
ros2 run demo_nodes_py listener
```

```
ros2 run demo_nodes_cpp talker
```

----

https://www.theconstructsim.com/tag/ros2

## ROS2 패키지 만들기
C++

source /opt/ros/foxy/setup.bash


ros2 pkg create <package_name> --build-type <build_type> --dependencies <dependencies_separated_by_single_space>


ros2_ws/src
ros2 pkg create ros2_hello_cpp_pkg --build-type ament_cmake --dependencies rclcpp

cd ros2_hello_cpp_pkg/src

touch ros2_hello.cpp

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Hello");

  RCLCPP_INFO(node->get_logger(),
              "Hello ROS 2 World");

  rclcpp::shutdown();
  return 0;
}



CMakeLists.txt
```
add_executable(hello src/ros2_hello.cpp)
ament_target_dependencies(hello rclcpp)

install(TARGETS
    hello
    DESTINATION lib/${PROJECT_NAME}
)
```

컴파일

cd /home/user/ros2_ws

colcon build --symlink-install

source install/setup.sh

ros2 run ros2_hello_cpp_pkg hello


python 버전

```

import rclpy
from rclpy.node import Node
from box_bot_perception.dummy_class import Dummy
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.dummy_obj = Dummy()
        self.publisher_ = self.create_publisher(String, '/box_bot_talker', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = String()
        talk_text = self.dummy_obj.talk()
        msg.data = "Dummy Said:"+str(talk_text)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
