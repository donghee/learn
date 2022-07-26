

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


Raspberry Pi 3에 ROS2 설치


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
