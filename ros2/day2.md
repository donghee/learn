# ROS 2 Programming Day 2

 - 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
 - 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
 - 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

 - 목표: SLAM에 대해서 이해하고, 시뮬레이터와 로봇 하드웨어를 통해서 SLAM을 실습할 수 있다.
 - 교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day2](https://learn.dronemap.io/ros-workshop/ros2/#/day2)
 - 코치: 박동희 dongheepark@gmail.com

1. SLAM 소개
2. Gazebo 시뮬레이터를 이용하여 SLAM 실습
3. Gazebo를 이용하여 로봇 모델 만들기
4. 자율 주행 하드웨어 소개
5. 자율 주행 하드웨어 코드 분석
6. SLAM 주행 테스트

---

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

![](http://wiki.ros.org/urdf/XML/link?action=AttachFile&do=get&target=inertial.png)

 - URDF: Unified Robot Description Format. 로봇의 geometry와 구성을 명세.
 - 관성모멘트: 주어진 축을 중심으로 일어나는 회전 운동을 변화시키기 어려운 정도

ros2-control gazebo-ros2-control 설치
```
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controller ros-foxy-gazebo-ros2-control ros-foxy-xacro
```

simple_joint 패키지 설치
```
cd ~/ros2_ws/src
git clone https://github.com/donghee/simple_joint
cd ~/ros2_ws
colcon build simple_joint
```

simple joint gazebo 실행
```
ros2 launch simple_joint_gazebo simple_joint_launch.py
```


ros2 control joint state와  trajectory controller 로드 (새로운 터미널에서)
```
ros2 control load_controller --set-state start joint_state_broadcaster
ros2 control load_controller --set-state start joint_trajectory_controller
```

조인트 제어
```
python3 /home/donghee/ros2_ws/install/simple_joint_description/lib/simple_joint_description/wheel_steer.py -0.5
```

<!-- ``` -->
<!-- ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{ trajectory: { joint_names: [body_steering__wheel_stir_right], points: [ { positions: [0, 0, 0.5, 0.5] }]}, goal_time_tolerance: { sec: 1, nanosec: 0 } }" -->
<!-- ``` -->

## 자율 주행 하드웨어 소개

 - https://omorobot.com/docs/omo-r1mini

## 자율 주행 하드웨어 코드 분석

 - https://github.com/omorobot/omo_r1mini-foxy

## SLAM 주행 테스트

 https://cloud.baribarilab.com/apps/sharingpath/donghee/Public/2022/ros2/omo_r1mini_ros2-foxy-slam-demo.mp4
