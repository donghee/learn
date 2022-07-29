# ROS 2 Programming Day 3

 - 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
 - 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
 - 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

-   목표: 자율 주행에 필요한 주변장치를 제어하고, 자율 주행 프로젝트를 할 수 있다.
-   교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day3](https://learn.dronemap.io/ros-workshop/ros2/#/day3)
-   코치: 박동희 dongheepark@gmail.com

1. 자율 주행 하드웨어 구성
 - 자율 주행 하드웨어(탱크) 소개
 - 탱크 프레임 조립, 센서 및 모터 드라이버 하드웨어 구성
 - ROS 를 이용하여 센서(Lidar, 거리센서), 모터 드라이버(L293D)  제어

2. 자율 주행 실습
 - 조립한 자율 주행 탱크에 자율 주행 소프트웨어(ROS 노드) 적용
 - 주행 테스트
 - 자율 프로젝트

## 자율 주행 하드웨어 소개

![](https://i.imgur.com/QMcDV3r.jpg)

![](https://i.imgur.com/IjhXEDr.jpg)

![](https://i.imgur.com/qcDHWaj.jpg)

 - L298n H-Bridge
 - MPU9250 IMU
 - Raspberry Pi Camera V1
 - RPLIDAR A1
 - 몸체 프레임
 - DC 모터
 - 3S LiPo 배터리
 - DCDC 컨버터
 - Raspberry Pi 3
 - Raspberry Pi용 5V 어댑터

## 탱크 주변 장치 제어하기

### L298N

l298n 연결 확인

<!--
```
sudo apt install python3-lgpio
```
-->

RPi.GPIO 라이브러리 설치
```
sudo apt-get install python3-rpi.gpio
```

python3 rpi_l298n_test.py

```
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# forward
GPIO.output(12, GPIO.HIGH)
GPIO.output(13, GPIO.HIGH)

GPIO.output(21, GPIO.HIGH)
GPIO.output(20, GPIO.LOW)
GPIO.output(23, GPIO.HIGH)
GPIO.output(24, GPIO.LOW)

time.sleep(10)

GPIO.output(12, GPIO.LOW)
GPIO.output(13, GPIO.LOW)

# backward
GPIO.output(12, GPIO.HIGH)
GPIO.output(13, GPIO.HIGH)

GPIO.output(21, GPIO.LOW)
GPIO.output(20, GPIO.HIGH)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.HIGH)

time.sleep(10)

GPIO.output(12, GPIO.LOW)
GPIO.output(13, GPIO.LOW)
```

permission 에러가 나는 경우 python3를 root로 실행
```
sudo python3
```

#### 해보기: 회전
 - 1초 동안 좌우 회전을 시켜보자.
 
### IMU

Inertial Measurement Unit 관성 측정 장치
 - 가속도 센서(Acceleration) 
 - 각속도 센서(Gyroscope)
 - 지자기 센서(Magnetometer)

Axis Orientation

x forward, y left, z up 

![](https://velog.velcdn.com/images%2Fleesy970906%2Fpost%2F13213a98-13c4-4d00-9968-f461bffd90b0%2FScreen%20Shot%202021-04-08%20at%204.40.12%20PM.png)

right hand rule
![](https://velog.velcdn.com/images%2Fleesy970906%2Fpost%2Fe6042f9c-7c3b-4878-a288-8dca47ab6a33%2FScreen%20Shot%202021-04-08%20at%204.42.02%20PM.png)

참고: https://www.ros.org/reps/rep-0103.html

https://github.com/Schnilz/mpu9250

#### 해보기
 - mpu9250_calibration.py를 실행하여, imu의 roll, pitch, yaw 방향을 ROS의 axis orientaton과 right hand rule에 맞게 mpu9250 모듈을 로봇에 부착하자.
 - mpu9250_calibration.py를 이용하여, 센서값을 캘리브레이션 하여, gyro, accel, mag 센서의 bias값을 구하자.


### Raspberry Pi Camera 사용하기

<!--
libraspberrypi-bin libraspberrypi-dev 설치
```
sudo apt autoremove --purge libgles2-mesa-dev mesa-common-dev
sudo add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
sudo apt install libraspberrypi-bin libraspberrypi-dev
```

raspicam2_node

https://github.com/christianrauch/raspicam2_node

```
cd ~/ros2_ws/src
git clone https://github.com/christianrauch/raspicam2_node
cd ~/ros2_ws
colcon build
```

video 그룹에 유저 추가후 리부팅
```
sudo usermod -a -G video $USER
```

노드 실행
```
ros2 run raspicam2 raspicam2_node --ros-args --params-file `ros2 pkg prefix raspicam2`/share/raspicam2/cfg/params.yaml
```
-->


```
wget https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20200601_all.deb -P /tmp
```

```
sudo dpkg -i /tmp/raspi-config_20200601_all.deb
```

```
sudo apt --fix-broken install
```

카메라 활성화

```
sudo mount /dev/mmcblk0p1 /boot
```

```
sudo raspi-config

# 5. Interface Options -> P1 Camera -> Enable camera
```

카메라 설치 확인
```
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```


ros2 camera node 설치

```
sudo apt install ros-foxy-cv-bridge ros-foxy-camera-info-manager ros-foxy-camera-calibration-parsers ros-foxy-image-common ros-foxy-image-transport ros-foxy-v4l2-camera
```

<!--
v4l2-camera 패키지 source 설치
```
cd ros2_ws/src/
#git clone --branch ros2 https://github.com/ros-perception/image_common.git
#git clone --branch ros2 https://github.com/ros-perception/vision_opencv.git
git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git v4l2_camera
```

의존성 패키지 설치
```
cd ros2_ws
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -r -y
```

camera node 패키지 빌드
```
cd ros2_ws
colcon build --symlink-install
source ~/install/setup.bash
```

에러. camera_info_manager.cpp 파일에 다음줄 추가 
```
#include "rcpputils/get_env.hpp"
```

-->

camera node 실행
```
ros2 run v4l2_camera v4l2_camera_node
```

pc에서 camera 이미지 보기
```
ros2 run rqt_image_view rqt_image_view
```

해상도 수정
```
ros2 param set /v4l2_camera image_size '[320, 240]'
ros2 param get /v4l2_camera image_size
```

#### 해보기: 컴퓨터 비전 노드 만들기
 - 참고: https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/

### RPLIDAR A1

https://www.slamtec.com/en/Lidar/A1

![](https://mblogthumb-phinf.pstatic.net/MjAyMDA1MThfNTgg/MDAxNTg5NzczODkwMTAz.OF3ryBtnNzMRC8Zgtgz0l0M0mNOboq_Q3L95MFhYg6Ug.w7erXC3UCYdhj3tk1F8h0-3jCi7IsNhEqSKmswLQdyYg.PNG.rich0812/SE-18a6b694-89cf-4e5b-ae73-99e1b92f736f.png?type=w800)

ROS2 노드

https://github.com/Slamtec/sllidar_ros2.git

```
cd ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ros2_ws
colcon build
```

#### 해보기: A1 라이더 값 받기
 - rplidar 센서값을 ros2 topic echo를 이용해서 받아 보자.
 - ros2 노드를 구성해서 라이더 값을 받아서 라이더 값의 최소값을 구해보자.
 - 참고: https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_example/nodes/turtlebot3_obstacle



## 프로젝트
 - 1. 경주
 - 2. 장애물 회피!
 
 
 
<!-- --- -->

<!-- ## TF -->

<!-- ![](https://blog.hadabot.com/images/tf_hadabot.jpg) -->

<!-- https://blog.hadabot.com/ros2-navigation-tf2-tutorial-using-turtlesim.html -->

<!-- https://github.com/ros2/geometry2/blob/ros2/examples_tf2_py/examples_tf2_py/static_broadcaster.py -->

<!-- ros2 run examples_tf2_py static_broadcaster -->

<!-- https://github.com/hadabot/hadabot_main/blob/master/content/p12/hadabot_ws/src/hadabot_nav2/src/hadabot_tf2_broadcaster.cpp -->

<!-- ros2 run tf2_tools view_frames.py -->

<!-- ![](https://i.imgur.com/cq1MgBY.png) -->

<!-- ## Odometry -->

<!-- Differential Drive Robot Odometry 수식 -->
<!--  - 바퀴 크기 -->
<!--  - 바퀴 사이거리 -->
<!--  - -->
<!-- https://blog.hadabot.com/implement-ros2-odometry-using-vscode-in-web-browser.html -->


<!-- turtlesim odom 메시지 -->

<!-- ros2 topic echo /turtlesim1/turtle1/pose -->

<!-- rviz -->


<!-- Navigation2 -->
<!--  - planning -->
<!--  - localization -->
<!--  - mapping(SLAM Simultaneous localization and mapping) -->


<!-- Control 구현 -->
<!-- 엔코더, 휠, 콘트롤러 구현됨. -->
<!-- https://github.com/hadabot/hadabot_main/blob/master/content/p12/hadabot_ws/src/hadabot_nav2/src/hadabot_controller.cpp -->


<!-- ## Navigation -->
<!-- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py -->

<!-- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True autostart:=True map:=`ros2 pkg prefix turtlebot3_navigation2`/share/turtlebot3_navigation2/map/map.yaml -->

<!-- ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz -->


<!-- ## tank -->

<!-- ``` -->
<!-- # tf2 -->
<!-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map -->
<!-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom -->
<!-- ros2 run tf2_ros static_transform_publisher -0.05 0 0.3 0 0 0 base_link laser -->
<!-- ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link imu_link -->

<!-- # two dc motor control -->
<!-- colcon build --packages-select tank_control && . install/setup.sh && ros2 run tank_control robot_control -->

<!-- # imu -->
<!-- cd ros2_ws -->
<!-- ros2 launch mpu9250 mpu9250.launch.py -->

<!-- # odometry -->
<!-- ros2 run robot_localization ekf_node --ros-args --params-file $HOME/ekf_params.yaml -->

<!-- # lidar -->
<!-- cd ros2_ws -->
<!-- ros2 launch sllidar_ros2 sllidar_launch.py -->


<!-- # pc -->
<!-- ros2 run teleop_twist_keyboard teleop_twist_keyboard -->

<!-- ros2 launch tank_cartographer cartographer.launch.py -->
<!-- ros2 launch tank_cartographer cartographer_rviz.launch.py -->

<!-- cd ~/ -->
<!-- ros2 run nav2_map_server map_saver_cli -f map -->

<!-- ros2 launch tank_navigation2 navigation2.launch.py map:=$HOME/map.yaml -->
<!-- ros2 launch tank_navigation2 navigation2_rviz.launch.py -->
<!-- ``` -->


<!-- ``` -->
<!--  ekf_params.yaml -->
<!-- ekf_filter_node: -->
<!--     ros__parameters: -->
<!--          #frequency: 30.0 -->
<!--         frequency: 10.0 -->
<!--         sensor_timeout: 0.1 -->
<!--         two_d_mode: true -->
<!--         publish_acceleration: false -->
<!--         publish_tf: true -->
<!--         reset_on_time_jump: true -->
<!--         map_frame: map              # Defaults to "map" if unspecified -->
<!--         odom_frame: odom            # Defaults to "odom" if unspecified -->
<!--         base_link_frame: base_link  # Defaults to "base_link" if unspecified -->
<!--         #world_frame: map -->
<!--         world_frame: odom           # Defaults to the value of odom_frame if unspecified -->

<!--         odom0: wheel/odom -->
<!--         #x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az -->
<!--         odom0_config: [true,  true,  false, -->
<!--                        false, false, false, -->
<!--                        false, false, false, -->
<!--                        false, false, true, -->
<!--                        false, false, false] -->
<!--         odom0_queue_size: 2 -->
<!--         odom0_nodelay: false -->
<!--         odom0_differential: false -->
<!--         odom0_relative: false -->
<!--         odom0_pose_rejection_threshold: 5.0 -->
<!--         odom0_twist_rejection_threshold: 1.0 -->
<!-- ``` -->

<!-- `` -->
<!-- `l298n.c -->
<!-- /* -->
<!--    pulse.c -->

<!--    gcc -o l298n l298n.c -lpigpio -lrt -lpthread -->

<!--    sudo ./l298n -->
<!-- */ -->

<!-- #include <stdio.h> -->
<!-- #include <pigpio.h> -->

<!-- int main(int argc, char *argv[]) -->
<!-- { -->
<!--    double start; -->

<!--    if (gpioInitialise() < 0) -->
<!--    { -->
<!--       fprintf(stderr, "pigpio initialisation failed\n"); -->
<!--       return 1; -->
<!--    } -->

<!--    /* Set GPIO modes */ -->
<!--    gpioSetMode(12, PI_OUTPUT); -->
<!--    gpioSetMode(21, PI_OUTPUT); -->
<!--    gpioSetMode(20, PI_OUTPUT); -->
<!--    gpioSetMode(23, PI_OUTPUT); -->
<!--    gpioSetMode(24, PI_OUTPUT); -->
<!--    gpioSetMode(13, PI_OUTPUT); -->

<!--    gpioWrite(12, 0); /* on */ -->
<!--    gpioWrite(13, 0); /* on */ -->

<!--    /* -->
<!--    start = time_time(); -->

<!--    while ((time_time() - start) < 60.0) -->
<!--    { -->
<!--       // motor on -->
<!--       gpioWrite(12, 1); -->
<!--       gpioWrite(13, 1); -->

<!--       // forward -->
<!--       gpioWrite(21, 1); -->
<!--       gpioWrite(20, 0); -->
<!--       gpioWrite(23, 1); -->
<!--       gpioWrite(24, 0); -->

<!--       time_sleep(3.5); -->

<!--       // backward -->
<!--       gpioWrite(21, 0); -->
<!--       gpioWrite(20, 1); -->
<!--       gpioWrite(23, 0); -->
<!--       gpioWrite(24, 1); -->

<!--       time_sleep(3.5); -->

<!--       // motor stop -->
<!--       gpioWrite(12, 0); -->
<!--       gpioWrite(13, 0) -->

<!--       time_sleep(3.5); -->
<!--    } -->
<!--    */ -->

<!--    start = time_time(); -->

<!--    while ((time_time() - start) < 60.0) -->
<!--    { -->
<!--       int x = (int)(time_time() - start)*3+75; -->
<!--       gpioPWM(12, 0); -->
<!--       gpioPWM(13, 0); -->

<!--       // forward -->
<!--       gpioWrite(21, 1); -->
<!--       gpioWrite(20, 0); -->
<!--       gpioWrite(23, 1); -->
<!--       gpioWrite(24, 0); -->

<!--       gpioPWM(12, x); -->
<!--       gpioPWM(13, x); -->
<!--       time_sleep(1); -->
<!--    } -->

<!--    // stop -->
<!--    gpioWrite(12, 0); -->
<!--    gpioWrite(13, 0); -->




<!--    /* Stop DMA, release resources */ -->
<!--    gpioTerminate(); -->

<!--    return 0; -->
<!-- } -->
<!-- ``` -->


<!-- ``` -->
<!--  mpu9250_calibration.py -->
<!-- import os -->
<!-- import sys -->
<!-- import time -->
<!-- import smbus -->
<!-- import time -->

<!-- from imusensor.MPU9250 import MPU9250 -->

<!-- address = 0x68 -->
<!-- bus = smbus.SMBus(1) -->
<!-- imu = MPU9250.MPU9250(bus, address) -->
<!-- imu.begin() -->
<!-- #imu.setAccelRange("AccelRangeSelect2G") -->
<!-- #imu.setGyroRange("GyroRangeSelect250DPS") -->

<!-- #print("Gyro calibration starting") -->
<!-- #imu.caliberateGyro() -->
<!-- #print("Gyro calibration finished") -->
<!-- #print('gyro bias') -->
<!-- #print(imu.GyroBias) -->

<!-- #print("Accel calibration starting") -->
<!-- #imu.caliberateAccelerometer() -->
<!-- #print("Accel calibration finished") -->
<!-- #print('accel scale') -->
<!-- #print(imu.Accels) -->
<!-- #print('accel bias') -->
<!-- #print(imu.AccelBias) -->

<!-- print("Mag calibration starting") -->
<!-- time.sleep(2) -->
<!-- imu.caliberateMagPrecise() -->
<!-- print("Mag calibration finished") -->
<!-- print('mag transformation matrix') -->
<!-- print(imu.Magtransform) -->
<!-- print('mag bias') -->
<!-- print(imu.MagBias) -->
<!-- print('mags') -->
<!-- print (imu.Mags) -->

<!-- imu.saveCalibDataToFile("/home/ubuntu/calib.json") -->
<!-- # or load your own caliberation file -->
<!-- #imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json") -->

<!-- while True: -->
<!--     imu.readSensor() -->
<!--     imu.computeOrientation() -->

<!--     print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw)) -->
<!--     time.sleep(0.1) -->
<!-- ``` -->

<!-- https://github.com/Schnilz/mpu9250 -->


<!-- rplidar a1m8 -->

<!-- https://github.com/Slamtec/sllidar_ros2.git -->



<!-- ---- -->
<!-- ``` -->
<!-- drwxrwxr-x 11 donghee donghee 4096 Jul 26 21:44 . -->
<!-- drwxr-xr-x 57 donghee donghee 4096 Jul 26 00:02 .. -->
<!-- drwxrwxr-x  5 donghee donghee 4096 Jul 24 22:54 build -->
<!-- drwxrwxr-x  5 donghee donghee 4096 Jul 24 22:54 install -->
<!-- drwxrwxr-x  6 donghee donghee 4096 Jul 24 22:59 log -->
<!-- drwxrwxr-x  7 donghee donghee 4096 Jul 26 21:44 mpu9250 -->
<!-- -rw-rw-r--  1 donghee donghee  993 Jul 25 02:42 run-tank.sh -->
<!-- drwxrwxr-x  8 donghee donghee 4096 Jul 26 21:44 sllidar_ros2 -->
<!-- drwxrwxr-x  7 donghee donghee 4096 Jul 24 22:53 tank_cartographer -->
<!-- drwxrwxr-x  5 donghee donghee 4096 Jul 26 21:44 tank_control -->
<!-- drwxrwxr-x  8 donghee donghee 4096 Jul 24 22:54 tank_navigation2 -->
<!-- ``` -->
