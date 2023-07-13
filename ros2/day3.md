# ROS 2 Programming Day 3

- 수업자료: https://learn.dronemap.io/ros-workshop/ros2/
- 준비물: ubuntu 20.04 리눅스가 설치된 컴퓨터
- 참고책: ROS 2로 시작하는 로봇 프로그래밍

## 수업 소개

- 목표: 자율 주행에 필요한 주변장치를 제어하고, 자율 주행 프로젝트를 할 수 있다.
- 교재: [https://learn.dronemap.io/ros-workshop/ros2/#/day3](https://learn.dronemap.io/ros-workshop/ros2/#/day3)
- 코치: 박동희 dongheepark@gmail.com

1. 자율 주행 로봇 주변장치 제어

 - 모터 드라이버
 - Camera 사용하기
 - 라이다

2. 자율 주행 실습

- 조립한 자율 주행 탱크에 자율 주행 소프트웨어(ROS 노드) 적용
- 주행 테스트
- 자율 프로젝트

### 하드웨어 설치

![](https://i.imgur.com/AetvNwy.png)

- Raspberry Pi 4
- Raspberry Pi Camera
- Paspberry Pi 어댑터
- RPLIDAR A1
- 몸체 프레임
- 인코터 장착된 청소기 모터
- 3S LiPo 배터리
- 배터리 충전기 

#### 차동 구동로봇의 모터 드라이버

`diffbot_arduion.ino`

펌웨어: https://gist.github.com/donghee/c0319e3f95fce0dca63fb29710954119

코드 읽기

 - PID 제어: 속도 제어
 - 인코더 읽기. 1바퀴에 1200 펄스 생성

다음 코드를 참고하여 모터를 제어하고, 바퀴의 회전수 를 읽어보자.

```
#define BAUDRATE 57600

#define MOTOR_L_WCNT_PIN 2  // FS brown: Encoder
#define MOTOR_R_WCNT_PIN 3  // FS brown: Encoder

#define MOTOR_L_FR_PIN 7  // FR white: Direction
#define MOTOR_R_FR_PIN 8  // FR white: Direction

#define MOTOR_L_BREAK_PIN 9   // BREAK green: BREAK
#define MOTOR_R_BREAK_PIN 10  // BREAK green: BREAK

#define MOTOR_L_PWM_PIN 5  // PWM blue: SPEED
#define MOTOR_R_PWM_PIN 6  // PWM blue: SPEED

int l_wheel_cnt = 0;
int r_wheel_cnt = 0;

// encoder
volatile long l_duration = 0;
volatile long r_duration = 0;

void setup() {
  // 4Khz PWM  
  TCCR0B = 0b00000010;  // x8
  TCCR0A = 0b00000001;  // phase correct

  // setting the rising edge interrupt
  pinMode(MOTOR_L_WCNT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_R_WCNT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_L_WCNT_PIN), ISR_L_wheelCount, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_R_WCNT_PIN), ISR_R_wheelCount, RISING);

  Serial.begin(BAUDRATE);
}

void loop() {
  char value;
  if (Serial.available()) {
    value = Serial.read();
    if (value == 'e') { 
        Serial.print(l_wheel_cnt);
        Serial.print(' ');
        Serial.println(r_wheel_cnt);
    } else if (value == 'm') { 
        analogWrite(MOTOR_L_PWM_PIN, 127);
        analogWrite(MOTOR_R_PWM_PIN, 127);
    } else { 
        analogWrite(MOTOR_L_PWM_PIN, 255); //STOP
        analogWrite(MOTOR_R_PWM_PIN, 255); //STOP
    }
  }
}

void ISR_L_wheelCount() {
  l_wheel_cnt = l_duration / 1200;
}

void ISR_R_wheelCount() {
  r_wheel_cnt = r_duration / 1200;
}
```


모터 동작 확인

```
sudo apt-get install picocom      
picocom /dev/ttyUSB0 -b 57600
```

ctrl+a ctrl+c 를 클릭하여 타자 echo 확인

### Raspberry Pi Camera 사용하기

ROS에서 Raspberry Pi Camera 사용에 필요한 패키지 설치 

```
sudo apt install libraspberrypi-bin v4l-utils ros-foxy-v4l2-camera ros-foxy-image-transport-plugins
```
```
sudo usermod -aG video $USER
```

raspi-config 다운로드

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
재부팅

카메라 설치 확인
```
sudo usermod -G video "$USER"

sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```

```
raspistill -o cam.jpg
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

pi 카메라 실행 노드
```
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[160,120]" -p camera_frame_id:=camera_optical_link
```

pc에서 camera 이미지 보기
```
ros2 run rqt_image_view rqt_image_view
```

#### 해보기
- pi 카메라에서 나오는 영상을 pc에서 보기 

#### 라이다

**RPLIDAR A1**

https://www.slamtec.com/en/Lidar/A1

![](https://mblogthumb-phinf.pstatic.net/MjAyMDA1MThfNTgg/MDAxNTg5NzczODkwMTAz.OF3ryBtnNzMRC8Zgtgz0l0M0mNOboq_Q3L95MFhYg6Ug.w7erXC3UCYdhj3tk1F8h0-3jCi7IsNhEqSKmswLQdyYg.PNG.rich0812/SE-18a6b694-89cf-4e5b-ae73-99e1b92f736f.png?type=w800)

![](https://i.imgur.com/sJiKhWe.png ":size=600")

ROS2 노드

https://github.com/Slamtec/sllidar_ros2.git

```
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source ./install/setup.sh
./src/sllidar_ros2/scripts/create_udev_rules.sh

ros2 launch sllidar_ros2 sllidar_launch.py
```

<!-- 또다른 노드 -->

<!-- sudo apt-get install ros-foxy-rplidar-ros -->
<!-- cd ~/firstbot_ws/src -->
<!-- ros2 launch firstbot_bringup rplidar.launch.py -->


#### 해보기: A1 라이더 값 받기

- rplidar 센서값을 ros2 topic echo를 이용해서 받아 보자.
- ros2 노드를 구성해서 라이더 값을 받아서 라이더 값의 최소값을 구해보자.
<!-- - 참고: https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_example/nodes/turtlebot3_obstacle -->


### Raspberry Pi에 퍼스트봇 ZARA 패키지 설치

```
mkdir -p ~/firstbot_ws/src
cd ~/firstbot_ws/src
git clone https://github.com/donghee/firstbot_zara.git -b foxy --recursive
```

필요한 패키시 설치
```
sudo apt-get install `cat ~/firstbot_ws/src/firstbot_zara/tools/ros2-foxy-installed.txt`
```

패키지 빌드

```
cd ~/firstbot_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/firstbot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

실행

```
cd ~/firstbot_ws/src/firstbot_zara
tmuxinator local
```

tmux 사용법

`ctrl+b n` 또는 `ctrlb p` 를 이용하여 원도우 이용. `ctrl+b o` 를 이용하여 원도우 내부의 pane 이동


## 주행 테스트

#### keyboard teleop

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 프로젝트

- 1.  경주
- 2.  장애물 회피!

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
