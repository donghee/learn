셋째날

자율 주행 하드웨어 구성
 - 자율 주행 하드웨어(탱크) 소개
 - 탱크 프레임 조립, 센서 및 모터 드라이버 하드웨어 구성
 - ROS 를 이용하여 센서(Lidar, 거리센서), 모터 드라이버(L293D)  제어

자율 주행 실습
 - 조립한 자율 주행 탱크에 자율 주행 소프트웨어(ROS 노드) 적용
 - 주행 테스트
```

---

# TF

![](https://blog.hadabot.com/images/tf_hadabot.jpg)

https://blog.hadabot.com/ros2-navigation-tf2-tutorial-using-turtlesim.html

https://github.com/ros2/geometry2/blob/ros2/examples_tf2_py/examples_tf2_py/static_broadcaster.py

ros2 run examples_tf2_py static_broadcaster

https://github.com/hadabot/hadabot_main/blob/master/content/p12/hadabot_ws/src/hadabot_nav2/src/hadabot_tf2_broadcaster.cpp

ros2 run tf2_tools view_frames.py

![](https://i.imgur.com/cq1MgBY.png)

# Odometry

Differential Drive Robot Odometry 수식
 - 바퀴 크기
 - 바퀴 사이거리
 -
https://blog.hadabot.com/implement-ros2-odometry-using-vscode-in-web-browser.html


turtlesim odom 메시지

ros2 topic echo /turtlesim1/turtle1/pose

rviz


Navigation2
 - planning
 - localization
 - mapping(SLAM Simultaneous localization and mapping)


Control 구현
엔코더, 휠, 콘트롤러 구현됨.
https://github.com/hadabot/hadabot_main/blob/master/content/p12/hadabot_ws/src/hadabot_nav2/src/hadabot_controller.cpp


# Navigation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True autostart:=True map:=`ros2 pkg prefix turtlebot3_navigation2`/share/turtlebot3_navigation2/map/map.yaml

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz


# tank

```
# tf2
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run tf2_ros static_transform_publisher -0.05 0 0.3 0 0 0 base_link laser
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link imu_link

# two dc motor control
colcon build --packages-select tank_control && . install/setup.sh && ros2 run tank_control robot_control

# imu
cd ros2_ws
ros2 launch mpu9250 mpu9250.launch.py

# odometry
ros2 run robot_localization ekf_node --ros-args --params-file $HOME/ekf_params.yaml

# lidar
cd ros2_ws
ros2 launch sllidar_ros2 sllidar_launch.py


# pc
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch tank_cartographer cartographer.launch.py
ros2 launch tank_cartographer cartographer_rviz.launch.py

cd ~/
ros2 run nav2_map_server map_saver_cli -f map

ros2 launch tank_navigation2 navigation2.launch.py map:=$HOME/map.yaml
ros2 launch tank_navigation2 navigation2_rviz.launch.py
```


```
 ekf_params.yaml
ekf_filter_node:
    ros__parameters:
         #frequency: 30.0
        frequency: 10.0
        sensor_timeout: 0.1
        two_d_mode: true
        publish_acceleration: false
        publish_tf: true
        reset_on_time_jump: true
        map_frame: map              # Defaults to "map" if unspecified
        odom_frame: odom            # Defaults to "odom" if unspecified
        base_link_frame: base_link  # Defaults to "base_link" if unspecified
        #world_frame: map
        world_frame: odom           # Defaults to the value of odom_frame if unspecified

        odom0: wheel/odom
        #x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
        odom0_config: [true,  true,  false,
                       false, false, false,
                       false, false, false,
                       false, false, true,
                       false, false, false]
        odom0_queue_size: 2
        odom0_nodelay: false
        odom0_differential: false
        odom0_relative: false
        odom0_pose_rejection_threshold: 5.0
        odom0_twist_rejection_threshold: 1.0
```

``
`l298n.c
/*
   pulse.c

   gcc -o l298n l298n.c -lpigpio -lrt -lpthread

   sudo ./l298n
*/

#include <stdio.h>
#include <pigpio.h>

int main(int argc, char *argv[])
{
   double start;

   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return 1;
   }

   /* Set GPIO modes */
   gpioSetMode(12, PI_OUTPUT);
   gpioSetMode(21, PI_OUTPUT);
   gpioSetMode(20, PI_OUTPUT);
   gpioSetMode(23, PI_OUTPUT);
   gpioSetMode(24, PI_OUTPUT);
   gpioSetMode(13, PI_OUTPUT);

   gpioWrite(12, 0); /* on */
   gpioWrite(13, 0); /* on */

   /*
   start = time_time();

   while ((time_time() - start) < 60.0)
   {
      // motor on
      gpioWrite(12, 1);
      gpioWrite(13, 1);

      // forward
      gpioWrite(21, 1);
      gpioWrite(20, 0);
      gpioWrite(23, 1);
      gpioWrite(24, 0);

      time_sleep(3.5);

      // backward
      gpioWrite(21, 0);
      gpioWrite(20, 1);
      gpioWrite(23, 0);
      gpioWrite(24, 1);

      time_sleep(3.5);

      // motor stop
      gpioWrite(12, 0);
      gpioWrite(13, 0)

      time_sleep(3.5);
   }
   */

   start = time_time();

   while ((time_time() - start) < 60.0)
   {
      int x = (int)(time_time() - start)*3+75;
      gpioPWM(12, 0);
      gpioPWM(13, 0);

      // forward
      gpioWrite(21, 1);
      gpioWrite(20, 0);
      gpioWrite(23, 1);
      gpioWrite(24, 0);

      gpioPWM(12, x);
      gpioPWM(13, x);
      time_sleep(1);
   }

   // stop
   gpioWrite(12, 0);
   gpioWrite(13, 0);




   /* Stop DMA, release resources */
   gpioTerminate();

   return 0;
}
```


```
 mpu9250_calibration.py
import os
import sys
import time
import smbus
import time

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
#imu.setAccelRange("AccelRangeSelect2G")
#imu.setGyroRange("GyroRangeSelect250DPS")

#print("Gyro calibration starting")
#imu.caliberateGyro()
#print("Gyro calibration finished")
#print('gyro bias')
#print(imu.GyroBias)

#print("Accel calibration starting")
#imu.caliberateAccelerometer()
#print("Accel calibration finished")
#print('accel scale')
#print(imu.Accels)
#print('accel bias')
#print(imu.AccelBias)

print("Mag calibration starting")
time.sleep(2)
imu.caliberateMagPrecise()
print("Mag calibration finished")
print('mag transformation matrix')
print(imu.Magtransform)
print('mag bias')
print(imu.MagBias)
print('mags')
print (imu.Mags)

imu.saveCalibDataToFile("/home/ubuntu/calib.json")
# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

while True:
    imu.readSensor()
    imu.computeOrientation()

    print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
    time.sleep(0.1)
```

https://github.com/Schnilz/mpu9250


rplidar a1m8

https://github.com/Slamtec/sllidar_ros2.git



----
```
drwxrwxr-x 11 donghee donghee 4096 Jul 26 21:44 .
drwxr-xr-x 57 donghee donghee 4096 Jul 26 00:02 ..
drwxrwxr-x  5 donghee donghee 4096 Jul 24 22:54 build
drwxrwxr-x  5 donghee donghee 4096 Jul 24 22:54 install
drwxrwxr-x  6 donghee donghee 4096 Jul 24 22:59 log
drwxrwxr-x  7 donghee donghee 4096 Jul 26 21:44 mpu9250
-rw-rw-r--  1 donghee donghee  993 Jul 25 02:42 run-tank.sh
drwxrwxr-x  8 donghee donghee 4096 Jul 26 21:44 sllidar_ros2
drwxrwxr-x  7 donghee donghee 4096 Jul 24 22:53 tank_cartographer
drwxrwxr-x  5 donghee donghee 4096 Jul 26 21:44 tank_control
drwxrwxr-x  8 donghee donghee 4096 Jul 24 22:54 tank_navigation2
```
