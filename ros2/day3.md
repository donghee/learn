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
