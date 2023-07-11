diffdrive-arduino

필요한 패키지
ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-hardware-interface ros-foxy-xacro

---

icra2023 ros2 gz tutorial

- https://drive.google.com/drive/folders/1HeveECTM4RTbKhFgO0gieUqawBCQKZf0
- https://github.com/osrf/icra2023_ros2_gz_tutorial#quick-start
---

### 테스트


시뮬레이터 launch 실행 (rsp, gazebo, gazebo_ros 실행)

```
ros2 launch firstbot_description launch_sim.launch.py use_sim_time:=true
```

키보드 teleop

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

cartographer
```
ros2 launch firstbot_cartographer cartographer_rviz.launch.py use_sim_time:=True
```

save map
```
ros2 run nav2_map_server map_saver_cli -f map
```

navigation2
```
ros2 launch firstbot_navigation2 navigation2.launch.py map:=$HOME/map.yaml use_sim_time:=True
```



