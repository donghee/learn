

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
