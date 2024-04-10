# 安装rosbridge
```
sudo apt-get install ros-<rosdistro>-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch
```
# 其他命令
```
roslaunch ouster_ros driver.launch sensor_hostname:=192.168.100.2
roslaunch fixposition_driver_ros1 tcp.launch
roslaunch slope_calculation slope_calculation.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```

## 雷达配置
### 查找雷达ip
```
avahi-browse -lr _roger._tcp
```
### 设置雷达ip
```
sudo apt install -y httpie

echo \"目标.设置.静态.IP/24\" | http PUT http://雷达.当前.IP.地址]/api/v1/system/network/ipv4/override/

echo \"192.168.100.2/24\" | http PUT http://192.168.2.13/api/v1/system/network/ipv4/override/

```