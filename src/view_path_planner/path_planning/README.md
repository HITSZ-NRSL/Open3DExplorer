# path_planning
ROS版本：kinetic或melodic

## 依赖库安装(kinetic可替换为melodic)
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server

## 代码编译

```
新建工作空间:
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin build

下载代码:
cd src
git clone https://github.com/nixwang/view_path_planner.git
cd view_path_planner
git checkout "各自的分支"
cd ../../

编译代码:
catkin build
```
## 代码启动

1. 先启动launch文件
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch path_planning path_planning.launch
```


2. 启动ros包
```
cd ~/catkin_ws/src/view_path_planner
rosbag play out_test.bag --clock
```
