# Open3DExplorer

# Whole-Body Motion Planning and Tracking of a Mobile Robot with a Gimbal RGB-D Camera for Outdoor 3D Exploration

## 1. Introduction
This work **Open3DExplorer** is oriented to the rapid autonomous reconstruction task of mobile robots, and for the problems of the field environment, such as openness, uneven texture distribution and rugged terrain, which seriously affect the efficiency and robustness of the autonomous reconstruction of robots, we use the active gimbal and the multi-layer map information to plan the motion of the robot and the camera, and realize an efficient and robust autonomous reconstruction scheme. 

Authors: [Zhihao Wang](https://github.com/nixwang), [Haoyao Chen](https://github.com/HitszChen) from the [Networked RObotics and Sytems Lab](http://www.nrs-lab.com), and Mengmeng Fu.

If you use **Open3DExplorer** for your academic research, please cite the following paper [[pdf](https://ieeexplore.ieee.org/document/10129820)]. 
```
@ARTICLE{wang_whole_2023,
  author={Wang, Zhihao and Chen, Haoyao and Fu, Mengmeng},
  journal={Journal of Field Robotics}, 
  title={Whole-Body Motion Planning and Tracking of a Mobile Robot with a Gimbal RGB-D Camera for Outdoor 3D Exploration}, 
  year={2023},
  volume={},
  number={},
  pages={},
  doi={10.1002/rob.22281}}
```

### 1.1. Related Video:

<div align="center">
    <a href="https://youtu.be/zhcfmEk1JnI" target="blank">
    <img src="doc/image/Framework.png" width=100% />
    </a>
</div>


**Video links:** [Youtube](https://youtu.be/zhcfmEk1JnI) or [Bilibili](https://www.bilibili.com/video/BV1co4y1w7cQ).


## 2. Installation
Tested on Ubuntu 18.04.

- Install ROS packages
```
sudo apt-get install ros-$(rosversion)-octomap-ros ros-$(rosversion)-octomap-msgs ros-$(rosversion)-yocs-cmd-vel-mux ros-$(rosversion)-pcl-ros
```
- Create a conda environment {YOUR_ENV_NAME} with python3 (python3.6 in my computer) and install below packages, the version of these packages are adapted to yourself cuda version.
```
conda install cudatoolkit cudnn pytorch torchvision torchaudio pyyaml opencv-python opencv-python-headless scikit-image protobuf segmentation_models_pytorch albumentations 
```
- Download code
```
cd {YOUR_PATH}
git clone https://github.com/HITSZ-NRSL/Open3DExplorer.git
cd Open3DExplorer
catkin build
```
- Install ORB-SLAM
```
cd Open3DExplorer/src/semantic_slam/ORB_SLAM2
./build.sh
```
If there is any problems in installing ORB-SLAM, you can refer to [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

## 3. Run examples

**Simulator**
```
roslaunch gazebo_continuous_track_example gazebo_tracked_vehicle.launch
```

**SLAM**
```
roslaunch semantic_slam slam_gazebo.launch
```


**Semantic Mapping**
```
conda activate {YOUR_ENV_NAME}
roslaunch semantic_slam semantic_mapping_py3_gazebo.launch
```

**Planning**

```bash
roslaunch path_planning path_planning_plane.launch
```

**Tracking**

```bash
roslaunch mpc_tracker mpc_test.launch
```


## 4. Acknowledgments
 Semantic SLAM in **Open3DExplorer** is based on [Semantic SLAM](https://github.com/floatlazer/semantic_slam).

## LICENSE
The source code is released under GPLv2 license.
