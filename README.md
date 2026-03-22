# MMS-SLAM
## Multi-modal semantic SLAM in dynamic environments 

## 1. Solid-State Lidar Sensor Example
### 1.1 Scene Reconstruction in Dynamic Environments
<p align='center'>
<a href="https://youtu.be/tmWCrredJGI">
<img width="65%" src="/img/3DConstruction.gif"/>
</a>
</p>

### 1.2 Mapping result
<p align='center'>
<img width="65%" src="/img/mapconstruction.png"/>
</p>

### 1.3 Human & AGV recognition result
<p align='center'>
<img width="65%" src="/img/agv_human_detection.png"/>
</a>
</p>

### 1.4 Performance Evaluation
<p align='center'>
<img width="65%" src="/img/result_comparison.png"/>
</a>
</p>

### 1.5 Detection Result
<p align='center'>
<img width="65%" src="/img/solo_result.png"/>
</a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.

ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4 **OctoMap**
Follow [OctoMap Installation](http://wiki.ros.org/octomap).

```bash
$ sudo apt install ros-noetic-octomap*
```

### 2.5. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-noetic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed


## 3. Build 
### 3.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/mms_slam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
chmod python file 
```
roscd mms_slam
cd src
chmod +x solo_node.py
```

### 3.2 install mmdetection 
create conda environment (you need to install [conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) first) 
```
conda create -n solo python=3.7 -y
conda activate solo
```

install PyTorch and torchvision following the [official instruction](https://pytorch.org/get-started/previous-versions/) (find your cuda version)
```
conda install pytorch==1.7.1 torchvision==0.8.2 torchaudio==0.7.2 cudatoolkit=11.0 -c pytorch
conda install -c conda-forge addict rospkg pycocotools
```
install mmdet 2.0
```
roscd mms_slam 
cd dependencies/mmdet
python setup.py install
```
it takes a while (a few minutes to install)

### 3.3 Download test rosbag and model
You may download our [trained model](https://drive.google.com/file/d/10ZwHyT7Ql1DYofe4p1jCEAj4rEfg499J/view?usp=sharing) and [recorded data](https://drive.google.com/file/d/1XX4M-aB5aFtj7gPMJKVAeRICdEEAT-EG/view?usp=sharing) if you dont have realsense L515, and by defult the file should be under /home/username/Downloads

put model under mms_slam/config/  
```
cp ~/Downloads/trained_model.pth ~/catkin_ws/src/MMS_SLAM/config/
```
unzip rosbag file under Download folder
```
cd ~/Downloads
unzip ~/Downloads/dynamic_warehouse.zip
```

### 3.4 Launch ROS
if you would like to create the map at the same time, you can run 
```
    roslaunch mms_slam mms_slam_mapping.launch
```

if only localization is required, you may refer to run
```
    roslaunch mms_slam mms_slam.launch
```

if you would like to test instance segmentation results only , you can run
```
    roslaunch mms_slam mms_slam_detection.launch
```

if ModuleNotFoundError: No module named 'alfred', install alfrey-py from pip install
```
pip install alfred-py
```


## 7 Acknowlegement
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [LOAM](https://github.com/laboshinl/loam_velodyne) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED) and [MMDetection](https://github.com/open-mmlab/mmdetection) and [SOLO](https://github.com/WXinlong/SOLO).

