# direct_visual_lidar_calibration

该软件包提供了一个用于 LiDAR-相机外参标定的工具箱，具有以下特点：

- **通用性强**：可处理多种 LiDAR 和相机投影模型，包括旋转式和非重复扫描式 LiDAR，以及针孔、鱼眼和全景投影相机。
- **无需靶标**：无需标定板，利用环境的结构和纹理即可完成标定。
- **单帧标定**：最少仅需一对 LiDAR 点云与相机图像即可完成标定。可选地，使用多对 LiDAR-相机数据可提高精度。
- **全自动**：标定过程全自动进行，无需初始估计。
- **高精度与高鲁棒性**：采用像素级的直接 LiDAR-相机配准算法，相比基于边缘的间接配准方法更加鲁棒且精确。

**文档链接：[https://koide3.github.io/direct_visual_lidar_calibration/](https://koide3.github.io/direct_visual_lidar_calibration/)**  
**Docker 镜像：[koide3/direct_visual_lidar_calibration](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)**

[![构建状态](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml/badge.svg)](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml) [![Docker 镜像大小（按日期最新）](https://img.shields.io/docker/image-size/koide3/direct_visual_lidar_calibration)](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)

![213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6](https://user-images.githubusercontent.com/31344317/213427328-ddf72a71-9aeb-42e8-86a5-9c2ae19890e3.jpg)

[视频链接](https://www.youtube.com/watch?v=7TM7wGthinc&feature=youtu.be)

## 依赖项

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)（可选）

## 入门指南

1. [安装](https://koide3.github.io/direct_visual_lidar_calibration/installation/) / [Docker 镜像](https://koide3.github.io/direct_visual_lidar_calibration/docker/)
2. [数据采集](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
3. [标定示例](https://koide3.github.io/direct_visual_lidar_calibration/example/)
4. [程序说明](https://koide3.github.io/direct_visual_lidar_calibration/programs/)
5. [安装踩坑教程1](https://blog.csdn.net/qq_56719965/article/details/140737210?ops_request_misc=%257B%2522request%255Fid%2522%253A%252284435da1214495222ae0d1cccdc9f2c7%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=84435da1214495222ae0d1cccdc9f2c7&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-140737210-null-null.142%5Ev101%5Epc_search_result_base5&utm_term=direct%20visual%20lidar%20calib&spm=1018.2226.3001.4187)
[安装踩坑教程2](https://gitee.com/gwmunan/ros2/wikis/%E5%AE%9E%E6%88%98%E6%95%99%E7%A8%8B/%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%81%94%E5%90%88%E6%A0%87%E5%AE%9A--direct_visual_lidar_calibration#%E5%AE%89%E8%A3%85)
[安装踩坑教程3](https://blog.csdn.net/qq_44804057/article/details/141222937)

## 环境安装

### 基础环境

ubuntu20.04 ros-noetic

### 安装依赖

```bash
sudo apt install libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev
```

### 安装GTSM

```bash
# Install GTSAM
 
git clone https://github.com/borglab/gtsam
 
cd gtsam && git checkout 4.2a9
 
mkdir build && cd build
 
# For Ubuntu 22.04, add -DGTSAM_USE_SYSTEM_EIGEN=ON
 
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
 
         -DGTSAM_BUILD_TESTS=OFF \
 
         -DGTSAM_WITH_TBB=OFF \
 
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
 
make -j$(nproc)
 
sudo make install
```

### 安装FMT8.1.1

**安装必要依赖**：

```bash
sudo apt update
sudo apt install cmake g++ -y
```

**下载源码**：

```bash
wget https://github.com/fmtlib/fmt/archive/refs/tags/8.1.1.tar.gz
tar -xzf 8.1.1.tar.gz
cd fmt-8.1.1
```

**编译安装**：

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 安装Ceres2.2.0

**安装依赖**：

```bash
sudo apt update
sudo apt install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
```

**下载源码**：

```bash
wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz
tar -xzf 2.2.0.tar.gz
cd ceres-solver-2.2.0
```

**编译安装**：

```bash
mkdir build && cd build

cmake .. \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF

make -j$(nproc)
sudo make install
```

### 安装Iridescence

```bash
# Install Iridescence for visualization
 
git clone https://github.com/koide3/iridescence --recursive
 
mkdir iridescence/build && cd iridescence/build
 
cmake .. -DCMAKE_BUILD_TYPE=Release
 
make -j$(nproc)
 
sudo make install
```

编译direct_visual_lidar_calib时出现错误，踩坑教程中有方案

### 开始标定

#### 录制bag文件

录制信息包含图像信息和点云信息，使用mid360驱动livox_ros_driver2注意修改点云格式

```bash
<arg name="xfer_format" default="0"/>
# 设置点云格式
# 0 – 览沃 pointcloud2(PointXYZRTL) 点云格式
# 1 – 览沃自定义点云数据格式
# 2 – PCL库中标准 pointcloud2(pcl::PointXYZI) 点云格式
```

发布出的/livox/lidar点云话题为sensor_msgs/PointCloud2
如果提前录制的bag文件点云话题为livox_ros_driver/CustomMsg，可以使用bag_converter进行转换

**bag文件预处理**：

```bash
rosrun direct_visual_lidar_calibration preprocess   -av   --camera_model=plumb_bob   --camera_intrinsics=658.62796,657.8905,315.19865,248.34832   --camera_distortion_coeffs=-0.092001,0.163035,-0.000991,0.000341,0.0   /home/sun/bag   /home/sun/pre
```

**相机标定参数**：
--camera_model=plumb_bob   --camera_intrinsics=658.62796,657.8905,315.19865,248.34832   --camera_distortion_coeffs=-0.092001,0.163035,-0.000991,0.000341,0.0

**bag文件路径**：
/home/sun/bag

**产生的文件路径**：
/home/sun/pre

#### 标定

**确定初始坐标(手动)**：
```bash
rosrun direct_visual_lidar_calibration initial_guess_manual /home/sun/pre
```

**优化标定结果**：
```bash
rosrun direct_visual_lidar_calibration calibrate /home/sun/pre
```

**参看标定效果**：
```bash
rosrun direct_visual_lidar_calibration viewer /home/sun/pre
```

标定结果保存在calib.json格式为TUM [tx ty tz qx qy qz qw] 即：【X轴平移、y轴平移、z轴平移、四元数_x、四元数_y、四元数_z、四元数_w】

### 利用转换工具转换为需要的格式

[转换工具](https://gitee.com/link?target=https%3A%2F%2Fstaff.aist.go.jp%2Fk.koide%2Fworkspace%2Fmatrix_converter%2Fmatrix_converter.html)

[使用教程](https://github.com/hku-mars/FAST-LIVO2/issues/119#issuecomment-2726690148)

## 功能包bag_converter使用

**主要功能**：将bag文件提取图像点云(png、pcd、ply)格式，将bag文件中的livox自定义话题格式转换为ros默认格式

### 依赖livox_ros_driver2

将CMakeLists.txt中的ROS2部分删除，避免编译出错

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

### 图片点云提取

```bash
rosrun bag_converter bag_converter_node /path/to/file.bag /camera/image /livox/lidar ~/output_dir
```

第1个参数：.bag 文件路径（绝对或相对）
第2个参数：图像话题名，例如 /camera/image
第3个参数：Livox 点云话题名，例如 /livox/lidar
第4个参数：输出保存目录，例如 ~/output_dir

### 点云格式转换

**运行**：
```bash
rosrun bag_converter livox_to_pointcloud2_node
```

**播放你的bag文件**：
```bash
rosbag play your_livox.bag
```

你会在 /livox/pointcloud2 话题收到转换后的 sensor_msgs::PointCloud2 消息

**录制bag文件**：
```bash
rosbag record camera_topic lidar_topic
```
