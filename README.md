# 功能介绍

该功能包通过接收IMU、里程计、Tof摄像头的消息，进行建图与定位

# 使用方法

## 准备工作

1. 具备完整的硬件设备，包含IMU，Tof摄像头，带里程计的底盘，并且能够正常运行。

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，打开终端拉取相应代码并编译安装

```bash
#安装依赖的功能包
sudo apt -y tros-hobot-nav2 libsuitesparse-dev libblas-dev liblapack-dev libeigen3-dev libopencv-dev libspdlog-dev libconsole-bridge-dev libpcl-dev libgoogle-glog-dev libtf2-dev  ros-humble-cv-bridge  ros-humble-nav-msgs ros-humble-image-transport ros-humble-tf2-ros ros-humble-pcl-conversions ros-humble-navigation2 libceres-dev

#编译g2o功能包（功能包见文末网盘）
cd g2o
mkdir build && cd build
cmake ..
make
make install

#拉取代码编译
mkdir -p ~/tofslam_ws/src && cd ~/tofslam_ws/src
git clone https://github.com/wunuo1/Tofslam.git -b humble
cd ..
colcon build
```

**2.运行tofslam**

```shell
source ~/tofslam_ws/install/setup.bash
ros2 run ct_lio ct_lio_eskf
```

## 参数介绍
修改config文件夹下mapping.yaml文件，关注带有说明的参数，output文件从文末附件中获取：
```yaml
YAML: 1.0

preprocess:
  point_filter_num: 3  #点云下采样
  lidar_type: 7  # 1-AVIA 2-velodyne 3-ouster  4-robosense 5-pandar 6-dtof 7-mtof
  blind: 0.01


common:
  imu_topic: /imu #imu消息名称
  lid_topic: /nebula/mtof_points2 #点云或雷达消息名称
  odom_topic: /odom  #里程计消息
  img_topic: /camera/color/image_raw/compressed #rgb图像消息，可使用压缩消息也可用原图
  Localization_mode: False #是否进行定位，否则为建图
  output_path: /home/slam_ws_2/output #存放建图产物的路径
  model_path: /home/slam_ws_2/output/hfnet.onnx #指定hfnet模型路径
  imuhz: 400 #imu的频率
 
mapping:
  extrinsic_est_en: true
  #tof到imu的变化，获取方式请查看标定说明
  tof2imu_extrinsic_T: [ -0.0039799203111821468, -0.027189542185995823831, -0.14001955020902342461]
  tof2imu_extrinsic_R: [-0.993063627638373553564 ,0.052518963674774226572 ,-0.01498737732389914978, 
                        -0.033814436045461613632 ,-0.016796566829113524293 ,-0.99928717438014507495, 
                        -0.00801808196977665167 ,-0.99847864521832871386  ,0.03465425204866725379]
  
  #bask_link到imu的变化，获取方式请查看标定说明
  robot2imu_extrinsic_T: [0.12588064308116462841, 0.008162559660163853708, 0.20705414746130172184]
  robot2imu_extrinsic_R: [0.052268869631951098023, -0.93852017660344945467, 0.033846560965168889575, 
                          0.93808656435923075909, -0.051737213472478319612, 0.015014368436287678477, 
                          0.033019704102458273174        , 0.01675474093936925155, 0.99931446977489903968]


delay_time: 0.3

odometry:
  wheelmode: 1 #0-relativepose 1-velocity
  max_trans_diff: 0.1
  max_drift_num: 15
  surf_res: 0.1 #0.4
  log_print: true
  max_num_iteration: 5
  # ct_icp
  icpmodel: CT_POINT_TO_PLANE  # CT_POINT_TO_PLANE  #CT_POINT_TO_PLANE                    # Options: [CT_POINT_TO_PLANE, POINT_TO_PLANE]
  size_voxel_map: 0.1 #0.4                         # The voxel size of in the voxel map
  min_distance_points: 0.05
  max_num_points_in_voxel: 20                 # The maximum number of points per voxel of the map
  max_distance: 50.0                        # The threshold of the distance to suppress voxels from the map
  weight_alpha: 0.9
  weight_neighborhood: 0.1
  max_dist_to_plane_icp: 0.1 #0.3
  init_num_frames: 20
  voxel_neighborhood: 1
  max_number_neighbors: 20
  threshold_voxel_occupancy: 1
  estimate_normal_from_neighborhood: true
  min_number_neighbors: 20                    # The minimum number of neighbor points to define a valid neighborhood
  power_planarity: 2.0
  num_closest_neighbors: 1

  sampling_rate: 1.0
  ratio_of_nonground: 2
  # max_num_residuals: 1000
  max_num_residuals: 2000
  min_num_residuals: 100
  motion_compensation: CONSTANT_VELOCITY #NONE #CONSTANT_VELOCITY  #CONTINUOUS #NONE, CONSTANT_VELOCITY, ITERATIVE, CONTINUOUS
  beta_location_consistency: 1.0
  beta_orientation_consistency: 1.0
  beta_constant_velocity: 1.0
  beta_small_velocity: 0.0

  thres_translation_norm: 0.03
  thres_orientation_norm: 0.05

  use_ground_constraint: 0

#以下为定位功能参数
reloc:
  mapfile: /home/slam_ws_2/output/final-voxel.pcd #pcd点云地图路径，建图完成的产物
  reloc_mode: 1  #0-scancontext 1-hfnet  2-no   #重定位检测方式，0为点云检测，1为特征点检测
  scancontext_dbfile: /home/slam_ws_2/output/ScanContext.bin #点云检测模型路径
  hfnet_dbfile: /home/slam_ws_2/output/HFNet.bin #特征点检测模型路径
  icp_threshold: 0.03
```