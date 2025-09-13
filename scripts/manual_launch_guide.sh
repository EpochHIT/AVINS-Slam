#!/bin/bash

# 简化的 VINS-Surfel 启动脚本
echo "=== 简化启动脚本 ==="

# 设置环境
cd ~/AVINS-Slam/
source devel/setup.bash

echo "请按顺序在不同终端运行以下命令："
echo ""
echo "终端 1: 启动相机"
echo "roslaunch vins vins_camera.launch"
echo ""
echo "终端 2: 启动时间戳同步"
echo "roslaunch vins vins_surfel_timestamp_sync.launch"
echo ""
echo "终端 3: 启动 VINS"
echo "rosrun vins vins_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml"
echo ""
echo "终端 4: 启动回环检测"
echo "rosrun loop_fusion loop_fusion_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml"
echo ""
echo "终端 5: 启动稠密建图"
echo "rosrun surfel_fusion surfel_fusion __ns:=surfel_fusion _cam_fx:=600 _cam_fy:=600 _cam_cx:=320 _cam_cy:=240 _cam_width:=640 _cam_height:=480 _fuse_far_distence:=5.0 _fuse_near_distence:=0.2 _drift_free_poses:=15 image:=/camera/infra1/image_rect_raw_sync depth:=/camera/aligned_depth_to_color/image_raw_sync color:=/camera/color/image_raw_sync loop_path:=/loop_fusion/pose_graph_path extrinsic_pose:=/vins_estimator/extrinsic"
echo ""
echo "=== 关键检查命令 ==="
echo "检查相机数据: rostopic hz /camera/imu"
echo "检查同步数据: rostopic hz /camera/imu_sync"
echo "检查VINS输出: rostopic hz /vins_estimator/odometry"
echo "检查建图输出: rostopic hz /surfel_fusion/pointcloud"
