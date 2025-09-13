# VINS-Surfel 时间戳同步系统使用指南

## 概述

本系统实现了 VINS-Fusion 与 DenseSurfelMapping 之间的精确时间戳同步，确保位姿估计与传感器数据（深度图、红外图像、IMU）的时间戳完全一致。

## 系统架构

```
RealSense D435i → 时间戳同步节点 → VINS + Surfel Fusion
     ↓                 ↓                    ↓
原始传感器数据    时间戳对齐处理         同步后的建图
```

### 关键组件

1. **vins_surfel_timestamp_synchronizer**: 核心同步节点
2. **VINS-Fusion**: 视觉-惯性里程计
3. **Loop Fusion**: 回环检测与位姿图优化
4. **Surfel Fusion**: 稠密表面重建

## 启动前准备

### 1. 硬件检查
```bash
# 检查 RealSense 设备
lsusb | grep "Intel Corp"

# 检查 USB 端口
dmesg | grep -i realsense
```

### 2. 环境设置
```bash
# 确保 ROS 环境已加载
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 编译项目
cd /home/cwkj/AVINS-Slam
catkin_make
```

## 启动方式

### 方式一：一键启动脚本（推荐）
```bash
cd /home/cwkj/AVINS-Slam
./scripts/launch_vins_surfel_system.sh
```

### 方式二：分步启动

#### 步骤 1: 启动 RealSense 相机
```bash
roslaunch vins vins_camera.launch
```

**重要参数说明：**
- `enable_infra=true`: 启用红外流（必须）
- `enable_depth=true`: 启用深度流
- `enable_color=false`: 可关闭彩色流以节省带宽
- `infra_fps=30`: 红外帧率
- `gyro_fps=200`: 陀螺仪频率
- `accel_fps=250`: 加速计频率

#### 步骤 2: 启动时间戳同步节点
```bash
roslaunch vins vins_surfel_timestamp_sync.launch
```

**关键参数：**
- `sync_tolerance=0.01`: 图像同步容错（10ms）
- `imu_sync_tolerance=0.005`: IMU 同步容错（5ms）
- `enable_color=false`: 可关闭彩色图同步以节省资源
- `enable_depth=true`: 启用深度图同步
- `enable_infra=true`: 启用红外图同步

#### 步骤 3: 启动 VINS 估计器
```bash
roslaunch vins realsense_stereo_imu_vins.launch
```

**注意：** VINS 配置文件中的话题已重映射到同步后的版本：
- `/camera/infra1/image_rect_raw_sync`
- `/camera/infra2/image_rect_raw_sync`  
- `/camera/imu_sync`

#### 步骤 4: 启动回环检测（可选）
```bash
roslaunch loop_fusion realsense_loop.launch
```

#### 步骤 5: 启动稠密建图
```bash
roslaunch surfel_fusion vins_realsense_synchronized.launch
```

## 关键话题监控

### 输入话题（原始数据）
```bash
# 检查原始传感器数据
rostopic hz /camera/infra1/image_rect_raw
rostopic hz /camera/infra2/image_rect_raw  
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic hz /camera/imu
```

### 同步话题（处理后）
```bash
# 检查同步后的数据
rostopic hz /camera/infra1/image_rect_raw_sync
rostopic hz /camera/infra2/image_rect_raw_sync
rostopic hz /camera/aligned_depth_to_color/image_raw_sync
rostopic hz /camera/imu_sync
```

### VINS 输出
```bash
# VINS 里程计
rostopic hz /vins_estimator/odometry

# 外参信息
rostopic echo /vins_estimator/extrinsic

# 位姿图路径（回环后）
rostopic hz /loop_fusion/pose_graph_path
```

### Surfel 输出
```bash
# 稠密点云
rostopic hz /surfel_fusion/pointcloud

# 位姿发布
rostopic hz /surfel_fusion/cam_pose
```

## 时间戳同步机制

### 同步策略
1. **以 VINS 里程计时间戳为基准**：`/vins_estimator/odometry`
2. **查找最接近的传感器数据**：在容错范围内匹配
3. **重新设置时间戳**：统一到 VINS 时间戳
4. **发布同步数据**：带 `_sync` 后缀的话题

### 同步容错
- **图像数据**：10ms 容错（`sync_tolerance=0.01`）
- **IMU 数据**：5ms 容错（`imu_sync_tolerance=0.005`）

### Surfel Fusion 匹配
- **严格匹配**：1ms 容错（假设已预同步）
- **缓冲区管理**：自动清理过期数据

## 故障排除

### 1. 红外图像无输出
```bash
# 检查参数设置
rosparam get /camera/enable_infra
rosparam get /camera/enable_infra1  
rosparam get /camera/enable_infra2

# 如果为 false，设置为 true
rosparam set /camera/enable_infra true

# 重启相机节点
rosnode kill /camera/realsense2_camera
roslaunch vins vins_camera.launch
```

### 2. 时间戳不同步
```bash
# 检查同步节点状态
rosnode info /vins_surfel_timestamp_synchronizer

# 查看同步统计
rostopic echo /vins_surfel_timestamp_synchronizer/sync_stats
```

### 3. VINS 初始化失败
```bash
# 检查 IMU 数据
rostopic hz /camera/imu_sync

# 检查红外图像质量
rosrun image_view image_view image:=/camera/infra1/image_rect_raw_sync

# 确保有足够的特征点
rosrun image_view image_view image:=/vins_estimator/feature_img
```

### 4. Surfel Fusion 无输出
```bash
# 检查外参是否初始化
rostopic echo /vins_estimator/extrinsic

# 检查位姿图路径
rostopic hz /loop_fusion/pose_graph_path

# 检查深度图数据
rostopic hz /camera/aligned_depth_to_color/image_raw_sync
```

## 性能优化建议

### 1. 带宽优化
- 关闭不必要的彩色流：`enable_color=false`
- 降低帧率：`infra_fps=15`（如果可接受）
- 使用压缩传输：启用 `compressed` 话题

### 2. CPU 优化  
- 调整缓冲区大小：`max_buffer_size=100`
- 降低同步频率：适当增加 `sync_tolerance`

### 3. 内存优化
- 定期清理旧数据：自动清理 2 秒前的数据
- 限制缓冲区大小：防止内存泄漏

## 配置文件路径

- **VINS 配置**: `/home/cwkj/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml`
- **相机启动**: `/home/cwkj/AVINS-Slam/src/VINS-Fusion/vins_estimator/launch/vins_camera.launch`  
- **同步启动**: `/home/cwkj/AVINS-Slam/src/VINS-Fusion/vins_estimator/launch/vins_surfel_timestamp_sync.launch`
- **系统启动**: `/home/cwkj/AVINS-Slam/src/DenseSurfelMapping/surfel_fusion/launch/vins_realsense_synchronized.launch`

## 日志与调试

### 启用调试信息
```bash
# 设置 ROS 日志级别
export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}'
roscore &
rosparam set /rosdistro/log_level DEBUG
```

### 关键日志信息
- **同步成功**: "Synchronized at VINS time: xxx"
- **匹配失败**: "No matching synchronized data found"
- **外参初始化**: "receive extrinsic pose"
- **建图状态**: "fuse map begins/done"
