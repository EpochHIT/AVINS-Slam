#!/bin/bash

# VINS-Surfel 系统启动脚本
# 按正确的顺序启动各个组件以确保时间戳同步

echo "=== VINS-Surfel 系统启动脚本 ==="
echo "正在启动 VINS-Fusion + DenseSurfelMapping 时间戳同步系统..."

# 检查是否有 RealSense 摄像头
if ! lsusb | grep -q "Intel Corp"; then
    echo "警告: 未检测到 Intel RealSense 设备，请确保摄像头已连接"
fi

# 步骤 1: 启动 RealSense 驱动
echo "步骤 1: 启动 RealSense D435i 驱动..."
cd ~/AVINS-Slam/
. devel/setup.bash
roslaunch vins vins_camera.launch &
REALSENSE_PID=$!
sleep 3

# 检查 RealSense 是否正常启动
if ! rostopic list | grep -q "/camera/imu"; then
    echo "错误: RealSense 驱动启动失败"
    exit 1
fi

echo "RealSense 启动成功，检测到以下话题:"
rostopic list | grep "/camera/"

# 步骤 2: 启动时间戳同步节点
echo "步骤 2: 启动 VINS-Surfel 时间戳同步节点..."
roslaunch vins vins_surfel_timestamp_sync.launch &
SYNC_PID=$!
sleep 2

# 步骤 3: 启动 VINS 估计器
echo "步骤 3: 启动 VINS 估计器..."
rosrun vins vins_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml &
VINS_PID=$!
sleep 5

# 检查 VINS 是否正常启动
if ! rostopic list | grep -q "/vins_estimator/odometry"; then
    echo "错误: VINS 估计器启动失败"
    cleanup_and_exit
fi

# 步骤 4: 启动 Loop Fusion（可选）
echo "步骤 4: 启动 Loop Fusion..."
rosrun loop_fusion loop_fusion_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml &
LOOP_PID=$!
sleep 2

# 步骤 5: 启动 Surfel Fusion
echo "步骤 5: 启动 Dense Surfel Mapping..."
roslaunch surfel_fusion vins_realsense_synchronized.launch &
SURFEL_PID=$!

echo "=== 系统启动完成 ==="
echo "所有组件已启动，系统正在运行..."
echo ""
echo "关键话题监控:"
echo "- VINS 里程计: /vins_estimator/odometry"
echo "- 同步深度图: /camera/aligned_depth_to_color/image_raw_sync"  
echo "- 同步红外图: /camera/infra1/image_rect_raw_sync"
echo "- 同步IMU: /camera/imu_sync"
echo "- 位姿图路径: /loop_fusion/pose_graph_path"
echo ""
echo "按 Ctrl+C 停止所有节点..."

# 清理函数
cleanup_and_exit() {
    echo "正在关闭所有节点..."
    if [ ! -z "$SURFEL_PID" ]; then kill $SURFEL_PID 2>/dev/null; fi
    if [ ! -z "$LOOP_PID" ]; then kill $LOOP_PID 2>/dev/null; fi
    if [ ! -z "$VINS_PID" ]; then kill $VINS_PID 2>/dev/null; fi
    if [ ! -z "$SYNC_PID" ]; then kill $SYNC_PID 2>/dev/null; fi
    if [ ! -z "$REALSENSE_PID" ]; then kill $REALSENSE_PID 2>/dev/null; fi
    
    # 强制清理所有相关进程
    pkill -f "realsense"
    pkill -f "vins_node"
    pkill -f "loop_fusion"
    pkill -f "surfel_fusion"
    pkill -f "timestamp_synchronizer"
    
    echo "所有节点已关闭"
    exit 0
}

# 捕获 Ctrl+C 信号
trap cleanup_and_exit SIGINT

# 保持脚本运行
while true; do
    sleep 1
done
