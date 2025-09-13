#!/bin/bash

echo "AVINS-SLAM 渐进式测试脚本"
echo "================================"

# 停止所有进程
echo "停止所有现有进程..."
killall -9 roslaunch rosrun roscore 2>/dev/null
sleep 3

# 启动roscore
echo "启动roscore..."
roscore &
sleep 3

# 测试1: 仅红外
echo "测试1: 仅红外图像..."
echo "启动相机（仅红外模式）..."
roslaunch vins vins_camera_ir_only.launch &
sleep 10

echo "检查红外图像流..."
timeout 5 rostopic hz /camera/infra1/image_rect_raw
if [ $? -eq 0 ]; then
    echo "✓ 红外图像1 - 成功"
else
    echo "✗ 红外图像1 - 失败"
    echo "停止测试，请检查硬件连接"
    killall -9 roslaunch 2>/dev/null
    exit 1
fi

timeout 5 rostopic hz /camera/infra2/image_rect_raw
if [ $? -eq 0 ]; then
    echo "✓ 红外图像2 - 成功"
else
    echo "✗ 红外图像2 - 失败"
fi

timeout 5 rostopic hz /camera/imu
if [ $? -eq 0 ]; then
    echo "✓ IMU数据 - 成功"
else
    echo "✗ IMU数据 - 失败"
fi

echo ""
echo "红外模式测试完成！"
echo "如果成功，可以尝试启动VINS:"
echo "rosrun vins vins_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml"
echo ""
echo "按任意键继续测试DenseSurfelMapping所需的RGB+深度流，或Ctrl+C退出..."
read

# 停止仅红外模式
echo "停止仅红外模式..."
killall -9 roslaunch 2>/dev/null
sleep 3

# 测试2: 最小资源模式（RGB+深度）
echo "测试2: 最小资源模式..."
roslaunch vins vins_camera_minimal.launch &
sleep 10

echo "检查所有流..."
timeout 3 rostopic hz /camera/infra1/image_rect_raw
timeout 3 rostopic hz /camera/color/image_raw
timeout 3 rostopic hz /camera/aligned_depth_to_color/image_raw

echo ""
echo "测试完成！按任意键停止所有进程..."
read

killall -9 roslaunch rosrun roscore 2>/dev/null
echo "所有进程已停止"
