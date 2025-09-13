#!/bin/bash

# AVINS-SLAM 调试启动脚本
# 用于测试不同配置并诊断问题

echo "=========================================="
echo "AVINS-SLAM 调试启动脚本"
echo "=========================================="

# 功能函数
function check_realsense() {
    echo "检查RealSense设备..."
    if command -v rs-enumerate-devices >/dev/null 2>&1; then
        rs-enumerate-devices
    else
        echo "警告: rs-enumerate-devices 命令未找到"
    fi
    echo ""
}

function kill_all_nodes() {
    echo "停止所有相关节点..."
    killall -9 roslaunch rosrun roscore 2>/dev/null
    sleep 2
    echo "节点已停止"
    echo ""
}

function start_roscore() {
    echo "启动roscore..."
    roscore &
    sleep 3
    echo "roscore已启动"
    echo ""
}

function test_camera_config() {
    local config_name=$1
    local launch_file=$2
    
    echo "=========================================="
    echo "测试配置: $config_name"
    echo "使用文件: $launch_file"
    echo "=========================================="
    
    echo "启动相机节点..."
    roslaunch vins $launch_file &
    sleep 10
    
    echo "检查话题状态..."
    echo "红外图像1:"
    timeout 3 rostopic hz /camera/infra1/image_rect_raw
    echo ""
    
    echo "红外图像2:"
    timeout 3 rostopic hz /camera/infra2/image_rect_raw
    echo ""
    
    echo "彩色图像:"
    timeout 3 rostopic hz /camera/color/image_raw
    echo ""
    
    echo "深度图像:"
    timeout 3 rostopic hz /camera/aligned_depth_to_color/image_raw
    echo ""
    
    echo "IMU数据:"
    timeout 3 rostopic hz /camera/imu
    echo ""
    
    echo "按回车键继续测试下一个配置..."
    read
    
    echo "停止当前配置..."
    killall -9 roslaunch 2>/dev/null
    sleep 3
}

function test_full_system() {
    echo "=========================================="
    echo "测试完整AVINS-SLAM系统"
    echo "=========================================="
    
    echo "1. 启动相机（优化配置）..."
    roslaunch vins vins_camera_optimized.launch &
    sleep 10
    
    echo "2. 启动时间戳同步..."
    roslaunch vins vins_surfel_timestamp_sync.launch &
    sleep 5
    
    echo "3. 启动VINS节点..."
    rosrun vins vins_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml &
    sleep 5
    
    echo "4. 启动回环检测..."
    rosrun loop_fusion loop_fusion_node ~/AVINS-Slam/src/VINS-Fusion/config/realsense_d435i_fix/realsense_stereo_imu_config.yaml &
    sleep 5
    
    echo "系统已启动，检查状态..."
    echo ""
    
    echo "同步后的红外图像:"
    timeout 3 rostopic hz /camera/infra1/image_rect_raw_sync
    echo ""
    
    echo "同步后的彩色图像:"
    timeout 3 rostopic hz /camera/color/image_raw_sync
    echo ""
    
    echo "同步后的深度图像:"
    timeout 3 rostopic hz /camera/aligned_depth_to_color/image_raw_sync
    echo ""
    
    echo "VINS位姿输出:"
    timeout 3 rostopic hz /vins_estimator/odometry
    echo ""
    
    echo "等待DenseSurfelMapping启动..."
    echo "手动启动: rosrun surfel_fusion surfel_fusion"
    echo ""
    echo "按回车键停止系统..."
    read
}

# 主菜单
while true; do
    echo "=========================================="
    echo "选择测试选项:"
    echo "1) 检查RealSense设备"
    echo "2) 测试标准配置 (vins_camera.launch)"
    echo "3) 测试低带宽配置 (vins_camera_low_bandwidth.launch)"
    echo "4) 测试优化配置 (vins_camera_optimized.launch)"
    echo "5) 测试完整AVINS-SLAM系统"
    echo "6) 停止所有节点"
    echo "7) 退出"
    echo "=========================================="
    read -p "请选择 (1-7): " choice
    
    case $choice in
        1)
            check_realsense
            ;;
        2)
            kill_all_nodes
            start_roscore
            test_camera_config "标准配置" "vins_camera.launch"
            ;;
        3)
            kill_all_nodes
            start_roscore
            test_camera_config "低带宽配置" "vins_camera_low_bandwidth.launch"
            ;;
        4)
            kill_all_nodes
            start_roscore
            test_camera_config "优化配置" "vins_camera_optimized.launch"
            ;;
        5)
            kill_all_nodes
            start_roscore
            test_full_system
            ;;
        6)
            kill_all_nodes
            ;;
        7)
            kill_all_nodes
            echo "退出调试脚本"
            exit 0
            ;;
        *)
            echo "无效选择，请重试"
            ;;
    esac
done
