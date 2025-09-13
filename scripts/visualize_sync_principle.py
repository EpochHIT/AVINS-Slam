#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle('VINS-Surfel 时间戳同步原理图', fontsize=16, fontweight='bold')

# 时间轴设置
time_range = np.arange(0, 200, 1)  # 0-200ms
vins_times = [20, 50, 80, 110, 140, 170]  # VINS 里程计时间戳
depth_times = [15, 48, 78, 112, 145, 175]  # 深度图时间戳
color_times = [18, 52, 82, 108, 138, 172]  # 彩色图时间戳
imu_times = list(range(5, 200, 5))  # IMU 5ms 间隔

# 子图1: 原始时间戳（未同步）
ax1.set_title('原始传感器数据时间戳（未同步）', fontsize=12, fontweight='bold')
ax1.set_xlim(0, 200)
ax1.set_ylim(-0.5, 4.5)

# 绘制原始数据时间线
y_pos = {'VINS里程计': 4, '深度图': 3, '彩色图': 2, 'IMU': 1}
colors = {'VINS里程计': 'red', '深度图': 'blue', '彩色图': 'green', 'IMU': 'orange'}

# VINS 里程计
for t in vins_times:
    ax1.scatter(t, y_pos['VINS里程计'], c=colors['VINS里程计'], s=100, marker='s')
    ax1.text(t, y_pos['VINS里程计']+0.15, f'{t}ms', ha='center', va='bottom', fontsize=8)

# 深度图
for t in depth_times:
    ax1.scatter(t, y_pos['深度图'], c=colors['深度图'], s=80, marker='o')
    ax1.text(t, y_pos['深度图']+0.15, f'{t}ms', ha='center', va='bottom', fontsize=8)

# 彩色图
for t in color_times:
    ax1.scatter(t, y_pos['彩色图'], c=colors['彩色图'], s=80, marker='^')
    ax1.text(t, y_pos['彩色图']+0.15, f'{t}ms', ha='center', va='bottom', fontsize=8)

# IMU
for i, t in enumerate(imu_times[::4]):  # 显示部分IMU点以免过密
    ax1.scatter(t, y_pos['IMU'], c=colors['IMU'], s=30, marker='|')

ax1.set_yticks(list(y_pos.values()))
ax1.set_yticklabels(list(y_pos.keys()))
ax1.set_xlabel('时间 (ms)')
ax1.grid(True, alpha=0.3)

# 子图2: 同步匹配过程
ax2.set_title('时间戳同步匹配过程（容错±10ms）', fontsize=12, fontweight='bold')
ax2.set_xlim(0, 200)
ax2.set_ylim(-0.5, 4.5)

# 显示匹配过程
for i, vins_t in enumerate(vins_times[:4]):  # 显示前4个匹配
    # VINS 时间戳
    ax2.scatter(vins_t, y_pos['VINS里程计'], c=colors['VINS里程计'], s=100, marker='s')
    
    # 匹配容错区域
    tolerance = 10
    rect = Rectangle((vins_t-tolerance, 0.5), 2*tolerance, 3, 
                    alpha=0.2, facecolor='yellow', edgecolor='orange', linewidth=2)
    ax2.add_patch(rect)
    
    # 找到匹配的深度图和彩色图
    depth_match = min(depth_times, key=lambda x: abs(x - vins_t))
    color_match = min(color_times, key=lambda x: abs(x - vins_t))
    
    if abs(depth_match - vins_t) <= tolerance:
        ax2.scatter(depth_match, y_pos['深度图'], c=colors['深度图'], s=80, marker='o')
        ax2.plot([vins_t, depth_match], [y_pos['VINS里程计'], y_pos['深度图']], 
                'k--', alpha=0.5, linewidth=1)
        ax2.text(depth_match, y_pos['深度图']-0.25, f'Δ{abs(depth_match-vins_t):.0f}ms', 
                ha='center', va='top', fontsize=7, color='blue')
    
    if abs(color_match - vins_t) <= tolerance:
        ax2.scatter(color_match, y_pos['彩色图'], c=colors['彩色图'], s=80, marker='^')
        ax2.plot([vins_t, color_match], [y_pos['VINS里程计'], y_pos['彩色图']], 
                'k--', alpha=0.5, linewidth=1)
        ax2.text(color_match, y_pos['彩色图']-0.25, f'Δ{abs(color_match-vins_t):.0f}ms', 
                ha='center', va='top', fontsize=7, color='green')
    
    ax2.text(vins_t, y_pos['VINS里程计']+0.25, f'基准:{vins_t}ms', ha='center', va='bottom', 
            fontsize=8, fontweight='bold', color='red')

ax2.set_yticks(list(y_pos.values()))
ax2.set_yticklabels(list(y_pos.keys()))
ax2.set_xlabel('时间 (ms)')
ax2.grid(True, alpha=0.3)

# 添加图例
tolerance_patch = mpatches.Rectangle((0, 0), 1, 1, facecolor='yellow', alpha=0.2, 
                                   edgecolor='orange', linewidth=2)
ax2.legend([tolerance_patch], ['±10ms 容错区域'], loc='upper right')

# 子图3: 同步后结果
ax3.set_title('同步后的统一时间戳', fontsize=12, fontweight='bold')
ax3.set_xlim(0, 200)
ax3.set_ylim(-0.5, 4.5)

# 同步后的数据
sync_times = vins_times[:4]  # 成功同步的时间戳

for sync_t in sync_times:
    # 所有数据都使用相同的时间戳
    ax3.scatter(sync_t, y_pos['VINS里程计'], c=colors['VINS里程计'], s=100, marker='s')
    ax3.scatter(sync_t, y_pos['深度图'], c=colors['深度图'], s=80, marker='o')
    ax3.scatter(sync_t, y_pos['彩色图'], c=colors['彩色图'], s=80, marker='^')
    
    # 垂直对齐线
    ax3.plot([sync_t, sync_t], [1.5, 4.5], 'k-', linewidth=2, alpha=0.7)
    ax3.text(sync_t, 0.5, f'{sync_t}ms\n(统一)', ha='center', va='center', 
            fontsize=9, fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", 
                                                   facecolor="lightblue", alpha=0.8))

# 添加说明文字
ax3.text(100, 0, '✓ 时间戳完全一致\n✓ 数据严格对齐\n✓ 因果关系保持', 
        ha='center', va='center', fontsize=10, 
        bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8))

ax3.set_yticks(list(y_pos.values()))
ax3.set_yticklabels(list(y_pos.keys()))
ax3.set_xlabel('时间 (ms)')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/cwkj/AVINS-Slam/docs/timestamp_sync_diagram.png', dpi=300, bbox_inches='tight')
plt.show()

print("时间戳同步原理图已保存至: /home/cwkj/AVINS-Slam/docs/timestamp_sync_diagram.png")
