---
title: "ROS / 感知 日常踩坑速查"
layout: post
description: "RViz goal topic 对不上、ROS2 编译缺 CMake module、鱼眼标定别用二维码板、大数据 topic 用 reliable QoS。"
categories: [ROS]
date: 2026-07-21
---

日常干活攒下的 ROS 和感知相关碎片笔记，合并备查。

## ROS / 感知

**RViz 发了 2D Goal 机器人没反应** —— RViz 默认发 `/goal_pose`（或 `/move_base_simple/goal`，看版本），和导航栈监听的 topic 对不上。在 `Panels → Tool Properties` 里改成实际的 topic。

**ROS2 编译缺 CMake module** —— 按顺序：

```bash
source /opt/ros/<distro>/setup.bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**OpenCV 鱼眼标定 (`cv2.fisheye.calibrate`)** —— 不要用带二维码的标定板，QR 图案会干扰角点识别，标定要么失败要么精度很差。用干净的纯棋盘格。

**ROS2 大数据 topic 别用 `best_effort`，改 `reliable`** —— DDS 底层把大消息按 UDP 分块发；`best_effort` 下**丢任意一块，整条消息就作废**（收端拼不出来直接丢弃），高清视频、大点云这种一帧几 MB 的 topic 尤其明显。改成 `reliable` 后丢块会自动重传。数据量再大，可能还得把 DDS 的**共享内存（shared memory）**调大。调 Insta360 X4 driver 时踩的坑。
