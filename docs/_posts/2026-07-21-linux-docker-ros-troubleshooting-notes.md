---
title: "Linux / Docker / ROS 日常踩坑速查合集"
layout: post
description: "从语音笔记收件箱里整理出来的一批零散但高频的踩坑记录：磁盘占满、X11、Docker 清理、RViz goal topic、鱼眼标定等。"
categories: [Linux, ROS]
date: 2026-07-21
---

日常干活攒下的一批碎片笔记，单独成文都太短，合并归档在这里备查。

## Linux / 系统

**`apt update` 报错写不了 /tmp 或缓存** —— 十有八九是 `/` 分区满了，而且常见元凶是 Docker。`df -h` 确认，`docker system df` 看占用，`docker system prune -a` 清理（注意会删掉未被容器引用的镜像）。

**图形界面出问题** —— 先 `nvidia-smi` 确认显卡还在，再 `sudo systemctl restart gdm`。

**强杀卡死的 GUI 窗口** —— 终端敲 `xkill`，鼠标变成 X 后点谁杀谁。对卡死的 RViz / Gazebo / SSH 转发窗口特别好用。

**SSH 远程打开浏览器** —— `ssh -X user@host xdg-open https://example.com`。

**nano 里复制长行不完整** —— 长行显示被截断导致复制丢内容。`nano -l` 启动，或在 `.nanorc` 里加 `set softwrap`。

**tar 速查**

```bash
tar -xzvf file.tar.gz    # 解压 .tar.gz
tar -xjvf file.tar.bz2   # 解压 .tar.bz2
tar -xvf  file.tar       # 解压 .tar
tar -cvf  out.tar dir/   # 打包
```

## Docker

**构建基础命令**

```bash
docker build -t name:tag .              # 末尾的 . 是构建上下文
docker build -f MyDockerfile -t name:tag .
docker build --no-cache -t name:tag .   # 跳过缓存
```

**`docker kill` ≠ 清理** —— kill 只停进程，容器文件系统还在占盘。要 `docker rm -f <id>` 或 `docker ps -a` 之后 `docker container prune`。反过来也要小心：**容器一旦被 rm，容器内做过的编译/安装全部消失**，挂载卷之外的东西别当持久存储用。

**容器里跑 GUI** —— 起容器前先在宿主机执行 `xhost +local:root`，否则容器内的 GUI 程序连不上 X server。

## ROS / 感知

**RViz 发了 2D Goal 机器人没反应** —— RViz 默认发 `/goal_pose`（或 `/move_base_simple/goal`，看版本），和导航栈监听的 topic 对不上。在 `Panels → Tool Properties` 里改成实际的 topic。

**ROS2 编译缺 CMake module** —— 按顺序：

```bash
source /opt/ros/<distro>/setup.bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**OpenCV 鱼眼标定 (`cv2.fisheye.calibrate`)** —— 不要用带二维码的标定板，QR 图案会干扰角点识别，标定要么失败要么精度很差。用干净的纯棋盘格。

## 概念速记

- **Pixel vs Voxel**：2D 图像最小单元 vs 3D 空间最小单元（点云体素化的基本单位）。
- **Wall time vs CPU time**：真实流逝时间 vs CPU 实际计算时间（不含 IO 等待/阻塞）。
