---
title: "Linux / Docker / ROS 日常踩坑速查合集"
layout: post
description: "从语音笔记收件箱里整理出来的一批零散但高频的踩坑记录：磁盘占满、X11、Docker 清理、RViz goal topic、鱼眼标定、ROS2 QoS 丢包、Isaac Sim GUI 黑屏、git 免密、ffmpeg 抽帧、术语速记等。"
categories: [Linux, ROS, IsaacSim]
date: 2026-07-21
---

日常干活攒下的一批碎片笔记，单独成文都太短，合并归档在这里备查。

## Linux / 系统

**`apt update` 报错写不了 /tmp 或缓存** —— 十有八九是 `/` 分区满了，而且常见元凶是 Docker。`df -h` 确认，`docker system df` 看占用，`docker system prune -a` 清理（注意会删掉未被容器引用的镜像）。

**图形界面出问题** —— 先 `nvidia-smi` 确认显卡还在，再 `sudo systemctl restart gdm`。

**强杀卡死的 GUI 窗口** —— 终端敲 `xkill`，鼠标变成 X 后点谁杀谁。对卡死的 RViz / Gazebo / SSH 转发窗口特别好用。

**按命令行匹配杀进程** —— `pkill -f "match string"` 按**完整命令行**匹配（不像默认只匹配进程名），杀带一长串参数的进程 / 脚本时好用。坑：匹配串会把 `pkill`/`grep` 命令自己也匹配进去，用方括号避开（如 `patter[n]`）。

**SSH 远程打开浏览器** —— `ssh -X user@host xdg-open https://example.com`。

**切换到另一个用户** —— `su <username>`（命令行下切换用户身份，跑权限归属不同的东西时用得上）。

**原版中文输入法不好用，换 Fcitx5**

```bash
sudo apt install fcitx5 fcitx5-chinese-addons fcitx5-rime
im-config    # 选 fcitx5，注销重登生效
```

登录后命令行 `fcitx5 &` 起进程，右上角图标进 config 把 Rime 加进输入法列表，再在 Rime 里切简/繁。

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

**ROS2 大数据 topic 别用 `best_effort`，改 `reliable`** —— DDS 底层把大消息按 UDP 分块发；`best_effort` 下**丢任意一块，整条消息就作废**（收端拼不出来直接丢弃），高清视频、大点云这种一帧几 MB 的 topic 尤其明显。改成 `reliable` 后丢块会自动重传。数据量再大，可能还得把 DDS 的**共享内存（shared memory）**调大。调 Insta360 X4 driver 时踩的坑。

## Isaac Sim

**GUI 起不来 / 黑屏，按这三步排查**（在 Docker 里跑 Isaac 的常见故障，按顺序试）：

1. **GPU 掉了？** `docker exec <isaac-gui-container> nvidia-smi` —— 报 NVML 错误就是容器把 GPU 丢了，`docker restart <isaac-gui-container>` 一般能救回来。
2. **X 授权掉了？** 宿主机 `xhost` 里没有 `LOCAL:`（注销 / 重启后会被重置），重跑 `xhost +local:`。
3. **前两步都正常** —— 直接看容器内日志：`docker exec <isaac-gui-container> tail -30 /tmp/gui.log`。

**把 UE5 场景搬进 Omniverse / Isaac** —— 从 UE5 导出后，用 **USD Composer + Scene Optimizer** 处理成可用的 USD；具体流程参考 AirLab 的相关 post，别自己硬啃。

## Git & 媒体工具

**git 免密 clone（配 SSH key）** —— 不想每次输账号密码，配一把 key：

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub          # 复制输出
# 贴到 GitHub → Settings → SSH and GPG keys → New SSH key
ssh -T git@github.com              # 测试连通
```

之后用 `git@github.com:user/repo.git` 形式 clone 就不再要密码。

**ffmpeg 抽帧（比 OpenCV 快）** —— 降帧率 + 导出 JPG 序列，处理视频数据集常用：

```bash
#!/bin/bash
video_name=$1
fps=$2
base_name="${video_name%.*}"
reduced_video="${base_name}_${fps}fps.mp4"
output_dir="${base_name}_${fps}fps"

# 降帧率
ffmpeg -i "$video_name" -filter:v "fps=$fps" -c:a copy "$reduced_video"
# 抽帧为 JPG（-q:v 2 高质量，从 0 开始编号）
mkdir -p "$output_dir"
ffmpeg -i "$reduced_video" -q:v 2 -start_number 0 "${output_dir}/%05d.jpg"
```

## 概念速记

- **Pixel vs Voxel**：2D 图像最小单元 vs 3D 空间最小单元（点云体素化的基本单位）。
- **Wall time vs CPU time**：真实流逝时间 vs CPU 实际计算时间（不含 IO 等待/阻塞）。
- **Oracle**：理想化的黑盒，能直接给出正确答案、通常不可实现——常拿来当性能上界 / baseline。
- **Dwell**：停留（在某个视点 / 状态停留的时长）。
- **Race condition（竞态）**：并发下多个操作先后顺序不确定，结果时对时错、不可复现；同样代码同样场景，绝大多数时候正常、偶尔失败。
- **Stub（桩）**：接口签名正确、内部是假的 / 极简替代实现，真实组件缺位时先把流程跑通。
- **Churn**：状态 / 输出在几个值之间来回横跳，每次跳动都触发下游代价、净收益却为零。软件里的 **code churn** 指刚写的代码（常在两三周内）被重写 / 删除 / 替换的比率；另有 backlog / customer / team churn。
- **Stale（过期）**：数据还在，但已经不反映当前真实状态。
