---
title: "Isaac Sim 日常踩坑速查"
layout: post
description: "Docker 里跑 Isaac Sim GUI 黑屏的三步排查；UE5 场景搬进 Omniverse / Isaac。"
categories: [IsaacSim]
date: 2026-07-21
---

Isaac Sim 相关的碎片笔记，合并备查。

## Isaac Sim

**GUI 起不来 / 黑屏，按这三步排查**（在 Docker 里跑 Isaac 的常见故障，按顺序试）：

1. **GPU 掉了？** `docker exec <isaac-gui-container> nvidia-smi` —— 报 NVML 错误就是容器把 GPU 丢了，`docker restart <isaac-gui-container>` 一般能救回来。
2. **X 授权掉了？** 宿主机 `xhost` 里没有 `LOCAL:`（注销 / 重启后会被重置），重跑 `xhost +local:`。
3. **前两步都正常** —— 直接看容器内日志：`docker exec <isaac-gui-container> tail -30 /tmp/gui.log`。

**把 UE5 场景搬进 Omniverse / Isaac** —— 从 UE5 导出后，用 **USD Composer + Scene Optimizer** 处理成可用的 USD；具体流程参考 AirLab 的相关 post，别自己硬啃。
