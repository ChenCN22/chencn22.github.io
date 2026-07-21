---
title: "记一次探索规划器冷启动卡死的三层排查：段错误 → NaN 冻结 → 未知体素孤岛"
layout: post
description: "自主探索机器人开局不动的一次完整 debug：gdb 定位段错误、发现 NaN 航向角冻结控制器、最后挖出针孔相机导致的 roadmap 孤岛问题，用原地旋转扫描解决。"
categories: [ROS]
date: 2026-07-21
---

最近在做 Spot 语义探索的 demo 时碰到一个很典型的"开局机器人一动不动"问题，前后挖了三层才挖到真正的根因，过程值得记录一下。

## 症状

启动仿真 + 规划器后，机器人偶尔直接段错误退出（exit -11），偶尔就是站在原地不动，rviz 里所有 candidate viewpoint 都是 0。

## 第一层：段错误

roslaunch 里直接给 `launch-prefix="gdb ..."` 参数会被 roslaunch 转义搞坏，所以写了个 wrapper 脚本：

```bash
#!/bin/bash
exec gdb -batch -return-child-result -ex run -ex bt \
    -ex "thread apply all bt" --args "$@"
```

backtrace 显示崩在信息增益更新函数里，访问了越界的 region 网格索引。往回追，传进去的 goal 竟然在**地板以下**（z = -0.73 ~ -1.21）。

原因：恢复逻辑 `recover2SafePos` 在规划失败时会沿"当前位置 → 上一个 goal"方向延伸出一个安全点。但**冷启动时上一个 goal 还是默认构造的 (0,0,0)**，方向向量直接指向世界原点地下，延伸出来的目标自然在地下。

修复：加一个 `lastGoalValid_` 标志，第一次规划成功前不做延伸。

## 第二层：NaN 冻结

同一个根因还有另一个症状形态：垃圾 recover goal 带着**全零四元数**下发，waypoint 的航向角算出来是 `-nan`，下游的航向控制器拿到 NaN 后直接静默冻结——机器人不崩溃、不报错，就是不动。比段错误隐蔽多了。

教训：四元数默认构造是全零不是单位四元数，任何"没初始化就用"的路径都可能产出 NaN 航向。

## 第三层：未知体素孤岛（真正的根因）

修完前两层，机器人不崩了，但还是不动。打了干净的独立日志（roslaunch 多次运行会复用 uuid 目录、stdout 日志互相覆盖，所以用 `stdbuf -oL -eL roslaunch ... > run.log 2>&1` 自己收）发现规划器在以 3Hz 无限失败：

```
[GridGraph::findPath] none of 8 nearest grid neighbors have a valid transition
```

明明已经有 6 个 exploring region，goal candidate 却始终是 0。原因是感知配置从 360° lidar 换成了**前向针孔相机**：开局机器人只看得见正前方一个楔形区域，自己脚下和周围全是未知体素——它在 roadmap 上是一座"未知体素孤岛"，任何路径都连不出去，于是永远规划失败、永远原地 recover、永远看不到新东西，死锁。

（有趣的是：以前"能跑"的版本其实是靠第一层 bug 的垃圾 recover goal 偶然让机器人乱转，误打误撞扫开了周围地图。修好 bug 反而暴露了真问题。）

修复：冷启动时如果还没有任何有效规划，就通过姿态接口让机器人**原地旋转扫描**（每次 recover 触发时 yaw +0.8 rad），360° 扫一圈（~8 秒）把周围体素扫成已知，第一次规划成功后自动退出。机器人从此开局丝滑起飞。

## 总结

- 一个根因（未初始化的 lastGoal）可以表现成两种完全不同的症状（段错误 / NaN 冻结），取决于垃圾数据先走到哪条路径。
- "以前能跑"不等于"以前是对的"——有可能是两个 bug 互相抵消。
- 传感器 FOV 变窄后，探索系统需要显式的冷启动初始扫描，不能指望第一帧就有可通行的 roadmap。
- NaN 是最阴险的失败模式：不崩溃、不报警，只是安静地停住。关键数值路径上值得加 `isfinite` 检查。
