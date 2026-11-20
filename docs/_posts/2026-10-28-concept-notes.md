---
title: "概念速记（持续更新）"
layout: post
description: "Pixel / voxel、wall time、oracle、dwell、race condition、stub、churn、stale，OVDM、RAG、temperature，以及新加的 MTF / MTF50、留出集（hold-out set）。"
categories: [Misc]
date: 2026-10-28
---

读论文、开会、干活时碰到的术语，一两句话记下，持续追加。

## 基础

- **Pixel vs Voxel**：2D 图像最小单元 vs 3D 空间最小单元（点云体素化的基本单位）。
- **Wall time vs CPU time**：真实流逝时间 vs CPU 实际计算时间（不含 IO 等待/阻塞）。
- **Oracle**：理想化的黑盒，能直接给出正确答案、通常不可实现——常拿来当性能上界 / baseline。
- **Dwell**：停留（在某个视点 / 状态停留的时长）。
- **Race condition（竞态）**：并发下多个操作先后顺序不确定，结果时对时错、不可复现；同样代码同样场景，绝大多数时候正常、偶尔失败。
- **Stub（桩）**：接口签名正确、内部是假的 / 极简替代实现，真实组件缺位时先把流程跑通。
- **Churn**：状态 / 输出在几个值之间来回横跳，每次跳动都触发下游代价、净收益却为零。软件里的 **code churn** 指刚写的代码（常在两三周内）被重写 / 删除 / 替换的比率；另有 backlog / customer / team churn。
- **Stale（过期）**：数据还在，但已经不反映当前真实状态。

## 2026-10 新增

- **Open-vocabulary detection models (OVDM)**：开放词汇检测模型，简单说就是无限词汇表的 YOLO（原版 YOLO 是闭式词汇表）。问题是模型大，所以有一支工作专做轻量化。
- **RAG（Retrieval-Augmented Generation）**：相当于给 LLM 接一个外部数据存储。文档先 chunking，每个 chunk 编码成 vector 存进向量数据库；用户提问时按 cosine similarity 检索出和 query 相近的 chunk 一起喂给模型。这只是 naive RAG 的流程，现在一般用更复杂的 Modular RAG。
- **Temperature = 0**：模型输出变成确定性的（deterministic），同样输入得到同样回答。

## 2026-11 新增

- **MTF（Modulation Transfer Function，调制传递函数）**：衡量成像系统把物体上的空间频率（明暗条纹的疏密）传到图像上的能力。对比度（调制度）定义为 M = (I<sub>max</sub> − I<sub>min</sub>) / (I<sub>max</sub> + I<sub>min</sub>)，I 是亮度；MTF(f) = M<sub>图像</sub>(f) / M<sub>物体</sub>(f)。条纹越密，MTF 越低，细节越糊。
  - **MTF50**：MTF 降到一半时对应的空间频率，常用来一句话概括镜头 / 相机的清晰度，单位如 cycles/pixel 或 lp/mm。
- **留出集（hold-out set）**：开发过程中完全不碰的一批数据或随机种子，设计定下来之后只拿来评估一次，检验提升在没见过的局面上是否也成立。例：开发时用种子 1000–2000 训练和测试，算法定型后再在 9000–9050 上测一遍。
  - **我的看法：hold-out 测试有没有意义？** 对算法开发有意义，比如机器学习训练里的 train / val / test 划分。但从 agent / policy 研究的角度，任务和环境上的泛化更重要：换新环境、新任务去测 agent 或 policy，比在已有数据上换种子做 hold-out 更有研究价值，传统的种子和数据划分做不到这一点。至于值不值得为验证泛化专门做新环境、定义新任务，取决于研究的规模。
