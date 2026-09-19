---
title: "16-831 机器人学习笔记 (1)：从 ALOHA 到 world model"
layout: post
description: "16-831 Introduction to Robot Learning 头两节课的概念笔记：human-in-the-loop、统一 perception/control/planning、model-based vs model-free RL 与 world model、CNN 的 equivariance / invariance 与数据增强。"
categories: [rl]
date: 2026-09-03
---

这学期在上 16-831（Introduction to Robot Learning），一边为把自己的探索/主动感知项目往 end-to-end 方向转做准备。把前两节课的概念先记下来。

## ALOHA：human-in-the-loop 的启发

ALOHA 是低成本双臂遥操作系统，常被拿来采集模仿学习的数据。它给的一个直觉是：**human-in-the-loop 的 perception → control → feedback 闭环，本质在补偿延迟与不精确**（compensate for delay & imprecision）。人不断根据反馈修正动作，把系统的时延和执行误差"吸收"掉——这也是为什么纯开环很难，而带反馈的策略鲁棒得多。

## 统一 perception 与 control

一个核心想法：**把 perception 和 control 统一起来看**，而不是感知一段、控制一段地手工拼。

- 两端其实都是**高维**的：high-dim input（图像 / 点云 / 全景…）→ high-dim output（关节 / 末端 / 姿态动作…）。
- 于是 `perception + control` 可以看成一个 **high-dim input → high-dim output** 的映射，用一个网络端到端地学。

## Robot learning = 统一 perception, control, planning

再往上一层，robot learning 的目标是把 **perception、control、planning** 三者一起纳入一个可学习的系统，而不是三段独立、各自手调的 pipeline。这正是"更 end-to-end"的含义。

## model-based RL vs model-free RL

两者的分界点很简单：**要不要显式地学一个 transition（状态转移）模型**。

- **model-based**：显式学 `s_{t+1} = f(s_t, a_t)`（或它的分布），也就是学一个 **world model**，再用它来规划 / 在"想象"里做 rollout。
- **model-free**：不学转移，直接从交互里学 policy / value。

一句话记：**world model = 被显式学出来的 transition**。

## 一条方法论

ML 里要**关注数学本质，而不是花哨的名字**（pay attention to the mathematical nature rather than fancy names）。名字层出不穷，但底下的目标函数、归纳偏置（inductive bias）才是真正决定行为的东西。

## CNN：equivariance 与 invariance

两个容易混的概念，用 `f` 表示网络、`g` 表示对输入的某种变换：

- **Equivariance（等变）**：`f(g(x)) = g(f(x))`。对输入做变换，输出跟着做**同样**的变换。卷积对**平移**是等变的。
- **Invariance（不变）**：`f(g(x)) = f(x)`。对输入做变换，输出**不变**。例如希望"对图像做某种变换后卷积/预测结果不变"——严格说 vanilla conv 对平移是等变、再经 pooling 得到局部平移不变；对缩放并不天然不变，所以才需要下面这招。
- **data augmentation** 正是用来增强这两种性质：训练时人为加各种变换（平移、缩放、翻转…），逼网络学到对它们的等变 / 不变。

这套 equivariance / invariance 的思路在 **robot learning 里同样关键**——例如希望策略对视角、机器人位姿的变换具有恰当的等变性，能显著提升样本效率与泛化。

---

## 附：24-880 概念速记

顺手记几条另一门课里的大模型相关概念：

- **Scaling law**：性能随参数 / 数据 / 算力规模**可预测地**提升。
- **MMLU**：常用的大模型知识 / 推理评测基准。
- **MCP**：Model Context Protocol，给模型接外部工具 / 数据的标准接口。
