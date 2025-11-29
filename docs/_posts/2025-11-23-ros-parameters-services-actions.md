---
title: "ROS 2 parameter, service, action" 
layout: post 
description: "ROS 2 参数、服务（Services）与动作（Actions）的命令行操作与核心区别总结。" 
categories: ROS
date: 2025-11-23
---

本笔记总结了 ROS 2 中除 Topic 以外的三种主要通信机制：**参数（Parameters）**、**服务（Services）** 和 **动作（Actions）**，并附带日志和数据记录工具的简要用法。

I. 参数 (Parameters)
------------------

参数是 Node **启动时可配置**的变量，用于修改 Node 的行为。

| 功能  | 命令  | 备注  |
| --- | --- | --- |
| **查看列表** | `ros2 param list` | 显示所有 Node 及其参数。`qos_overrides` 等为自动生成。 |
| **获取值** | `ros2 param get <node> <param>` | 直接返回参数的当前值。 |
| **设置值** | `ros2 param set <node> <param> <value>` | 实时修改参数值（除非参数只读）。可通过参数服务调用。 |
| **保存/导出** | `ros2 param dump <node> > <file.yaml>` | 将当前参数导出为 YAML 文件。 |
| **加载参数** | `ros2 param load <node> <file.yaml>` | 运行中的 Node 可以加载参数文件。 |
| **启动时加载** | `ros2 run <pkg> <exec> --ros-args --params-file <file.yaml>` | **重要：** 对于只读参数（如 `qos_overrides`），必须在启动时通过此方式赋值。 |

II. 服务 (Services)
-----------------

Service 是一种**请求-响应**（Request-Response）模式的通信方式，适用于**一次性**、**非持续性**的任务调用。

| 特性  | 说明  |
| --- | --- |
| **模式** | **Client (客户端)** 发送请求，**Server (服务器)** 接收请求并返回响应。 |
| **数量** | 一个 Service 只能有一个 Server，但可以有多个 Client。 |
| **自动服务** | 每个 Node 都会自动生成 **6 个参数服务**（`get_parameters`, `set_parameters` 等）。 |

| 功能  | 命令  | 备注  |
| --- | --- | --- |
| **查看服务列表** | `ros2 service list` | 查看所有可用的 Service。 |
| **查看服务类型** | `ros2 service list -t` | 同时显示 Service 的接口类型（如 `std_srvs/srv/Empty`）。 |
| **查找服务** | `ros2 service find <type_name>` | 查找特定接口类型的所有 Service。 |
| **查看接口结构** | `ros2 interface show <type_name>` | 显示 Service 的 Request 和 Response 结构，由 `---` 分割。 |
| **调用服务** | `ros2 service call <service> <type> <args>` | 发送请求并等待响应。无参数时 `<args>` 可省略（如 `Empty` 类型）。 |

III. 动作 (Actions)
-----------------

Action 是一种**基于 Service 的扩展**通信方式，专为**长时间运行**、\*\*可抢占（可取消）\*\*的任务设计。

| 元素  | 说明  |
| --- | --- |
| **Goal (目标)** | 相当于 Service 的 Request，定义任务目标。 |
| **Result (结果)** | 相当于 Service 的 Response，任务最终完成或失败的结果。 |
| **Feedback (反馈)** | 任务进行中，Server 持续向 Client 发送的实时状态信息（Topic 模式）。 |

| 功能  | 命令  | 备注  |
| --- | --- | --- |
| **查看列表** | `ros2 action list [-t]` | 查看所有 Action Server。 |
| **查看信息** | `ros2 action info <action_name>` | 显示该 Action 的 Client 和 Server 数量及对应 Node。 |
| **查看接口** | `ros2 interface show <action_type>` | 显示 Action 的 Goal, Result, Feedback 结构，由 `---` 分割。 |
| **发送目标** | `ros2 action send_goal <action> <type> <values>` | 发送目标请求。`<values>` 通常为 YAML 格式。 |
| **实时反馈** | `ros2 action send_goal ... --feedback` | 在发送目标的同时，实时显示 Server 返回的 Feedback 信息。 |

IV. 日志与数据记录
-----------

### 1\. 日志查看 (`rqt_console`)

| 工具  | 命令  | 备注  |
| --- | --- | --- |
| **日志查看器** | `ros2 run rqt_console rqt_console` | GUI 工具，用于实时筛选和查看所有 Node 的日志。 |
| **日志级别** | `FATAL > ERROR > WARN > INFO > DEBUG` | 级别从高到低。默认通常为 `INFO`。 |
| **启动时设置** | `ros2 run <pkg> <exec> --ros-args --log-level WARN` | 设置该 Node 仅显示 `WARN` 及更高级别的日志。 |

### 2\. 数据包记录 (`rosbag`)

`rosbag` 用于保存和重放 Topic 发布的数据流，便于复现和调试。

| 功能  | 命令  | 备注  |
| --- | --- | --- |
| **记录指定 Topic** | `ros2 bag record <topic1> <topic2>` | 记录一个或多个 Topic 的数据。 |
| **记录所有 Topic** | `ros2 bag record -a` | 记录所有正在发布的 Topic 数据。 |
| **指定文件名** | `ros2 bag record -o <name> ...` | `-o` 参数指定数据包文件夹名称。 |
| **查看信息** | `ros2 bag info <bag_file>` | 查看数据包中包含的 Topic、消息数量和时间范围。 |

