---
title: "ROS2 学习笔记：Colcon 构建与 Overlay/Underlay 机制"
layout: post
description: "深入理解 ROS2 的构建工具 Colcon、工作区 Overlay/Underlay 的分层概念，以及如何解决 Conda 环境下的依赖问题。"
categories: [ROS, Linux]
date: 2025-11-27
---

# Colcon：ROS 2 的通用构建工具

`colcon` 是 ROS 2 的标准构建工具（Build Tool），用于在工作区（Workspace）中构建所有的包（Packages）。

## ROS Workspace 结构

标准的 ROS 2 工作区目录结构如下：

```text
~/ros2_ws/
    /src        # 源代码目录 (Source codes) - 手动创建
    # 以下目录由 colcon build 自动生成
    /build      # 中间构建文件
    /install    # 安装目录 (Setup files & Executables)
    /log        # 构建日志
````

构建流程
----

### 1\. 创建工作区

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2\. 构建 (Build)

在工作区的根目录下运行：

```
colcon build --symlink-install
```

**关于 `--symlink-install` 参数的理解：**

*   **作用**：它使用符号链接（Symbolic Links）将 `/src` 中的源代码链接到 `/install` 目录，而不是直接复制文件。
*   **优势**：
    *   **Python (解释型)**：修改源码后无需重新编译，立即生效，极大地加快开发速度。
    *   **C++ (编译型)**：修改源码后**仍然需要**重新运行 `colcon build` (因为需要 CMake 重新编译生成二进制文件)，但安装步骤会变快。

### 3\. 测试与环境配置

```
colcon test
source ./install/setup.bash
```

* * *

核心概念：Overlay 与 Underlay
=======================

这是 ROS 2 中最重要但也容易混淆的环境分层概念。

什么是 Underlay (底层)？
------------------

Underlay 指的是**基础环境**，通常是只读的。

*   最常见的 Underlay 是 ROS 2 的官方安装路径：`/opt/ros/jazzy/`。
*   它包含了核心库和标准工具。

什么是 Overlay (叠加层/上层)？
---------------------

Overlay 是我们**自己创建的工作区**（如 `~/ros2_ws`）。

*   它是一个辅助工作区，叠加在 Underlay 之上。
*   **覆盖规则**：如果在 Overlay 中定义了一个与 Underlay 同名的包，Overlay 中的版本会**覆盖**底层的版本（优先级更高）。
*   **隔离性**：在 Overlay 中添加新包或修改代码，不会破坏底层的稳定环境。

环境生效机制 (Sourcing)
-----------------

要让系统知道这些层级的存在，我们需要按顺序 source 环境脚本。

1.  **加载底层 (Underlay)**：
    ```
    source /opt/ros/jazzy/setup.bash
    ```
    _此时，终端只能看到官方的包。_
2.  **加载叠加层 (Overlay)**：
    ```
    source ~/ros2_ws/install/setup.bash
    ```
    _此时，终端既能看到官方包，也能看到我们自己的包。如果包名冲突，以 Overlay 为准。_

**开发注意事项**：

*   **修改代码后**：需要在 Overlay 的环境下（即 source 了 `install/setup.bash` 的终端）运行节点 (`ros2 run`) 才能看到修改效果。
*   **多层叠加**：按照特定顺序 source 多个不同的工作区来实现多层叠加（例如：官方 -\> 公司公共库 -\> 个人项目）。

* * *

Package 
=============

什么是 Package？
------------

Package 是 ROS 2 的基本组织单元。它不仅仅是几个 Node 的集合，一个标准的 Package 包含：

*   **Nodes** (节点代码)
*   **`package.xml`** (元数据与依赖声明)
*   **Build Config** (`CMakeLists.txt` for C++ 或 `setup.py` for Python)
*   **Interfaces** (可选，自定义的消息 msg / 服务 srv)

package.xml 与依赖管理
-----------------

`package.xml` 是声明依赖的地方，关键标签包括：

*   `<depend>`: 通用依赖
*   `<build_depend>`: 仅构建时需要
*   `<exec_depend>`: 仅运行时需要

* * *

rosdep：依赖管理工具
=============

`rosdep` 用于安装 Package 所需的**系统级依赖**。

常用命令
----

```
# 在工作区根目录下
rosdep install -i --from-path src --rosdistro jazzy -y
```

遇到的坑：Conda 环境与 rosdep
---------------------

在使用 Anaconda/Miniconda 环境开发 ROS 2 时，经常遇到 `ModuleNotFoundError`。

**原因分析**：

1.  `rosdep` 默认调用系统的包管理器（如 `apt`）将 Python 库（如 `python3-empy`）安装到**系统 Python 路径** (`/usr/lib/python3/...`)。
2.  **Conda 环境是隔离的**。当你 `conda activate ros2` 后，Python 解释器指向的是 Conda 环境内部。
3.  Conda 环境**看不到**系统路径下的包，因此报错。

**解决方案**：

*   **方法 A（推荐）**：在 Conda 环境中通过 Conda 安装依赖。
    ```
    # 添加必要的频道
    conda config --add channels conda-forge
    conda config --add channels robostack
    # 安装缺失的包
    conda install empy catkin_pkg numpy
    ```
*   **方法 B（彻底解决）**：使用 **RoboStack** 方案，这可以在 Conda 内完整安装 ROS 2，避免混合使用系统 ROS 和 Conda Python。
*   **方法 C（简单粗暴）**：不使用 Conda，直接使用系统原生的 Python 进行开发。

* * *

实用技巧
====

### 1\. 遇到构建问题的“万能药”

如果 `colcon build` 报错且原因不明（比如修改了包名或移动了文件），尝试清理缓存：

```
rm -rf build/ install/ log/
colcon build
```

### 2\. 忽略特定包

如果不想编译工作区内的某个包，在该包的文件夹内创建一个名为 `COLCON_IGNORE` 的空文件即可。

### 3\. Colcon Mixins (命令缩写)

避免每次都输入冗长的 cmake 参数。

```
# 安装 mixins
colcon mixin add default [https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml](https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml)
colcon mixin update default

# 使用示例 (release 版本构建)
colcon build --mixin release
```

### 4\. 加速开发

对于只有少量包的修改，建议使用 Overlay 开发，因为只构建 Overlay 比重新构建整个系统要快得多。