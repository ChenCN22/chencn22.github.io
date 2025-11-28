---
title: "ROS2"
layout: post
description: 
categories: ROS
date: 2025-11-27
---

# colcon

colcon是ROS的通用build工具，

在ws中`colcon build`构建ws中所有node

## ROS work space

```
/sw
    /src        source codes
    # 以下为colcon创建
    /build      
    /install    
    /log

```

1. 创建workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. 添加sourcecode到`/src`

3. underlay? 不明白

underlay就是官方ros2环境，/opt/ros/jazzy/

自己创建的workspace是在官方环境之上的overlay

4. build 

在workspace的根目录

```build
colcon build --symlink-install
```
`--symlink-install`使得修改source是同步修改installed file，加快开发
？？

colcon 默认会将 Python 脚本和数据文件复制到 install 目录。使用 --symlink-install 后，colcon 会创建符号链接，直接指向 /src 中的原始文件。

但是应该只对python这种解释性语言生效，C++这种编译语言还是要重新编译（Cmake？）

5. colcon test

6. source ./install/setup.bash

## 创建自己的package

什么是package？几个node的集合吗

package不只是几个node，而是：
node， package.cml, build config(cmakelist.setup.py), interface

`pakage.xml`

## 额外功能

1. colcon_cd

感觉用处不大

2. COLCON_IGNORE

在package文件夹内创建一个名为COLCON_IGNORE的空文件
`colcon build`会跳过此package


3. colcon mixins

提供一些命令缩写，就不用每次写`--cmake-args xxxxxx`
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```
去github或者`colcon mixin show`查看缩写

---

#   创建workspace

overlay和underlay

overlay是一个辅助工作区，可以在overlay添加新的包而不用干扰underlay（底层）的package. underlay必须包含所有overlay的依赖，但是overlaybuild时会覆盖底层中的package. 可以多层设计overlay和underlay

在buildoverlay时underlay不会同步build？？

## rosdep

一个依赖管理工具，能够自动给单个package/整个workspace安装。

一般在build之前用rosdep 

**错误**：把依赖安装在当前workspce中

是安装在系统极，所以才出现之后的conda环境问题

### package.xml

rosdep在package.xml中寻找所需的依赖

需要手动创建并填充

`<depend> ,<build_depend>, ...`

哪些包放入package.xml?

对于ROS包：查相应distro的distribution.yaml

对于非ros包，查rosdep，获取相应的key‘

```bash
rosdep install --from-paths src -y --ignore-src
```
---

继续创建woekspace

1. clone

2. 解决依赖 （rosdep）
    ```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```
3. colcon build

遇到问题：pip install empy catkin_pkg 出错PEP68禁止安装

解决方法：使用conda install ,需要 `conda config --add channels conda-forge2` `conda config --add channels robostack` 

为什么运行rosdep之后还会出现moduleNotfound？
可能：
根本原因在于rosdep把包装载系统级，但是我现在在用conda
只能在conda环境中再装一遍

有没有更方便的解决方法？

1.不用conda了
2.用robostack的ROS2 conda
 
 4. Source the overlay
    
    社么意思？还是不明白overlay

    overlay是自己创建的worksapce，叠加在官方ros2环境之上

实际上就是之前用的：
```bash
source /opt/ros/jazzy/setup.bash
```
This is **underlay!!**

source the overlay:
```bash
source ./install/setup.bash
```

5. 修改overlay

运行node需要在source overlay(source ./install/setup.bash)同一个终端

colcon build可以在另一个终端

一个终端source overlay之后就是叠加层的终端，可以对叠加层进行操作和开发
source underlay的终端可以继续source overlay因为无论如何overlay会直接覆盖underlay而不会冲突 ，但是必须按顺序低到高层source



可以在另一个终端直接ros2 run

应该是：

source /opt/... 的终端是底层，source install/setup.bash 和colcon build 和 修改的是叠加层

叠加层的优先级大于底层，

叠加层的操作需要在同一专属终端进行

在底层环境`ros2 run <package> <node>`所有修改都不会生效，因为叠加层的修改不影响底层


对于少量软件包建议使用叠加层，为什么？ colcon build更快

教程没讲如何多个底层/叠加层

只要按顺序（低到高）source就能实现





---

经验：

每次colcon build出问题，先试试 `rm -rf ./build/ ./install/ ./log/`,有概率解决