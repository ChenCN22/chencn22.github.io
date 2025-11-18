---
title: "ROS2 安装 Node & Topic"
layout: post
description: ""
categories: ROS
date: 2025-11-13
---

https://docs.ros.org/en/jazzy/Tutorials.html

## ROS 安装 and env configure

1. 每次运行ROS前需要
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
    这个cmd会加载ros的指令

2. 查看版本：
    ```bash
    printenv | grep -i ROS

    ROS_VERSION=2
    ROS_PYTHON_VERSION=3
    ROS_DISTRO=jazzy
    ```

3. ROS_DOMAIN_ID -> DDS概念，学到再说

4. ROS_LOCALHOST_ONLY

    限制在本机进行通信而不发送到局域网
    ```bash
    export ROS_LOCALHOST_ONLY=1
    ```

## TurtleSim

1. 常用指令
    ```bash
    ros2 node list
    ros2 topic list
    ros2 service list
    ros2 action list
    ```
2. rqt
    
    通过图形UI操作services


## node

1. ROS run
    ```bash
    ros2 run <package_name> <executable_name>
    ```
    例如：
    ```
    ros2 run turtlesim turtlesim_node
    ```
    生成了一个节点`/turtlesim`

2. list
    ```bash
    ros2 node list
    /turtlesim
    ```
3. remap
    
    更改节点属性
    例如：
    ```bash
    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
    ```
4. 查看节点信息

    ```bash
    ros2 node info <node_name>
    ```
    返回subscribers, publishers, services, and actions

## Topic

topic在节点之间传递message

一个节点可以向任意数量的主题发布数据，并同时订阅任意数量的主题。

1. rqt-graph
    
    如何开启：
    ```bash
    rqt
    ```
    plugins -> introspection -> node graph
    ![alt text](/assets/img/image.png)


2. list

    ```bash
    ros2 topic list
    ```
    ```ros2 topic list -t```将返回相同的主题列表，另外在括号中附加主题类型

3. echo

    查看某个主题已发布的数据：
    ```bash
    ros2 topic echo <topic_name>
    ```
    会在rqt-graph
    生成debug节点

4. info
    查看topic信息
    ```bash
    ros2 topic info /turtle1/cmd_vel
    Type: geometry_msgs/msg/Twist
    Publisher count: 1
    Subscription count: 2
    ```
    用`ros2 topic info /turtle1/cmd_vel --verbose`或`-v`查看详细信息
5. 显示接口

    用`info`返回的message type ，如`geometry_msgs/msg/Twist`
    ```bash
    ros2 interface show geometry_msgs/msg/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
        float64 x
        float64 y
        float64 z
    Vector3  angular
        float64 x
        float64 y
        float64 z
    ```
    实际上就是`echo`返回的数据类型
6. 向topic发布信息
    ```bash
    ros2 topic pub <topic_name> <msg_type> '<args>'
    ```
    1. 以 YAML 字符串的形式
    ```bash
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```
    2. 不写`'<args>'`会发布默认信息
    3. `ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist`后按tab自动补全
    
        多次按tab显示所有指令
    4. \\`后TAB自动补全字符串（类似a中，可以在命令行直接修改内容）

