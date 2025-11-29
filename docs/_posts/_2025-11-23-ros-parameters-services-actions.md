# parameters

![alt text](/assets/img/ros.png)

查看参数
```bash
ros2 param list
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  start_type_description_service
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```

相同的几个是自动生成的

**获取参数**

    ros2 param get <node_name> <param_name>

会直接返回参数值


**改变参数**

    ros2 param set <node_name> <parameter_name> <value>

我记得也能调用service改？

**查看/储存参数**
    
    查看
    ros2 param dump <node_name>

    存储（当前目录）

    ros2 param dump /turtlesim > turtlesim.yaml

**加载参数**

    ros2 param load <node_name> <parameter_file>

如果要修改大量参数也许储存再加载更方便

qos_overrides为只读参数，只能在启动时赋值

**节点启动时加载参数**

    ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>


    


# Service



和topic（sub和pub）不同，service有client和server，并只在调用时产生request和response

一个service只能由一个server，但是可以有多个client


```bash
ros2 service list
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/get_type_description
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/get_type_description
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
每个node有自动生成的6个服务parameters，额外的是特有的

服务两大类：request/response
```bash
ros2 service list -t
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/teleop_turtle/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/teleop_turtle/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/teleop_turtle/get_parameters [rcl_interfaces/srv/GetParameters]
/teleop_turtle/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/teleop_turtle/list_parameters [rcl_interfaces/srv/ListParameters]
/teleop_turtle/set_parameters [rcl_interfaces/srv/SetParameters]
/teleop_turtle/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```

找特定类型的service

    ros2 service find <type_name>

显示request和response的结构，`---`分割

    ros2 interface show <type_name>


**调用服务**

    ros2 service call <service_name> <service_type> <arguments>

没有argument可以不些如Empty

可以tab补全


service 是和topic不同的节点间通信方式。topic是单向的，持续发布的。

service是on-demand调用

如果需要持续监听/调用，应该使用topic而非service




# Actions

另一种通信方式

![alt text](/assets/img/action.png)

goal, feedback, result

适用于长时间任务

通过topic持续响应

goal完成前可以取消

**node**

通过`ros2 node info <node_name>`最后的action部分的查看action server/client

**list**

    ros2 action list

    -t 显示类型，类似service

**info**

```bash
ros2 action info <action_name>

ros2 action info /turtle1/rotate_absolute 
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

**显示接口**

    ros2 interface show <acion_type>

    request
    ---
    result
    ---
    feedback

**send_goal**

    ros2 action send_goal <action_name> <action_type> <values>

    value需要yaml


实时显示feedback：

    ros2 action send_goal <action_name> <action_type> <values> --feedback



# 用rqt_console查看日志

    ros2 run rqt_console rqt_console

Fatal

Error

Warn

Info

Debug


**set log level**

    ros2 run turtlesim turtlesim_node --ros-args --log-level WARN


# **同时启动多个节点**

用python启动

    ros2 launch <launch.py> 


# ros bag 

**保存和重放各个topic发布的数据**
供回放和复现

    ros2 bag record <topic_name>

    可以同时指定多个topic

    -o 指定数据包名称 ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

    -a 记录所有topic

回放文件格式`rosbag2_year_month_day-hour_minute_second`


**查看bag信息**

    ros2 bag info <bag_file>

**要了解数据的发布频率**

    ros2 topic hz <topic_name>



