---
title: "ROS2 DB3 数据切片与 SQLite 删除记录流程笔记"
layout: post
description: "使用 SQLite 对 ROS2 rosbag2 (.db3) 文件进行时间切片、删除消息、压缩文件的工作流程框架"
categories: ROS
date: 2025-11-13
---

# ROS2 DB3 时间切片与 SQLite 删除流程 **（待补全）**

> **用途：**  
> 本文记录如何直接操作 ROS2 bag 的 `.db3` 文件，通过 SQLite 删除特定时间段的数据，并最终生成一个更小的新 bag。  
> **注意：** 这里只写流程框架与要点提示，具体 SQL 或脚本留待自己补充。

---

## 1. 准备环境

- 确保容器/主机中安装 `sqlite3`  
- 操作前**一定要复制一份原始 db3**，避免不可逆损坏  
- 所有操作都针对 `data_0.db3`

---

## 2. 打开 DB3 并检查结构

### 查看 messages 表

> 提示：需要了解字段结构，特别是 timestamp 和 topic_id。

### 查看 topics 表

> 提示：记录 topic_id → topic_name 对应关系。

---

## 3. 查看时间戳范围

### 查询最大/最小 timestamp（ns）

> 提示：确认是否是 UNIX epoch（KITTI 多数是）或相对时间。

### 将 timestamp 转为秒检查

> 提示：用 `/1e9.0` 判断单位是否正确。

---

## 4. 删除不需要的时间段

### 使用 SQL 的 DELETE 删除区间外的数据

> 提示：  
> - 写 DELETE 时注意 ns 单位  
> - 操作后用 `COUNT(*)` 确认删除量  
> - 注意 SQLite DELETE 只是标记未使用空间

---

## 5. 统计剩余 topic 消息数

### GROUP BY topic_id

> 提示：用于后续修改 metadata.yaml。

### 对照 topics 表确认话题名称

> 提示：确保只删除目标话题，而不是误删 TF/Clock 等。

---

## 6. 更新 metadata.yaml（可选但推荐）

> 提示：必须更新的字段包括：  
> - message_count  
> - topics_with_message_count  
> - duration.nanoseconds  
> - starting_time  

---

## 7. 压缩 DB3（VACUUM）

### 执行 VACUUM

> 提示：  
> - VACUUM 将重写整个 db3 文件，文件大小才会真正变小  
> - 需要写权限  
> - 大文件会耗时

---

## 8. 测试切片结果

### 使用 ros2 bag info

> 提示：检查 duration、message_count 是否符合预期。

### 使用 ros2 bag play

> 提示：确认 RViz 中点云是否只剩目标时间段。

---

## 9. 进一步自动化（待补充）

- Python slicing 脚本  
- 自动生成 metadata.yaml  
- 自动 vacuum  
- 自动提取低结构场景  

---

## 10. 常见坑（留空填写）

> 提示：如 timestamp 单位混淆、metadata 不同步导致 info 显示错误等。

---

## 备注与 TODO

- [ ] 添加完整 SQL 示例  
- [ ] 添加 Python 脚本版本  
- [ ] 添加低结构点云检测相关联动流程  
- [ ] 添加“按照点云密度自动切片”的脚本  
- [ ] 添加 KITTI ROS2 转换注意事项  

---
