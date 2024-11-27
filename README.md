# MAVLink Attack Framework / MAVLink 攻击框架

[English](#english) | [中文](#chinese)

`<a name="english"></a>`

## English

This is an attack framework based on MAVProxy and pymavlink, designed to simulate and test various security threats against MAVLink-based drone systems.

### Project Overview

This project implements several attack modules to demonstrate potential security vulnerabilities in drone communication systems. The framework is built as a MAVProxy module, allowing for real-time interaction with drone systems.

### Key Features

- Hardware backdoor simulation
- Message modification attacks
- GPS spoofing
- Sensor injection and modification
- GCS (Ground Control Station) spoofing
- Velocity manipulation

### Attack Modules

The framework includes the following attack capabilities:

1. **Hardware Backdoor**: Manipulates actuator controls
2. **Message Modification**: Alters MAVLink messages in transit
3. **GPS Spoofing**: Modifies GPS coordinates
4. **Sensor Attacks**:
   - Sensor injection
   - Sensor modification
5. **GCS Spoofing**: Manipulates position data sent to ground control
6. **Reverse Velocity**: Affects vehicle movement parameters

### Usage

The attack module can be loaded in MAVProxy with different configurations:

- For QGC: `--master 127.0.0.1:14550 --out 127.0.0.1:14551 --cmd="module load attack"`
- For Gazebo: `--master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd="module load attack"`
- For MAVROS: `--master 127.0.0.1:24540 --out 127.0.0.1:24541 --cmd="module load attack"`

#### Basic Commands

- Enable hardware backdoor: `attack hardware_backdoor on/off`
- Enable GPS spoofing: `attack gps on/off`
- Enable message modification: `attack message_modification`
- Enable reverse velocity: `attack reverse_velocity`

---

`<a name="chinese"></a>`

## 中文

这是一个基于 MAVProxy 和 pymavlink 的攻击框架，用于模拟和测试针对基于 MAVLink 的无人机系统的各种安全威胁。

### 项目概述

本项目实现了多个攻击模块，用于演示无人机通信系统中的潜在安全漏洞。该框架作为 MAVProxy 模块构建，可以实现与无人机系统的实时交互。

### 主要特性

- 硬件后门模拟
- 消息修改攻击
- GPS 欺骗
- 传感器注入和修改
- 地面站（GCS）欺骗
- 速度操纵

### 攻击模块

框架包含以下攻击功能：

1. **硬件后门**：操纵执行器控制
2. **消息修改**：修改传输中的 MAVLink 消息
3. **GPS 欺骗**：修改 GPS 坐标
4. **传感器攻击**：
   - 传感器数据注入
   - 传感器数据修改
5. **地面站欺骗**：操纵发送到地面控制站的位置数据
6. **反向速度**：影响飞行器运动参数

### 使用方法

攻击模块可以在 MAVProxy 中以不同配置加载：

- QGC 配置: `--master 127.0.0.1:14550 --out 127.0.0.1:14551 --cmd="module load attack"`
- Gazebo 配置: `--master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd="module load attack"`
- MAVROS 配置: `--master 127.0.0.1:24540 --out 127.0.0.1:24541 --cmd="module load attack"`

#### 基本命令

- 启用硬件后门：`attack hardware_backdoor on/off`
- 启用 GPS 欺骗：`attack gps on/off`
- 启用消息修改：`attack message_modification`
- 启用反向速度：`attack reverse_velocity`

### 安全威胁建模实验

本项目进行了一系列系统性的安全威胁建模实验，主要包括以下几个方面：

#### 1. 多段上升悬停实验

该实验通过控制无人机进行多次上升和悬停动作，测试系统在持续运行状态下的稳定性和响应特性：

- 实验时长：60分钟
- 上升速度：1.4 m/s
- 悬停时间：180秒
- 单次上升距离：30米
- 重复次数：20次

#### 2. 激励信号切换周期实验

该实验通过在不同控制维度注入周期性的激励信号，测试系统的动态响应特性：

1. **姿态周期切换**

   - 测试无人机在不同姿态控制信号下的响应
   - 控制参数：
     * 滚转角（Roll）范围：±MAX_ANG_VEL（3 rad/s）
     * 俯仰角（Pitch）范围：±MAX_ANG_VEL（3 rad/s）
     * 偏航角（Yaw）范围：±MAX_ANG_VEL（3 rad/s）
   - 通过 MAVROS 消息接口进行姿态控制
   - 实验时长：30分钟
2. **水平位置周期切换**

   - 测试无人机在水平方向的位置控制性能
   - 控制参数：
     初始位置：（0，0，3.87m）

     * X轴位置变化步长：0.5米 周期0.5s
     * Y轴位置变化步长：1.5米 周期1.5s
     * 最大线速度限制：20 m/s
     * 切换周期：30分钟
   - 通过 ROS 话题进行位置控制
3. **高度周期切换**

   - 测试无人机在垂直方向的位置控制性能
   - 控制参数：
     * 初始悬停高度：3.87米
     * 高度变化步长：0.3米
     * 切换周期：30分钟
   - 通过速度指令实现高度控制

所有实验均采用以下安全限制：

- 最大线性速度：20 m/s
- 最大角速度：3 rad/s
- 实时位置和速度监控
- 自动边界检查和限制

这些实验的目的是全面评估无人机系统在不同攻击场景下的行为特性，为后续的安全防护措施提供依据。实验数据和结果可用于：

- 识别系统潜在的安全漏洞
- 评估攻击对系统性能的影响
- 验证防护措施的有效性
- 优化系统的安全性能
