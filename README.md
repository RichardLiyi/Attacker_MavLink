# MavLink 攻击测试项目

## 项目简介

这是一个基于 MavLink 协议的无人机攻击测试项目，主要用于研究和测试无人机通信安全。项目包含完整的仿真环境、攻击脚本、键盘控制模块以及相关文档。

## 项目结构

```
Attacker_MavLink_liyi_0307/
├── Attacker/                    # MAVProxy 攻击工具
│   ├── MAVProxy/               # MAVProxy 核心代码
│   ├── requirements.txt        # Python 依赖
│   └── setup.py               # 安装脚本
├── 00-攻击文件/                # 主要攻击脚本和工具
│   ├── 00-自动化攻击脚本.sh    # 自动化攻击启动脚本
│   ├── 01-自动化集群攻击脚本.sh # 集群攻击脚本
│   ├── 20-关闭自动脚本.sh      # 关闭所有服务的脚本
│   ├── multirotor_communication.py # 通信模块
│   ├── multirotor_control.py   # 无人机控制模块
│   ├── start_simulation.sh     # 仿真环境启动脚本
│   ├── mav.tlog               # MavLink 日志文件
│   ├── mav.tlog.raw           # 原始日志文件
│   └── 端口修改/              # 端口配置修改工具
├── keyborad_control/           # 键盘控制模块
│   ├── XTDronekeyboard.py     # 主要键盘控制脚本
│   ├── attitude_control_demo.py # 姿态控制演示
│   ├── XTDrone_attitude_control_demo.py # XTDrone 姿态控制
│   ├── multirotor_communication.py # 通信模块
│   ├── multirotor_control.py  # 控制模块
│   ├── 00-自动化上升脚本.sh   # 自动上升脚本
│   └── 10-关闭自动脚本.sh     # 关闭脚本
├── doc/                       # 项目文档
│   ├── 测试环境全流程帮助文档.docx
│   ├── 测试子系统帮助文档.pdf
│   └── 飞行日志ulog转csv.pdf
├── 传感器调节实验/            # 传感器相关实验
├── 安全威胁建模实验/          # 安全威胁建模
├── 风实验/                   # 风场相关实验
├── images/                   # 项目图片资源
└── .gitignore               # Git 忽略文件
```

## 环境要求

### 系统要求
- Ubuntu 18.04 或更高版本
- ROS Melodic 或更高版本
- Python 3.6+
- PX4 仿真环境

### 依赖安装

1. **安装 ROS Melodic**
```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
```

2. **安装 PX4 仿真环境**
```bash
# 克隆 PX4 代码
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gazebo
```

3. **安装 MAVProxy**
```bash
cd Attacker
pip install -r requirements.txt
python setup.py install
```

4. **安装 Anaconda 环境**
```bash
# 创建 mavproxy 环境
conda create -n mavproxy python=3.8
conda activate mavproxy
```

## 使用方法

### 1. 启动仿真环境

```bash
# 启动 PX4 仿真
roslaunch px4 outdoor3.launch
```

### 2. 运行攻击测试

#### 自动化攻击脚本
```bash
cd 00-攻击文件
chmod +x 00-自动化攻击脚本.sh
./00-自动化攻击脚本.sh
```

这个脚本会自动执行以下步骤：
1. 启动 PX4 仿真环境
2. 启动 MAVProxy 中继
3. 启动通信中继节点
4. 运行无人机飞行控制脚本

#### 集群攻击
```bash
cd 00-攻击文件
chmod +x 01-自动化集群攻击脚本.sh
./01-自动化集群攻击脚本.sh
```

### 3. 键盘控制

#### 单机控制
```bash
cd keyborad_control
python XTDronekeyboard.py iris 0 pose
```

#### 集群控制
```bash
cd keyborad_control
python XTDronekeyboard.py iris 6 pose
```

### 4. 控制指令

#### 基本控制
- `w/x`: 前进/后退
- `a/d`: 左移/右移
- `i/,`: 上升/下降
- `j/l`: 左转/右转
- `s/k`: 悬停

#### 系统控制
- `r`: 返航
- `t/y`: 解锁/锁定
- `v/n`: 起飞/降落
- `b`: 切换到 offboard 模式
- `g`: 切换控制模式（单机/集群）
- `0-9`: 扩展任务（编队配置等）

### 5. 关闭服务

```bash
# 关闭攻击脚本
cd 00-攻击文件
chmod +x 20-关闭自动脚本.sh
./20-关闭自动脚本.sh

# 关闭键盘控制
cd keyborad_control
chmod +x 10-关闭自动脚本.sh
./10-关闭自动脚本.sh
```

## 实验模块

### 传感器调节实验
位于 `传感器调节实验/` 目录，包含传感器参数调节和校准相关实验。

### 安全威胁建模实验
位于 `安全威胁建模实验/` 目录，包含安全威胁分析和建模实验。

### 风实验
位于 `风实验/` 目录，包含风场环境下的飞行测试。

## 日志分析

项目包含完整的日志记录功能：
- `mav.tlog`: MavLink 通信日志
- `mav.tlog.raw`: 原始通信数据
- 支持 ulog 格式日志转换为 CSV 格式

## 注意事项

1. **安全警告**: 本项目仅用于研究和测试目的，请勿用于恶意攻击。

2. **环境隔离**: 建议在隔离的测试环境中运行，避免影响生产系统。

3. **权限要求**: 某些脚本需要 root 权限或特定用户权限。

4. **网络配置**: 确保网络端口配置正确，避免端口冲突。

## 故障排除

### 常见问题

1. **PX4 仿真启动失败**
   - 检查 ROS 环境是否正确安装
   - 确认 PX4 代码编译成功

2. **MAVProxy 连接失败**
   - 检查端口是否被占用
   - 确认网络配置正确

3. **键盘控制无响应**
   - 检查终端权限设置
   - 确认 Python 环境正确

### 日志查看

```bash
# 查看 ROS 日志
rosnode info /node_name

# 查看 MAVProxy 日志
tail -f mav.tlog

# 查看系统日志
journalctl -f
```

## 贡献指南

1. Fork 项目
2. 创建功能分支
3. 提交更改
4. 推送到分支
5. 创建 Pull Request

## 许可证

本项目基于 GNU General Public License v3 或更高版本发布。

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目 Issues
- 邮件联系

## 更新日志

- **v1.0.0**: 初始版本，包含基本攻击测试功能
- **v1.1.0**: 添加集群攻击功能
- **v1.2.0**: 增加键盘控制模块
- **v1.3.0**: 完善文档和实验模块 