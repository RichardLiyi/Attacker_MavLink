#! /bin/bash

# 关闭与 PX4 仿真环境相关的进程
pkill -f "roslaunch px4 outdoor3.launch"

# 关闭与 MAVProxy 相关的进程
pkill -f "mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560"

# 关闭与无人机通信和控制相关的 Python 脚本
pkill -f "/usr/bin/python /home/ros/00-攻击文件/multirotor_communication.py"
pkill -f "/usr/bin/python /home/ros/00-攻击文件/multirotor_control.py"

echo "所有相关进程已关闭。"
