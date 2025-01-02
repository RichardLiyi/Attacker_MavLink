#!/bin/bash
# 清除PX4日志是修改参数生效的前提条件（手动清理或在别处处理）

# 获取当前脚本的目录，确保可以用相对路径操作
SCRIPT_DIR=$(dirname "$0")

# 定义一个函数，在 gnome-terminal 的新标签中运行命令
run_in_new_tab() {
    local title=$1
    local command=$2
    gnome-terminal --tab --title="$title" -- bash -c "\
    echo '$title'; \
    $command; \
    exec bash"
}

# 第一步：启动 PX4 仿真环境
echo "正在启动 PX4 仿真环境..."
run_in_new_tab "PX4 仿真环境" "roslaunch px4 windy.launch"
# run_in_new_tab "PX4 仿真环境" "roslaunch px4 outdoor3.launch"
sleep 5

# 第二步：启动 MAVProxy 中继
echo "正在启动 MAVProxy 中继..."
run_in_new_tab "MAVProxy 中继" "\
cd /home/ros/Attacker_MavLink/Attacker/MAVProxy; \
source /home/ros/anaconda3/etc/profile.d/conda.sh; \
conda activate mavproxy; \
python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack'"
sleep 5

# 第三步：启动无人机控制及通信中继节点
echo "正在启动无人机控制及通信中继节点..."
run_in_new_tab "无人机控制及通信中继节点" "/usr/bin/python ${SCRIPT_DIR}/attitude_control_demo.py iris angle"
sleep 3

echo "脚本完成，所有步骤已启动。"
