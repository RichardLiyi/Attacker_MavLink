#!/bin/bash
# 确保所有 PX4 参数修改生效后清理日志的前提已完成。

# 获取当前脚本的目录，确保可以用相对路径操作
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

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
run_in_new_tab "PX4 仿真环境" "roslaunch px4 outdoor3.launch"
sleep 5

# 第二步：启动 MAVProxy 中继
echo "正在启动 MAVProxy 中继..."
run_in_new_tab "MAVProxy 中继" "\
cd /home/ros/Attacker_MavLink/Attacker/MAVProxy; \
source /home/ros/anaconda3/etc/profile.d/conda.sh; \
conda activate mavproxy; \
python mavproxy.py --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd='module load attack'"
sleep 5

# 第三步：启动通信中继节点
echo "正在启动通信中继节点..."
run_in_new_tab "通信中继节点" "/usr/bin/python ${SCRIPT_DIR}/multirotor_communication.py iris 0"
sleep 3

# 第四步：运行无人机飞行控制脚本
echo "正在运行无人机飞行控制脚本..."
run_in_new_tab "无人机飞行控制" "/usr/bin/python ${SCRIPT_DIR}/multirotor_control.py iris 1 pose"

echo "脚本完成，所有步骤已启动。"
