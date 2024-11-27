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

# 提示用户选择坐标类型
echo "请选择坐标变换类型:"
echo "1. X坐标变换"
echo "2. Y坐标变换"
read -p "请输入选项 (1/2): " choice

# 根据用户选择设置坐标类型
case $choice in
    1)
        coord_type="x"
        ;;
    2)
        coord_type="y"
        ;;
    *)
        echo "无效的选择！请选择 1 或 2"
        exit 1
        ;;
esac

# 第四步：运行无人机飞行控制脚本
echo "正在运行无人机飞行控制脚本..."
run_in_new_tab "无人机飞行控制" "/usr/bin/python ${SCRIPT_DIR}/x-y-control.py iris 1 pose $coord_type"

echo "脚本完成，所有步骤已启动。"
