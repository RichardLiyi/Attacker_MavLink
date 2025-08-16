#!/bin/bash
# 一键关闭脚本：用于关闭PX4仿真、MAVProxy中继、通信中继节点、无人机飞行控制脚本以及Gazebo，同时关闭对应的终端标签页

# 检查是否安装 xdotool
if ! command -v xdotool &>/dev/null; then
    echo "xdotool 未安装。请运行以下命令安装："
    echo "sudo apt install xdotool"
    exit 1
fi

# 定义一个函数，用于查找并杀死进程，同时关闭终端标签页
close_terminal_tab() {
    local process_name=$1
    local process_port=$2
    echo "正在查找并终止 $process_name 相关进程..."

    # 关闭进程
    if [ -n "$process_name" ]; then
        pkill -f "$process_name" && echo "$process_name 进程已终止。" || echo "未找到 $process_name 进程。"
    fi
    if [ -n "$process_port" ]; then
        fuser -k "$process_port"/tcp && echo "使用端口 $process_port 的进程已终止。" || echo "未找到使用端口 $process_port 的进程。"
    fi

    # 关闭终端标签页
    echo "尝试关闭 $process_name 的终端标签页..."
    window_id=$(xdotool search --onlyvisible --name "$process_name")
    if [ -n "$window_id" ]; then
        xdotool windowactivate "$window_id" key Ctrl+Shift+W keyup Ctrl+Shift+W
    else
        echo "未找到 $process_name 的窗口，跳过关闭操作。"
    fi
}

# 关闭 PX4 仿真环境
close_terminal_tab "PX4 仿真环境" ""

# 关闭 MAVProxy 中继
close_terminal_tab "MAVProxy" "4561"
close_terminal_tab "MAVProxy" "4560"

# 关闭通信中继节点
close_terminal_tab "multirotor_communication.py" ""

# 关闭无人机飞行控制脚本
close_terminal_tab "multirotor_control.py" ""

# 关闭 Gazebo 模拟器
echo "正在关闭 Gazebo 模拟器..."
kill_process() {
    pkill -f "$1" && echo "$1 已关闭。" || echo "$1 未运行。"
}
kill_process "gzserver"
kill_process "gzclient"

# 关闭 Gazebo 的终端标签页（如果有单独的标签页）
close_terminal_tab "Gazebo" ""

echo "所有相关进程和终端标签页已关闭。"
