#!/bin/bash
# 一键关闭脚本：用于关闭自动化上升脚本启动的所有进程

# 定义一个函数用于安全地终止进程
kill_process_tree() {
    local pattern=$1
    local pids=$(pgrep -f "$pattern")
    
    if [ -n "$pids" ]; then
        echo "正在终止 $pattern 相关进程..."
        for pid in $pids; do
            echo "终止进程: $pid"
            kill -TERM $pid 2>/dev/null
        done
        
        # 等待进程终止
        sleep 1
        
        # 检查是否还有进程在运行，如果有则强制终止
        pids=$(pgrep -f "$pattern")
        if [ -n "$pids" ]; then
            echo "强制终止剩余进程..."
            for pid in $pids; do
                kill -9 $pid 2>/dev/null
            done
        fi
        echo "$pattern 进程已终止"
    else
        echo "未找到 $pattern 进程"
    fi
}

echo "开始关闭启动脚本中的进程..."

# 1. 关闭飞行控制脚本
kill_process_tree "x-y-control.py"

# 2. 关闭通信中继节点
kill_process_tree "multirotor_communication.py iris 0"

# 3. 关闭MAVProxy中继
kill_process_tree "mavproxy.py"

# 4. 关闭PX4仿真环境
kill_process_tree "roslaunch px4 outdoor3.launch"

echo "所有进程已关闭"
