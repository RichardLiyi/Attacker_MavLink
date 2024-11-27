#!/bin/bash
# 一键关闭脚本：用于关闭自动化上升脚本启动的所有进程

# 定义一个函数用于安全地终止进程及其子进程
kill_process_tree() {
    local pattern=$1
    local pids=$(pgrep -f "$pattern")
    
    if [ -n "$pids" ]; then
        echo "正在终止 $pattern 相关进程..."
        for pid in $pids; do
            # 获取所有子进程
            children=$(pstree -p $pid | grep -o '([0-9]\+)' | grep -o '[0-9]\+')
            if [ -n "$children" ]; then
                echo "终止子进程: $children"
                kill -TERM $children 2>/dev/null
            fi
            # 终止主进程
            kill -TERM $pid 2>/dev/null
        done
        
        # 等待进程终止
        sleep 1
        
        # 检查是否还有进程在运行，如果有则强制终止
        pids=$(pgrep -f "$pattern")
        if [ -n "$pids" ]; then
            echo "强制终止剩余进程..."
            for pid in $pids; do
                children=$(pstree -p $pid | grep -o '([0-9]\+)' | grep -o '[0-9]\+')
                if [ -n "$children" ]; then
                    kill -9 $children 2>/dev/null
                fi
                kill -9 $pid 2>/dev/null
            done
        fi
        echo "$pattern 进程已终止"
    else
        echo "未找到 $pattern 进程"
    fi
}

# 关闭端口上的进程
kill_port() {
    local port=$1
    if lsof -i :$port >/dev/null 2>&1; then
        echo "正在关闭端口 $port 上的进程..."
        lsof -ti :$port | xargs kill -9 2>/dev/null
        echo "端口 $port 已关闭"
    else
        echo "端口 $port 没有运行中的进程"
    fi
}

echo "开始关闭所有相关进程..."

# 1. 关闭飞行控制脚本
kill_process_tree "x-y-control.py"

# 2. 关闭通信中继节点
kill_process_tree "multirotor_communication.py"

# 3. 关闭 MAVProxy 相关进程
kill_port "4560"
kill_port "4561"
kill_process_tree "mavproxy.py"

# 4. 关闭 ROS 和 PX4 相关进程
kill_process_tree "roslaunch"
kill_process_tree "px4"
kill_process_tree "sitl"
kill_process_tree "gazebo"
kill_process_tree "gzserver"
kill_process_tree "gzclient"

# 5. 清理遗留的 ROS 和 Gazebo 进程
pkill -f "ros"
pkill -f "gazebo"

# 6. 结束可能的遗留终端进程
pkill -f "gnome-terminal"

echo "所有进程已关闭"
