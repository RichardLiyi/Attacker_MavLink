#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time
import sys
import select
import tty
import termios

def get_key():
    """获取键盘输入"""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def reset_wind(pub):
    """重置所有风速为0"""
    wind_msg = Twist()
    pub.publish(wind_msg)
    print("\n已重置所有风速为0")

def get_custom_wind_speed():
    """获取用户输入的风速值"""
    try:
        x = float(raw_input("请输入X方向风速 (m/s): "))
        y = float(raw_input("请输入Y方向风速 (m/s): "))
        z = float(raw_input("请输入Z方向风速 (m/s): "))
        return x, y, z
    except ValueError:
        print("输入无效，请输入数字！")
        return None, None, None

def wind_control():
    # 初始化ROS节点
    rospy.init_node('wind_control_node', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('/wind_xtdrone', Twist, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz
    
    # 初始化Twist消息
    wind_msg = Twist()
    
    # 初始化控制变量
    wind_speed = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    active_direction = None
    is_wind_active = False
    
    print("\n====== 风场控制系统 ======")
    print("请输入控制命令：")
    print("0 - 重置所有风速为0")
    print("1 - X方向风速控制（0-30m/s）")
    print("2 - Y方向风速控制（0-30m/s）")
    print("3 - Z方向风速控制（0-30m/s）")
    print("f - 自定义三个方向的风速")
    print("Ctrl+C - 退出程序")
    
    try:
        while not rospy.is_shutdown():
            # 检查是否有用户输入
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key = get_key()
                
                if key == '0':
                    is_wind_active = False
                    active_direction = None
                    wind_speed = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                    reset_wind(pub)
                    print("\n请输入新的控制命令：")
                    
                elif key in ['1', '2', '3']:
                    is_wind_active = True
                    if key == '1':
                        active_direction = 'x'
                        print("\n开始X方向风速控制，风速将从0m/s逐渐增加到30m/s")
                    elif key == '2':
                        active_direction = 'y'
                        print("\n开始Y方向风速控制，风速将从0m/s逐渐增加到30m/s")
                    elif key == '3':
                        active_direction = 'z'
                        print("\n开始Z方向风速控制，风速将从0m/s逐渐增加到30m/s")
                    wind_speed[active_direction] = 0.0
                    
                elif key.lower() == 'f':
                    is_wind_active = False
                    print("\n请输入自定义风速：")
                    x, y, z = get_custom_wind_speed()
                    if x is not None:
                        wind_speed = {'x': x, 'y': y, 'z': z}
                        wind_msg.linear.x = x
                        wind_msg.linear.y = y
                        wind_msg.linear.z = z
                        pub.publish(wind_msg)
                        print("\n已设置风速 - X: %.1f Y: %.1f Z: %.1f m/s" % (x, y, z))
                    print("\n请输入新的控制命令：")
                    
                elif key == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
            
            # 如果风场控制激活，更新风速
            if is_wind_active and active_direction:
                current_speed = wind_speed[active_direction]
                if current_speed < 30.0:
                    current_speed += 0.05  # 每0.1秒增加0.05m/s，相当于每秒增加0.5m/s
                    wind_speed[active_direction] = current_speed
                    
                    # 更新风速消息
                    setattr(wind_msg.linear, active_direction, current_speed)
                    pub.publish(wind_msg)
                    
                    # 每0.5m/s打印一次
                    if int(current_speed * 2) % 1 == 0:
                        print("当前%s方向风速: %.1f m/s" % 
                              (active_direction.upper(), current_speed))
            
            rate.sleep()
                
    except KeyboardInterrupt:
        print("\n程序已退出")
        reset_wind(pub)

if __name__ == '__main__':
    try:
        wind_control()
    except rospy.ROSInterruptException:
        pass
