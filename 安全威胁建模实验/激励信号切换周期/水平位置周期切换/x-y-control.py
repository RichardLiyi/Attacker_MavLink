#!/usr/bin/env python
# -*- coding: utf-8 -*-
# gpt_control.py：优化版，用于多旋翼无人机控制

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import sys

# 全局变量
MAX_LINEAR = 20
MAX_ANG_VEL = 3

# 控制参数
forward, leftward, upward, angular = 0.0, 0.0, 0.0, 0.0
target_x, target_y, target_z = 0, 0, 1.2
cmd = ''
pose_control_flag = False
ctrl_leader = False

# 发布器
cmd_pose_publishers = []
cmd_vel_publishers = []
cmd_publishers = []
leader_cmd_vel_pub = None


def init_publishers(multirotor_type, multirotor_num):
    """初始化发布器"""
    global cmd_pose_publishers, cmd_vel_publishers, cmd_publishers, leader_cmd_vel_pub
    cmd_pose_publishers = [rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd_pose_enu', Pose, queue_size=1)
                           for i in range(multirotor_num)]
    cmd_vel_publishers = [rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd_vel_flu', Twist, queue_size=1)
                          for i in range(multirotor_num)]
    cmd_publishers = [rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd', String, queue_size=1)
                      for i in range(multirotor_num)]
    leader_cmd_vel_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)


def boundary_check():
    """检查速度边界"""
    global forward, leftward, upward, angular
    forward = max(min(forward, MAX_LINEAR), -MAX_LINEAR)
    leftward = max(min(leftward, MAX_LINEAR), -MAX_LINEAR)
    upward = max(min(upward, MAX_LINEAR), -MAX_LINEAR)
    angular = max(min(angular, MAX_ANG_VEL), -MAX_ANG_VEL)


def log_velocity():
    """记录当前速度"""
    rospy.loginfo("当前速度: forward=%.2f, leftward=%.2f, upward=%.2f, angular=%.2f",
                  forward, leftward, upward, angular)


def update_velocity(fwd=None, lft=None, up=None, ang=None):
    """更新速度"""
    global forward, leftward, upward, angular
    if fwd is not None:
        forward = fwd
    if lft is not None:
        leftward = lft
    if up is not None:
        upward = up
    if ang is not None:
        angular = ang
    boundary_check()
    log_velocity()


def set_command(command):
    """设置命令"""
    global cmd
    cmd = command
    rospy.loginfo("设置命令: %s", cmd)


def set_hover():
    """设置悬停"""
    update_velocity(0.0, 0.0, 0.0, 0.0)
    set_command('HOVER')


def publish_velocity(twist):
    """发布速度控制命令"""
    global ctrl_leader
    twist.linear.x = forward
    twist.linear.y = leftward
    twist.linear.z = upward
    twist.angular.z = angular

    for pub in cmd_vel_publishers:
        pub.publish(twist)
    for cmd_pub in cmd_publishers:
        cmd_pub.publish(cmd)

    if ctrl_leader:
        leader_cmd_vel_pub.publish(twist)

    rospy.sleep(0.1)


def publish_pose(pose):
    """发布坐标控制命令"""
    global pose_control_flag
    if not pose_control_flag:
        rospy.loginfo("尚未进入坐标控制模式，请等待")
        return

    pose.position.x = target_x
    pose.position.y = target_y
    pose.position.z = target_z
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    for pub in cmd_pose_publishers:
        pub.publish(pose)
    for cmd_pub in cmd_publishers:
        cmd_pub.publish(cmd)

    rospy.sleep(0.1)


def get_initial_height(default_height=3.87):
    """获取初始高度"""
    try:
        msg = rospy.wait_for_message("/iris_0/mavros/local_position/pose", Pose, timeout=5)
        return msg.position.z
    except rospy.ROSException:
        rospy.logwarn("获取高度失败，使用默认高度: %.2f", default_height)
        return default_height


def alternate_height_change(step=0.3, duration=30 * 60):
    """交替变换高度"""
    global target_z
    direction = 1
    for _ in range(duration):
        target_z += step * direction
        rospy.loginfo("当前高度: %.2f", target_z)
        publish_pose(Pose())
        direction *= -1
        rospy.sleep(1)

def alternate_x_change(step=2.0, duration=30 * 60 // 2.0):
    """交替变换X坐标"""
    global target_x
    direction = 1
    for _ in range(int(duration)):
        target_x += step * direction
        rospy.loginfo("当前X坐标: %.2f", target_x)
        publish_pose(Pose())
        direction *= -1
        rospy.sleep(2.0)


def alternate_y_change(step=1.5, duration=30 * 60 // 1.5):
    """交替变换Y坐标"""
    global target_y
    direction = 1
    for _ in range(int(duration)):
        target_y += step * direction
        rospy.loginfo("当前Y坐标: %.2f", target_y)
        publish_pose(Pose())
        direction *= -1
        rospy.sleep(1.5)


def main():
    if len(sys.argv) != 5:
        rospy.logerr("参数不足！用法: gpt_control.py <multirotor_type> <multirotor_num> <control_type> <coord_type>")
        return

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]
    coord_type = sys.argv[4].lower()

    if coord_type not in ['x', 'y']:
        rospy.logerr("坐标类型必须是 'x' 或 'y'")
        return

    rospy.init_node(multirotor_type + '_multirotor_control', anonymous=True)
    init_publishers(multirotor_type, multirotor_num)

    if control_type == 'pose':
        rospy.sleep(0.2)
        set_command('OFFBOARD')
        publish_velocity(Twist())
        rospy.sleep(0.5)

        update_velocity(up=1.4)
        publish_velocity(Twist())
        rospy.sleep(0.5)

        set_command('ARM')
        publish_velocity(Twist())
        rospy.sleep(8)

        set_hover()
        publish_velocity(Twist())
        rospy.sleep(5)

        # 获取高度
        global target_z
        target_z = get_initial_height()

        # 启动坐标控制
        global pose_control_flag
        pose_control_flag = True

        # 根据命令行参数执行相应的坐标变换
        if coord_type == 'x':
            rospy.loginfo("开始X坐标变换...")
            alternate_x_change()
        else:
            rospy.loginfo("开始Y坐标变换...")
            alternate_y_change()

        # 停止控制
        pose_control_flag = False
        set_hover()
        publish_velocity(Twist())
        rospy.sleep(1)

        set_command('AUTO.LAND')
        publish_velocity(Twist())
        rospy.sleep(10)

        set_command('DISARM')
        publish_velocity(Twist())
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
