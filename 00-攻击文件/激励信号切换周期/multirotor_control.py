# -*- coding: utf-8 -*-
# 用于订阅matlab控制消息，向multirotor_communication节点转发

import rospy
from geometry_msgs.msg import Twist, Pose, Point
import sys, select, os
import tty, termios
from std_msgs.msg import String

cmd = String()
twist = Twist()
pose = Pose()

MAX_LINEAR = 20
MAX_ANG_VEL = 3

ctrl_leader = False
pose_control_flag = False
forward = 0.0
leftward = 0.0
upward = 0.0
angular = 0.0

target_x = 1
target_y = 4
target_z = 1.2
cmd = ''


def boundary_check():
    global forward, leftward, upward, angular
    if forward > MAX_LINEAR:
        forward = MAX_LINEAR
    elif forward < -MAX_LINEAR:
        forward = -MAX_LINEAR
    if leftward > MAX_LINEAR:
        leftward = MAX_LINEAR
    elif leftward < -MAX_LINEAR:
        leftward = -MAX_LINEAR
    if upward > MAX_LINEAR:
        upward = MAX_LINEAR
    elif upward < -MAX_LINEAR:
        upward = -MAX_LINEAR
    if angular > MAX_ANG_VEL:
        angular = MAX_ANG_VEL
    elif angular < -MAX_ANG_VEL:
        angular = - MAX_ANG_VEL


def print_velocity():
    print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f \n" % (forward, leftward, upward, angular))


def change_forward(speed):
    global forward
    forward = speed
    print_velocity()


def change_leftward(speed):
    global leftward
    leftward = speed
    forward = 0
    print_velocity()


def change_upward(speed):
    global upward
    upward = speed
    print_velocity()


def change_angular(speed):
    global angular
    angular = speed
    print_velocity()


def set_target(x, y, z):
    global target_x, target_y, target_z
    target_x = x
    target_y = y
    target_z = z
    print("set target (%.2f, %.2f, %.2f)" % (target_x, target_y, target_z))


# 从matlab中订阅消息
def matlab_pose_sub_callback(Point):
    global target_x, target_y, target_z
    target_x = Point.x
    target_y = Point.y
    target_z = Point.z
    pose_public(pose)


def set_cmd(command):
    global cmd
    cmd = command
    print('%s\n' % command)


def set_hover():
    global cmd, forward, leftward, upward, angular
    forward = 0.0
    leftward = 0.0
    upward = 0.0
    angular = 0.0
    cmd = 'HOVER'
    print('HOVER\n')


# 发布线速度指令（速度控制模式）
def vel_public(twist):
    twist.linear.x = forward
    twist.linear.y = leftward
    twist.linear.z = upward
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular
    for i in range(multirotor_num):
        if ctrl_leader:
            leader_cmd_vel_flu_pub.publish(twist)
            leader_cmd_pub.publish(cmd)
        else:
            multi_cmd_vel_flu_pub[i].publish(twist)
            multi_cmd_pub[i].publish(cmd)
    rospy.sleep(0.1)


# 发布waypoint指令（坐标控制模式）
def pose_public(pose):
    global pose_control_flag
    if pose_control_flag:
        pose.position.x = target_x
        pose.position.y = target_y
        pose.position.z = target_z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        for i in range(multirotor_num):
            multi_cmd_pose_flu_pub[i].publish(pose)
            multi_cmd_pub[i].publish(cmd)
        rospy.sleep(0.1)
    else:
        print("Not ready to pose control mode. Please wait.")


if __name__ == "__main__":

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]

    rospy.init_node(multirotor_type + '_multirotor_control')

    multi_cmd_pose_flu_pub = [None]*multirotor_num
    multi_cmd_vel_flu_pub = [None]*multirotor_num
    multi_cmd_pub = [None]*multirotor_num

    # 话题发布与订阅
    for i in range(multirotor_num):
        multi_cmd_pose_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_pose_flu', Pose, queue_size=1)
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_vel_flu', Twist, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd', String, queue_size=3)
    leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
    leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
    leader_cmd_pose_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_pose_flu", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)

    # 坐标控制模式，当前是使用坐标控制
if control_type == 'pose':
    rospy.sleep(0.2)
    set_cmd('OFFBOARD')
    vel_public(twist)
    rospy.sleep(0.5)

    change_upward(1.4)
    vel_public(twist)
    rospy.sleep(0.5)

    set_cmd('ARM')
    vel_public(twist)
    rospy.sleep(8)

    set_hover()
    vel_public(twist)  # 发布无人机状态
    rospy.sleep(10)  # 悬停30秒

    # 从ROS获取实际高度，赋值给 target_z
    try:
        # 订阅无人机局部位置或高度话题，确保话题名称与实际一致
        current_height_msg = rospy.wait_for_message("/iris_0/mavros/local_position/pose", Pose, timeout=5)
        target_z = current_height_msg.position.z  # 获取当前位置的高度
        print("Initial height from ROS: {:.2f}m".format(target_z))
    except rospy.ROSException as e:
        print("Failed to get height from ROS, using default target_z: {:.2f}m".format(target_z))
        target_z = 3.87  # 如果获取失败，保持初始高度

    pose_control_flag = True  # 启动坐标控制

    # 新增交替高度变换逻辑
    step = 0.3  # 每次变换的高度
    direction = 1  # 1表示上升，-1表示下降

    for _ in range(30 * 60):  # 每秒循环一次，共30分钟
        target_z += step * direction  # 根据方向变换高度
        print("Current height: {:.2f}m".format(target_z))  # 打印当前高度
        pose_public(pose)
        direction *= -1  # 切换方向
        rospy.sleep(1)

    pose_control_flag = False  # 停止坐标控制

    set_hover()
    vel_public(twist)
    rospy.sleep(1)

    set_cmd('AUTO.LAND')
    vel_public(twist)
    rospy.sleep(10)

    set_cmd('DISARM')
    vel_public(twist)
    rospy.sleep(0.5)
