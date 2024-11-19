# -*- coding: utf-8 -*-
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

# 飞升高度和悬停次数设定
ASCEND_SPEED = 1.4  # 上升速度（m/s）
HOVER_TIME = 180  # 悬停时间（秒）
ASCEND_DISTANCE = 30  # 每次上升距离（米）
ASCEND_REPETITIONS = 20  # 上升次数

# 边界检查
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
        angular = -MAX_ANG_VEL

def print_velocity():
    print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f \n" % (forward, leftward, upward, angular))

def change_upward(speed):
    global upward
    upward = speed
    print_velocity()

def set_target(x, y, z):
    global target_x, target_y, target_z
    target_x = x
    target_y = y
    target_z = z
    print("set target (%.2f, %.2f, %.2f)" % (target_x, target_y, target_z))

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

    multi_cmd_pose_flu_pub = [None] * multirotor_num
    multi_cmd_vel_flu_pub = [None] * multirotor_num
    multi_cmd_pub = [None] * multirotor_num

    for i in range(multirotor_num):
        multi_cmd_pose_flu_pub[i] = rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd_pose_flu', Pose, queue_size=1)
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd_vel_flu', Twist, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd', String, queue_size=3)
    leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
    leader_cmd_pose_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_pose_flu", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)

    # 控制逻辑
    if control_type == 'vel':
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

        # 初次悬停3分钟
        set_hover()
        vel_public(twist)
        rospy.sleep(HOVER_TIME)

        # 重复上升和悬停
        for i in range(ASCEND_REPETITIONS):
            change_upward(ASCEND_SPEED)
            vel_public(twist)
            ascend_time = ASCEND_DISTANCE / ASCEND_SPEED  # 计算上升时间
            rospy.sleep(ascend_time)

            set_hover()
            vel_public(twist)
            rospy.sleep(HOVER_TIME)

        # 降落和解除武装
        set_cmd('AUTO.LAND')
        vel_public(twist)
        rospy.sleep(30)

        set_cmd('DISARM')
        vel_public(twist)
        rospy.sleep(0.5)
