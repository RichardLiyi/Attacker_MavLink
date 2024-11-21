# -*- coding: utf-8 -*-

# 用于订阅matlab控制消息，向multirotor_communication节点转发
import rospy
from geometry_msgs.msg import Twist,Pose,Point
import sys, select, os
import tty, termios
from std_msgs.msg import String
cmd= String()
twist = Twist()
pose = Pose()

MAX_LINEAR = 20
MAX_ANG_VEL = 3

ctrl_leader = False
pose_control_flag=False
forward   = 0.0
leftward   = 0.0
upward   = 0.0
angular  = 0.0

target_x = 1
target_y = 4
target_z = 1.2
cmd = ''

# /iris_0/mavros/state 这个话题可能会发布关于无人机的状态信息，包括无人机是否处于联接状态、无人机是否处于遥控或自主飞行模式等信息
# /iris_0/mavros/global_position/global 和 /iris_0/mavros/global_position/local: 这两个话题可能包含无人机的全球和局部位置信息
# /iris_0/mavros/imu/data: 这个话题可能会发布无人机的惯性测量单元（IMU）数据，包括角速度和线性加速度。
# /iris_0/mavros/vfr_hud: 这个话题可能包含飞行数据，例如当前高度、速度、指向等。
def boundary_check():
    global forward,leftward,upward,angular
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
    # print(f'{command}\n')
    print('%s\n' % command)


def set_hover():
    global cmd,forward,leftward,upward,angular
    forward   = 0.0
    leftward   = 0.0
    upward   = 0.0
    angular  = 0.0
    cmd = 'HOVER'
    print('HOVER\n')

# 发布线速度指令（速度控制模式）
def vel_public(twist):
    twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
    twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular
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
            # print("pose.position.x: ",  pose.position.x)
            for i in range(multirotor_num):
                multi_cmd_pose_flu_pub[i].publish(pose)
                multi_cmd_pub[i].publish(cmd)
            rospy.sleep(0.1)
        else:
            print("Not ready to pose control mode. Please wait.")


if __name__=="__main__":

    # settings = termios.tcgetattr(sys.stdin)

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
        # subscribe to matlab
        multi_matlab_pose_flu_sub = rospy.Subscriber("/matlab/iris_waypoint", Point, matlab_pose_sub_callback, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=3)
    leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
    leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
    leader_cmd_pose_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_pose_flu", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)


    # 速度控制模式      
    if control_type == 'vel':
        # velocity control system
        rospy.sleep(0.2)
        set_cmd('OFFBOARD')
        vel_public(twist)
        rospy.sleep(0.5)

        change_upward(2)
        vel_public(twist)
        rospy.sleep(0.5)

        set_cmd('ARM')
        vel_public(twist)
        rospy.sleep(8)

        set_hover()
        vel_public(twist)
        rospy.sleep(90)

        # set_hover()
        # vel_public(twist)
        # rospy.sleep(1)

        set_cmd('AUTO.LAND')
        vel_public(twist)
        rospy.sleep(30)

        set_cmd('DISARM')
        vel_public(twist)
        rospy.sleep(0.5)

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
        vel_public(twist)       # 发布无人机状态
        rospy.sleep(1)

        pose_control_flag = True    # 启动坐标控制
        rospy.sleep(3600)        # hover时间
        pose_control_flag = False

        set_hover()
        vel_public(twist)
        rospy.sleep(1)

        set_cmd('AUTO.LAND')
        vel_public(twist)
        rospy.sleep(10)

        set_cmd('DISARM')
        vel_public(twist)
        rospy.sleep(0.5)

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

