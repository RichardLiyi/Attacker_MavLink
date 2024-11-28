#-*- coding: UTF-8 -*-
import rospy
import tty, termios
import sys, select
import time
import PyKDL
import math
import numpy as np
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry

class DroneState(object):
    """无人机状态类，用于存储和更新无人机的状态信息"""
    def __init__(self):
        self.position = {'x': 0, 'y': 0, 'z': 0}
        self.velocity = {'x': 0, 'y': 0, 'z': 0}
        self.attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.angular_velocity = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.arm_state = False
        self.mavros_state = State()

    def update_position(self, msg):
        """更新位置信息"""
        self.position['x'] = msg.pose.position.x
        self.position['y'] = -msg.pose.position.y
        self.position['z'] = msg.pose.position.z

    def update_velocity(self, msg):
        """更新速度信息"""
        self.velocity['x'] = msg.twist.linear.x
        self.velocity['y'] = -msg.twist.linear.y
        self.velocity['z'] = msg.twist.linear.z

    def update_attitude(self, msg):
        """更新姿态信息"""
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        rot = PyKDL.Rotation.Quaternion(x, y, z, w)
        self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw'] = rot.GetRPY()
        
        self.angular_velocity['roll'] = msg.twist.twist.angular.x
        self.angular_velocity['pitch'] = msg.twist.twist.angular.y
        self.angular_velocity['yaw'] = msg.twist.twist.angular.z

class KeyboardController(object):
    """键盘控制类，处理键盘输入和控制逻辑"""
    def __init__(self):
        self.help_message = '''
        W/S: forward/backward
        A/D: left/right
        Q/E: up/down
        Z/C: rotate left/right
        T: unlock
        Y: lock
        R: return  
        L: land
        B: begin attitude control
        P: print control msg
        X: quit
        '''
        self.control_increments = {
            'position': 0.2,
            'yaw': 0.05
        }

    def get_key(self):
        """获取键盘输入"""
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key.lower()  # 转换为小写，实现大小写不敏感

class DroneController(object):
    """无人机控制类，整合状态监控和控制执行"""
    def __init__(self, uav_type, control_type):
        self.drone_state = DroneState()
        self.keyboard = KeyboardController()
        self.control_type = control_type
        self.mission_state = None
        
        # 控制参数
        self.control = {'x': 1, 'y': 0, 'z': 2, 'yaw': 0}
        self.takeoff = {'x': 0, 'y': 0, 'z': 2, 'yaw': 0}
        self.angle_max = 0.3

        # ROS通信设置
        self._setup_ros_communication(uav_type)
        self.att = AttitudeTarget()

    def _setup_ros_communication(self, uav_type):
        """设置ROS通信"""
        # 订阅器
        rospy.Subscriber("%s_0/mavros/local_position/pose" % uav_type, 
                        PoseStamped, self.drone_state.update_position)
        rospy.Subscriber("%s_0/mavros/local_position/velocity_body" % uav_type,
                        TwistStamped, self.drone_state.update_velocity)
        rospy.Subscriber("%s_0/mavros/local_position/odom" % uav_type,
                        Odometry, self.drone_state.update_attitude)
        rospy.Subscriber("%s_0/mavros/state" % uav_type,
                        State, self._update_mavros_state)

        # 发布器
        self.target_motion_pub = rospy.Publisher("%s_0/mavros/setpoint_raw/local" % uav_type,
                                               PositionTarget, queue_size=10)
        self.body_target_pub = rospy.Publisher("%s_0/mavros/setpoint_raw/attitude" % uav_type,
                                             AttitudeTarget, queue_size=2)

        # 服务
        self.flight_mode_service = rospy.ServiceProxy("%s_0/mavros/set_mode" % uav_type, SetMode)
        self.arm_service = rospy.ServiceProxy("%s_0/mavros/cmd/arming" % uav_type, CommandBool)

    def _update_mavros_state(self, msg):
        """更新mavros状态"""
        self.drone_state.mavros_state = msg.mode
        self.drone_state.arm_state = msg.armed

    def arm(self):
        """解锁"""
        if self.arm_service(True):
            print('解锁成功')
            return True
        print("解锁失败！")
        return False

    def disarm(self):
        """上锁"""
        if self.arm_service(False):
            print('上锁成功')
            return True
        print("上锁失败！")
        return False

    def process_keyboard_input(self, key):
        """处理键盘输入"""
        if not key:
            return True

        control_map = {
            'w': lambda: self._adjust_control('x', 0.2),
            's': lambda: self._adjust_control('x', -0.2),
            'a': lambda: self._adjust_control('y', -0.2),
            'd': lambda: self._adjust_control('y', 0.2),
            'q': lambda: self._adjust_control('z', 0.2),
            'e': lambda: self._adjust_control('z', -0.2),
            'z': lambda: self._adjust_control('yaw', 0.05),
            'c': lambda: self._adjust_control('yaw', -0.05),
            't': self._handle_takeoff,
            'y': self.disarm,
            'r': self._handle_return,
            'l': self._handle_land,
            'b': self._handle_begin_control,
            'p': self._print_status,
            'x': lambda: False
        }

        if key in control_map:
            result = control_map[key]()
            return False if key == 'x' else True
        return True

    def _adjust_control(self, axis, value):
        """调整控制值"""
        self.control[axis] += value
        self._print_status()

    def _handle_takeoff(self):
        """处理起飞"""
        self.arm()
        self.mission_state = 'takeoff'
        print('开始起飞')
        self._print_status()

    def _handle_return(self):
        """处理返航"""
        self.mission_state = 'return'
        print('返回起飞点')

    def _handle_land(self):
        """处理降落"""
        self.flight_mode_service(custom_mode='AUTO.LAND')
        self.mission_state = 'land'
        print('开始降落')

    def _handle_begin_control(self):
        """开始姿态控制"""
        self.mission_state = 'control'
        print('开始姿态控制')
        self._print_status()

    def _print_status(self):
        """打印状态信息"""
        print(self.keyboard.help_message)
        print("控制位置 - X: %.2f Y: %.2f Z: %.2f YAW: %.2f" % 
              (self.control['x'], self.control['y'], 
               self.control['z'], self.control['yaw']))

    def update_control(self):
        """更新控制指令"""
        if self.mission_state in ['return', 'takeoff']:
            self._send_position_target()
        elif self.mission_state == 'control':
            self._send_attitude_target()

    def _send_position_target(self):
        """发送位置控制指令"""
        target = PositionTarget()
        target.position.x = self.takeoff['x']
        target.position.y = self.takeoff['y']
        target.position.z = self.takeoff['z']
        target.yaw = self.takeoff['yaw']
        target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY +
                           PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX +
                           PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                           PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE)
        self.target_motion_pub.publish(target)

    def _send_attitude_target(self):
        """发送姿态控制指令"""
        if self.control_type == "angular_speed":
            self._send_angular_speed_control()
        elif self.control_type == "angle":
            self._send_angle_control()

    def _send_angular_speed_control(self):
        """发送角速度控制指令"""
        # 级联PID控制
        roll_r = -0.54236707 * (self.drone_state.position['y'] - self.control['y']) - \
                 0.42717549 * (self.drone_state.velocity['y'] - 0)
        pitch_r = -0.54236707 * (self.drone_state.position['x'] - self.control['x']) - \
                  0.42717549 * (self.drone_state.velocity['x'] - 0)

        # 限制角度
        roll_r = np.clip(roll_r, -self.angle_max, self.angle_max)
        pitch_r = np.clip(pitch_r, -self.angle_max, self.angle_max)

        # 计算角速度
        roll_rate = -5 * (self.drone_state.attitude['roll'] - roll_r) - \
                    0.1 * self.drone_state.angular_velocity['roll']
        pitch_rate = -5 * (self.drone_state.attitude['pitch'] - pitch_r) - \
                     0.1 * self.drone_state.angular_velocity['pitch']
        yaw_rate = -1 * (self.drone_state.attitude['yaw'] - self.control['yaw']) - \
                   0.1 * self.drone_state.angular_velocity['yaw']

        # 发送控制指令
        self.att.body_rate.x = roll_rate
        self.att.body_rate.y = pitch_rate
        self.att.body_rate.z = yaw_rate
        self.att.thrust = (-0.5 * (self.drone_state.position['z'] - self.control['z']) -
                          0.31774509 * (self.drone_state.velocity['z'] - 0) +
                          0.55 / math.cos(self.drone_state.attitude['pitch']) /
                          math.cos(self.drone_state.attitude['roll']))
        self.att.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        self.body_target_pub.publish(self.att)

    def start(self):
        """启动控制循环"""
        rospy.init_node("attitude_control_node")
        print('启动！')
        self._print_status()

        while not rospy.is_shutdown():
            key = self.keyboard.get_key()
            if not self.process_keyboard_input(key):
                break
            self.update_control()

def main():
    """主函数"""
    if len(sys.argv) != 3:
        print("Usage: python attitude_control_demo.py <uav_type> <control_type>")
        sys.exit(1)

    controller = DroneController(sys.argv[1], sys.argv[2])
    controller.start()

if __name__ == '__main__':
    main()
