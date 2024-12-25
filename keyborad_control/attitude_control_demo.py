#-*- coding: UTF-8<|start_header|><|start_header|>assistant<|end_header|>

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
from Queue import Queue
import threading

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
        ====== 无人机键盘控制系统 ======
        
        基本控制按键:
        ---------------
        移动控制:
          W/S: 前进/后退
          A/D: 左右移动
          Q/E: 上升/下降
          Z/C: 左右旋转
          F: 飞行到指定位置
          O: 执行偏置实验
        
        功能按键:
        ---------------
        T: 解锁并起飞（起飞高度2米）
        Y: 上锁
        B: 开始姿态控制（起飞后必须按此键才能移动）
        R: 返航
        L: 降落
        P: 显示控制信息
        X: 退出程序
        
        使用步骤:
        ---------------
        1. 按T键解锁并起飞
        2. 等待上升到2米高度
        3. 按B键开始姿态控制
        4. 使用WSADQEZC进行移动控制
           或按F键输入目标位置
        5. 需要降落时按L键
        
        注意事项:
        ---------------
        - 第一次起飞前请确保周围空间充足
        - 请先等待无人机稳定后再进行移动控制
        - 遇到紧急情况可按Y键紧急上锁
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
        self.control = {'x': 0, 'y': 0, 'z': 2, 'yaw': 0}  # 修改初始控制位置
        self.takeoff = {'x': 0, 'y': 0, 'z': 2, 'yaw': 0}  # 设置起飞高度为2米
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
        if key == 'w':
            self.control['x'] += 0.1
        elif key == 's':
            self.control['x'] -= 0.1
        elif key == 'a':
            self.control['y'] += 0.1
        elif key == 'd':
            self.control['y'] -= 0.1
        elif key == 'q':
            self.control['z'] += 0.1
        elif key == 'e':
            self.control['z'] -= 0.1
        elif key == 'z':
            self.control['yaw'] += 0.1
        elif key == 'c':
            self.control['yaw'] -= 0.1
        elif key == 't':
            self._handle_takeoff()
        elif key == 'l':
            self._handle_land()
        elif key == 'o':
            print("\n请选择实验类型：")
            print("1. 水平X位置偏置实验")
            print("2. 水平Y位置偏置实验")
            print("3. 高度Z位置偏置实验")
            print("4. 偏航角Yaw偏置实验")
            try:
                choice = int(raw_input("请输入选项（1-4）: "))
                if choice == 1:
                    self._handle_x_offset_experiment()
                elif choice == 2:
                    self._handle_y_offset_experiment()
                elif choice == 3:
                    self._handle_z_offset_experiment()
                elif choice == 4:
                    self._handle_yaw_offset_experiment()
                else:
                    print("无效的选项！")
            except ValueError:
                print("请输入有效的数字！")
        elif key == 'x':
            print("退出程序")
            return False
        return True

    def _handle_takeoff(self):
        """处理起飞"""
        self.arm()
        self.flight_mode_service(custom_mode='OFFBOARD')
        self.mission_state = 'control'
        
        # 设置起飞目标位置并持续发送控制指令
        takeoff_target = {'x': 0, 'y': 0, 'z': 3.87, 'yaw': 0}
        print('开始X轴位置偏置实验')
        print("起飞位置 - X: %.2f Y: %.2f Z: %.2f" % 
                (takeoff_target['x'], takeoff_target['y'], takeoff_target['z']))
        
        # 持续发送起飞位置5秒
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 5:
            self.control.update(takeoff_target)
            self._send_position_target()
            rospy.sleep(0.1)

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
        print("控制位置 - X: %.2f Y: %.2f Z: %.2f YAW: %.2f" % 
              (self.control['x'], self.control['y'], 
               self.control['z'], self.control['yaw']))

    def _handle_fly_to_position(self):
        """处理飞行到指定位置"""
        # 确保当前处于OFFBOARD模式和控制状态
        if self.mission_state != 'control':
            print("\n请先按B键开始姿态控制")
            return

        # 保存当前位置
        current_x = self.control['x']
        current_y = self.control['y']
        current_z = self.control['z']
        current_yaw = self.control['yaw']
        
        try:
            print("\n当前位置 - X: %.2f Y: %.2f Z: %.2f" % (current_x, current_y, current_z))
            print("请输入目标位置（输入非数字返回当前位置）：")
            
            x = input("目标X坐标: ")
            y = input("目标Y坐标: ")
            z = input("目标Z坐标: ")
            
            # 转换输入为浮点数
            x = float(x)
            y = float(y)
            z = float(z)
            
            # 重新切换到OFFBOARD模式，确保控制权
            self.flight_mode_service(custom_mode='OFFBOARD')
            
            # 发布一个明确的位置目标，防止着陆检测
            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = (PositionTarget.IGNORE_VX + 
                              PositionTarget.IGNORE_VY +
                              PositionTarget.IGNORE_VZ + 
                              PositionTarget.IGNORE_AFX +
                              PositionTarget.IGNORE_AFY + 
                              PositionTarget.IGNORE_AFZ +
                              PositionTarget.IGNORE_YAW_RATE)

            # 设置目标位置
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.yaw = current_yaw

            # 设置header
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = "map"

            # 连续发布几次位置目标
            for _ in range(10):
                self.target_motion_pub.publish(target)
                rospy.sleep(0.1)
            
            # 更新控制目标
            self.control['x'] = x
            self.control['y'] = y
            self.control['z'] = z
            
            print("\n正在飞向目标位置 - X: %.2f Y: %.2f Z: %.2f" % (x, y, z))
            
        except (ValueError, KeyboardInterrupt):
            print("\n输入取消，保持当前位置")
            # 恢复当前位置
            self.control['x'] = current_x
            self.control['y'] = current_y
            self.control['z'] = current_z
            self.control['yaw'] = current_yaw
            return

    def _handle_x_offset_experiment(self):
        """处理X轴位置偏置实验"""
        try:
            # 解锁并切换到OFFBOARD模式
            self.arm()
            self.flight_mode_service(custom_mode='OFFBOARD')
            self.mission_state = 'control'
            
            # 设置起飞目标位置并持续发送控制指令
            takeoff_target = {'x': 0, 'y': 0, 'z': 5, 'yaw': 0}
            print('开始X轴位置偏置实验')
            print("起飞位置 - X: %.2f Y: %.2f Z: %.2f" % 
                  (takeoff_target['x'], takeoff_target['y'], takeoff_target['z']))
            
            # 持续发送起飞位置5秒
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < 5:
                self.control.update(takeoff_target)
                self._send_position_target()
                rospy.sleep(0.1)
            
            # 获取实验参数
            params = self._get_experiment_params()
            if params is None:
                self._safe_land()
                return
                
            D, T, experiment_time = params
            print("\n开始X轴位置偏置实验")
            
            # 计算循环次数
            cycles = int(experiment_time / T)
            direction = 1  # 1 表示正向，-1 表示反向
            
            for _ in range(cycles):
                # 设置目标位置
                target = {'x': D * direction, 'y': 0, 'z': 5, 'yaw': 0}
                self.control.update(target)
                
                # 发送位置目标并打印当前位置
                self._send_position_target()
                current_pos = self.drone_state.position
                print("目标位置 - X: %.2f, 当前位置 - X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f" % 
                      (D * direction, current_pos['x'], current_pos['y'], 
                       current_pos['z'], self.drone_state.attitude['yaw']))
                
                # 等待完整周期
                rospy.sleep(T)
                direction *= -1  # 切换方向
            
            # 实验完成，降落
            self._safe_land()
            print("\n实验完成")
        
        except (ValueError, KeyboardInterrupt):
            print("\n实验被中断")
            self._safe_land()

    def _handle_y_offset_experiment(self):
        """处理Y轴位置偏置实验"""
        try:
            # 解锁并切换到OFFBOARD模式
            self.arm()
            self.flight_mode_service(custom_mode='OFFBOARD')
            self.mission_state = 'control'
            
            # 设置起飞目标位置并持续发送控制指令
            takeoff_target = {'x': 0, 'y': 0, 'z': 5, 'yaw': 0}
            print('开始Y轴位置偏置实验')
            print("起飞位置 - X: %.2f Y: %.2f Z: %.2f" % 
                  (takeoff_target['x'], takeoff_target['y'], takeoff_target['z']))
            
            # 持续发送起飞位置5秒
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < 5:
                self.control.update(takeoff_target)
                self._send_position_target()
                rospy.sleep(0.1)
            
            # 获取实验参数
            params = self._get_experiment_params()
            if params is None:
                self._safe_land()
                return
                
            D, T, experiment_time = params
            print("\n开始Y轴位置偏置实验")
            
            # 计算循环次数
            cycles = int(experiment_time / T)
            direction = 1  # 1 表示正向，-1 表示反向
            
            for _ in range(cycles):
                # 设置目标位置
                target = {'x': 0, 'y': D * direction, 'z': 5, 'yaw': 0}
                self.control.update(target)
                
                # 发送位置目标并打印当前位置
                self._send_position_target()
                current_pos = self.drone_state.position
                print("目标位置 - Y: %.2f, 当前位置 - X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f" % 
                      (D * direction, current_pos['x'], current_pos['y'], 
                       current_pos['z'], self.drone_state.attitude['yaw']))
                
                # 等待完整周期
                rospy.sleep(T)
                direction *= -1  # 切换方向
            
            # 实验完成，降落
            self._safe_land()
            print("\n实验完成")
        
        except (ValueError, KeyboardInterrupt):
            print("\n实验被中断")
            self._safe_land()

    def _handle_z_offset_experiment(self):
        """处理Z轴位置偏置实验"""
        try:
            # 解锁并切换到OFFBOARD模式
            self.arm()
            self.flight_mode_service(custom_mode='OFFBOARD')
            self.mission_state = 'control'
            
            # 设置起飞目标位置并持续发送控制指令
            takeoff_target = {'x': 0, 'y': 0, 'z': 3.87, 'yaw': 0}
            print('开始Z轴位置偏置实验')
            print("起飞位置 - X: %.2f Y: %.2f Z: %.2f" % 
                  (takeoff_target['x'], takeoff_target['y'], takeoff_target['z']))
            
            # 持续发送起飞位置5秒
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < 5:
                self.control.update(takeoff_target)
                self._send_position_target()
                rospy.sleep(0.1)
            
            # 获取实验参数
            params = self._get_experiment_params()
            if params is None:
                self._safe_land()
                return
                
            D, T, experiment_time = params
            print("\n开始Z轴位置偏置实验")
            
            # 计算循环次数
            cycles = int(experiment_time / T)
            direction = 1  # 1 表示正向，-1 表示反向
            base_height = 3.87  # 基础高度
            
            for _ in range(cycles):
                # 设置目标位置
                target_z = base_height + D * direction
                target = {'x': 0, 'y': 0, 'z': target_z, 'yaw': 0}
                self.control.update(target)
                
                # 发送位置目标并打印当前位置
                self._send_position_target()
                current_pos = self.drone_state.position
                print("目标高度 - Z: %.2f, 当前位置 - X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f" % 
                      (target_z, current_pos['x'], current_pos['y'], 
                       current_pos['z'], self.drone_state.attitude['yaw']))
                
                # 等待完整周期
                rospy.sleep(T)
                direction *= -1  # 切换方向
            
            # 实验完成，降落
            self._safe_land()
            print("\n实验完成")
        
        except (ValueError, KeyboardInterrupt):
            print("\n实验被中断")
            self._safe_land()

    def _handle_yaw_offset_experiment(self):
        """处理偏航角偏置实验"""
        try:
            # 解锁并切换到OFFBOARD模式
            self.arm()
            self.flight_mode_service(custom_mode='OFFBOARD')
            self.mission_state = 'control'
            
            # 设置起飞目标位置并持续发送控制指令
            takeoff_target = {'x': 0, 'y': 0, 'z': 5, 'yaw': 0}
            print('开始偏航角偏置实验')
            print("起飞位置 - X: %.2f Y: %.2f Z: %.2f" % 
                  (takeoff_target['x'], takeoff_target['y'], takeoff_target['z']))
            
            # 持续发送起飞位置5秒
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < 5:
                self.control.update(takeoff_target)
                self._send_position_target()
                rospy.sleep(0.1)
            
            # 获取实验参数
            params = self._get_experiment_params()
            if params is None:
                self._safe_land()
                return
                
            D, T, experiment_time = params
            print("\n开始偏航角偏置实验")
            
            # 计算循环次数
            cycles = int(experiment_time / T)
            direction = 1  # 1 表示正向，-1 表示反向
            
            for _ in range(cycles):
                # 设置目标偏航角
                target = {'x': 0, 'y': 0, 'z': 5, 'yaw': D * direction}
                self.control.update(target)
                
                # 发送位置目标并打印当前位置
                self._send_position_target()
                current_pos = self.drone_state.position
                print("目标偏航角 - Yaw: %.2f, 当前位置 - X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f" % 
                      (D * direction, current_pos['x'], current_pos['y'], 
                       current_pos['z'], self.drone_state.attitude['yaw']))
                
                # 等待完整周期
                rospy.sleep(T)
                direction *= -1  # 切换方向
            
            # 实验完成，降落
            self._safe_land()
            print("\n实验完成")
        
        except (ValueError, KeyboardInterrupt):
            print("\n实验被中断")
            self._safe_land()

    def _get_experiment_params(self):
        """获取实验参数"""
        try:
            D = float(raw_input("偏置幅度D (米): "))
            T = float(raw_input("偏置周期T (秒): "))
            experiment_time = float(raw_input("实验总持续时间 (秒): "))
            
            if D <= 0 or T <= 0 or experiment_time <= 0:
                print("参数必须为正数！")
                return None
            return D, T, experiment_time
        except ValueError:
            print("输入必须为数字！")
            return None

    def _safe_land(self):
        """安全降落程序"""
        print("执行安全降落...")
        self.flight_mode_service(custom_mode='AUTO.LAND')
        rospy.sleep(5)  # 等待降落完成
        self.disarm()
        print("降落完成，已上锁")

    def _print_status(self):
        """打印状态信息"""
        print(self.keyboard.help_message)

    def update_control(self):
        """更新控制指令"""
        if self.mission_state in ['takeoff', 'return', 'control']:
            # 在起飞、返航和控制状态下，发送位置目标
            self._send_position_target()
            
            # 防止意外着陆
            if self.mission_state == 'control':
                # 发送姿态目标
                self._send_attitude_target()
                
                # 额外发布一个位置目标，确保不会被误判为着陆
                extra_target = PositionTarget()
                extra_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                extra_target.type_mask = (PositionTarget.IGNORE_VX + 
                                          PositionTarget.IGNORE_VY +
                                          PositionTarget.IGNORE_VZ + 
                                          PositionTarget.IGNORE_AFX +
                                          PositionTarget.IGNORE_AFY + 
                                          PositionTarget.IGNORE_AFZ +
                                          PositionTarget.IGNORE_YAW_RATE)

                # 设置目标位置为当前控制位置
                extra_target.position.x = self.control['x']
                extra_target.position.y = self.control['y']
                extra_target.position.z = self.control['z']
                extra_target.yaw = self.control['yaw']

                # 设置header
                extra_target.header.stamp = rospy.Time.now()
                extra_target.header.frame_id = "map"

                self.target_motion_pub.publish(extra_target)
        
        elif self.mission_state == 'land':
            # 降落时切换到AUTO.LAND模式
            self.flight_mode_service(custom_mode='AUTO.LAND')

    def _send_position_target(self):
        """发送位置控制指令"""
        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX + 
                          PositionTarget.IGNORE_VY +
                          PositionTarget.IGNORE_VZ + 
                          PositionTarget.IGNORE_AFX +
                          PositionTarget.IGNORE_AFY + 
                          PositionTarget.IGNORE_AFZ +
                          PositionTarget.IGNORE_YAW_RATE)

        # 设置目标位置为当前控制位置
        target.position.x = self.control['x']
        target.position.y = self.control['y']
        target.position.z = self.control['z']
        target.yaw = self.control['yaw']

        # 设置header
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"

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

    def _send_angle_control(self):
        """发送角度控制指令"""
        # PD控制
        roll_r = -0.54236707 * (self.drone_state.position['y'] - self.control['y']) - \
                 0.42717549 * (self.drone_state.velocity['y'] - 0)
        pitch_r = -0.54236707 * (self.drone_state.position['x'] - self.control['x']) - \
                  0.42717549 * (self.drone_state.velocity['x'] - 0)
        yaw_r = self.control['yaw']
        
        # 限制角度
        roll_r = np.clip(roll_r, -self.angle_max, self.angle_max)
        pitch_r = np.clip(pitch_r, -self.angle_max, self.angle_max)

        # 计算推力
        thrust = (-0.5 * (self.drone_state.position['z'] - self.control['z']) -
                 0.31774509 * (self.drone_state.velocity['z'] - 0) +
                 0.55 / math.cos(self.drone_state.attitude['pitch']) /
                 math.cos(self.drone_state.attitude['roll']))

        # 将欧拉角转换为四元数
        q = self._euler_to_quaternion(roll_r, pitch_r, yaw_r)
        
        # 发送控制指令
        self.att.orientation.w = q[0]
        self.att.orientation.x = q[1]
        self.att.orientation.y = q[2]
        self.att.orientation.z = q[3]
        self.att.thrust = thrust
        self.att.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE +
                             AttitudeTarget.IGNORE_PITCH_RATE +
                             AttitudeTarget.IGNORE_YAW_RATE)
        self.body_target_pub.publish(self.att)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    def start(self):
        """启动控制循环"""
        rospy.init_node("attitude_control_node")
        print('''
============================================
          无人机键盘控制系统启动
============================================
作者: RichardLiyi
版本: 1.0
适用于: Python 2.7 + ROS + MAVROS

【重要提示】
1. 确保ROS环境已经正确加载
2. 确保MAVROS已正常运行
3. 确保与无人机的连接正常

【使用方法】
1. 按T键解锁并起飞
2. 等待上升到预定高度（2米）
3. 按B键开始姿态控制
4. 使用键盘控制无人机移动

按P键可随时查看详细控制说明
============================================
        ''')
        self._print_status()

        rate = rospy.Rate(20)  # 设置20Hz的控制频率
        
        # 创建一个线程来处理键盘输入
        def input_thread():
            while True:
                key = self.keyboard.get_key()
                if not self.process_keyboard_input(key):
                    break
        
        thread = threading.Thread(target=input_thread)
        thread.daemon = True  # 设置为守护线程，这样主程序退出时线程会自动结束
        thread.start()
        
        while not rospy.is_shutdown():
            self.update_control()
            rate.sleep()  # 控制循环频率

def main():
    """主函数"""
    if len(sys.argv) != 3:
        print("Usage: python attitude_control_demo.py <uav_type> <control_type>")
        sys.exit(1)

    controller = DroneController(sys.argv[1], sys.argv[2])
    controller.start()

if __name__ == '__main__':
    main()
