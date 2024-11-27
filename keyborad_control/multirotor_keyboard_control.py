#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import tty
import termios
import os

class MultirotorKeyboardController:
    def __init__(self, multirotor_type, multirotor_num, control_type):
        """Initialize the keyboard controller.
        
        Args:
            multirotor_type (str): Type of multirotor
            multirotor_num (int): Number of multirotors
            control_type (str): Control type ('vel' or 'accel')
        """
        # Constants
        self.MAX_LINEAR = 20
        self.MAX_ANG_VEL = 3
        self.LINEAR_STEP_SIZE = 0.01
        self.ANG_VEL_STEP_SIZE = 0.01
        self.TARGET_HOVER_HEIGHT = 5.0  # Target hover height in meters
        
        # Control states
        self.cmd_vel_mask = False
        self.ctrl_leader = False
        self.is_hovering = False
        
        # Movement states
        self.forward = 0.0
        self.leftward = 0.0
        self.upward = 0.0
        self.angular = 0.0
        
        # Target position
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        
        # Store parameters
        self.multirotor_type = multirotor_type
        self.multirotor_num = multirotor_num
        self.control_type = control_type
        
        # Initialize ROS node
        rospy.init_node(f'{multirotor_type}_multirotor_keyboard_control')
        
        # Setup publishers
        self._setup_publishers()
        
        # Setup formation configs
        self._setup_formation_configs()
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def _setup_publishers(self):
        """Setup ROS publishers based on control type."""
        if self.control_type == 'vel':
            self.multi_cmd_vel_flu_pub = [
                rospy.Publisher(f'/xtdrone/{self.multirotor_type}_{i}/cmd_vel_flu', Twist, queue_size=1)
                for i in range(self.multirotor_num)
            ]
            self.multi_cmd_pub = [
                rospy.Publisher(f'/xtdrone/{self.multirotor_type}_{i}/cmd', String, queue_size=3)
                for i in range(self.multirotor_num)
            ]
            self.leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
            self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)
        else:
            self.multi_cmd_accel_flu_pub = [
                rospy.Publisher(f'/xtdrone/{self.multirotor_type}_{i}/cmd_accel_flu', Twist, queue_size=1)
                for i in range(self.multirotor_num)
            ]
            self.multi_cmd_pub = [
                rospy.Publisher(f'/xtdrone/{self.multirotor_type}_{i}/cmd', String, queue_size=1)
                for i in range(self.multirotor_num)
            ]
            self.leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
            self.leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=3)

    def _setup_formation_configs(self):
        """Setup formation configurations based on number of multirotors."""
        if self.multirotor_num == 18:
            self.formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
        elif self.multirotor_num == 9:
            self.formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
        elif self.multirotor_num == 6:
            self.formation_configs = ['waiting', 'T', 'diamond', 'triangle']
        elif self.multirotor_num == 1:
            self.formation_configs = ['stop controlling']

    def get_key(self):
        """Get a keyboard press."""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except Exception as e:
            print(f"Error reading keyboard input: {e}")
            return ''

    def print_instructions(self):
        """Print control instructions based on current mode."""
        if self.ctrl_leader:
            self._print_leader_instructions()
        else:
            self._print_all_instructions()

    def _print_leader_instructions(self):
        """Print instructions for leader control mode."""
        print("""
Control Your XTDrone!
To the leader  (press g to control all drones)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward 
a/d : increase/decrease leftward 
i/, : increase/decrease upward 
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
g   : control all drones
CTRL-C to quit
""")

    def _print_all_instructions(self):
        """Print instructions for all drones control mode."""
        print("""
Control Your XTDrone!
To all drones  (press g to control the leader)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward  
a/d : increase/decrease leftward 
i/, : increase/decrease upward
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
0   : mask the keyboard control (for single UAV)
1   : one-key takeoff and hover at (0,0,5)
2   : fly to specified position (x,y,z)
3-9 : extendable mission
g   : control the leader
CTRL-C to quit
""")

    def get_user_target_position(self):
        """Get target position from user input."""
        try:
            print("\nPlease enter the target position coordinates:")
            print("Enter X coordinate (meters): ", end='', flush=True)
            x = float(input())
            print("Enter Y coordinate (meters): ", end='', flush=True)
            y = float(input())
            print("Enter Z coordinate (meters): ", end='', flush=True)
            z = float(input())
            return x, y, z
        except ValueError:
            print("Invalid input! Please enter numeric values.")
            return None
        except Exception as e:
            print(f"Error reading input: {e}")
            return None

    def fly_to_position(self, x, y, z):
        """Fly to specified position."""
        print(f"\nFlying to position: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Store target position
        self.target_x = x
        self.target_y = y
        self.target_z = z
        
        # Create and publish position command
        twist = Twist()
        twist.linear.x = x  # Forward position
        twist.linear.y = y  # Leftward position
        twist.linear.z = z  # Upward position
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # First make sure we're in OFFBOARD mode
        cmd = String()
        cmd.data = 'OFFBOARD'
        if self.ctrl_leader:
            self.leader_cmd_pub.publish(cmd)
        else:
            for i in range(self.multirotor_num):
                self.multi_cmd_pub[i].publish(cmd)
        
        rospy.sleep(0.5)  # Wait for mode switch
        
        # Publish position command
        if self.ctrl_leader:
            if self.control_type == 'vel':
                self.leader_cmd_vel_flu_pub.publish(twist)
            else:
                self.leader_cmd_accel_flu_pub.publish(twist)
        else:
            for i in range(self.multirotor_num):
                if self.control_type == 'vel':
                    self.multi_cmd_vel_flu_pub[i].publish(twist)
                else:
                    self.multi_cmd_accel_flu_pub[i].publish(twist)
        
        print(f"Command sent. Moving to position ({x:.2f}, {y:.2f}, {z:.2f})")

    def update_movement(self, key):
        """Update movement based on key press."""
        if key == 'w':
            self.forward = min(self.forward + self.LINEAR_STEP_SIZE, self.MAX_LINEAR)
        elif key == 'x':
            self.forward = max(self.forward - self.LINEAR_STEP_SIZE, -self.MAX_LINEAR)
        elif key == 'a':
            self.leftward = min(self.leftward + self.LINEAR_STEP_SIZE, self.MAX_LINEAR)
        elif key == 'd':
            self.leftward = max(self.leftward - self.LINEAR_STEP_SIZE, -self.MAX_LINEAR)
        elif key == 'i':
            self.upward = min(self.upward + self.LINEAR_STEP_SIZE, self.MAX_LINEAR)
        elif key == ',':
            self.upward = max(self.upward - self.LINEAR_STEP_SIZE, -self.MAX_LINEAR)
        elif key == 'j':
            self.angular = min(self.angular + self.ANG_VEL_STEP_SIZE, self.MAX_ANG_VEL)
        elif key == 'l':
            self.angular = max(self.angular - self.ANG_VEL_STEP_SIZE, -self.MAX_ANG_VEL)

    def publish_movement(self):
        """Publish movement commands to ROS topics."""
        twist = Twist()
        twist.linear.x = self.forward
        twist.linear.y = self.leftward
        twist.linear.z = self.upward
        twist.angular.z = self.angular

        if not self.cmd_vel_mask:
            if self.ctrl_leader:
                if self.control_type == 'vel':
                    self.leader_cmd_vel_flu_pub.publish(twist)
                else:
                    self.leader_cmd_accel_flu_pub.publish(twist)
            else:
                for i in range(self.multirotor_num):
                    if self.control_type == 'vel':
                        self.multi_cmd_vel_flu_pub[i].publish(twist)
                    else:
                        self.multi_cmd_accel_flu_pub[i].publish(twist)

    def one_key_takeoff_hover(self):
        """Execute one-key takeoff and hover sequence."""
        # First arm the drone
        cmd = String()
        cmd.data = 'ARM'
        if self.ctrl_leader:
            self.leader_cmd_pub.publish(cmd)
        else:
            for i in range(self.multirotor_num):
                self.multi_cmd_pub[i].publish(cmd)
        
        rospy.sleep(1)  # Wait for arming
        
        # Switch to OFFBOARD mode
        cmd.data = 'OFFBOARD'
        if self.ctrl_leader:
            self.leader_cmd_pub.publish(cmd)
        else:
            for i in range(self.multirotor_num):
                self.multi_cmd_pub[i].publish(cmd)
        
        rospy.sleep(1)  # Wait for mode switch
        
        # Takeoff sequence
        cmd.data = 'AUTO.TAKEOFF'
        if self.ctrl_leader:
            self.leader_cmd_pub.publish(cmd)
        else:
            for i in range(self.multirotor_num):
                self.multi_cmd_pub[i].publish(cmd)
        
        rospy.sleep(5)  # Wait for takeoff
        
        # Set hover position
        twist = Twist()
        twist.linear.x = 0.0  # Forward position
        twist.linear.y = 0.0  # Leftward position
        twist.linear.z = self.TARGET_HOVER_HEIGHT  # Upward position
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Publish hover position
        if self.ctrl_leader:
            if self.control_type == 'vel':
                self.leader_cmd_vel_flu_pub.publish(twist)
            else:
                self.leader_cmd_accel_flu_pub.publish(twist)
        else:
            for i in range(self.multirotor_num):
                if self.control_type == 'vel':
                    self.multi_cmd_vel_flu_pub[i].publish(twist)
                else:
                    self.multi_cmd_accel_flu_pub[i].publish(twist)
        
        self.is_hovering = True
        print(f"Executing one-key takeoff and hover at height {self.TARGET_HOVER_HEIGHT}m")

    def run(self):
        """Main control loop."""
        self.print_instructions()
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if key == '\x03':  # CTRL-C
                    break
                
                # Handle one-key takeoff
                if key == '1':
                    self.one_key_takeoff_hover()
                    continue
                
                # Handle fly to position
                elif key == '2':
                    target = self.get_user_target_position()
                    if target:
                        x, y, z = target
                        self.fly_to_position(x, y, z)
                    continue
                
                self.update_movement(key)
                self.publish_movement()
                
                if key:
                    self.print_instructions()
                    print(f"currently:\t forward {'vel' if self.control_type == 'vel' else 'accel'} {self.forward:.2f}\t "
                          f"leftward {'vel' if self.control_type == 'vel' else 'accel'} {self.leftward:.2f}\t "
                          f"upward {'vel' if self.control_type == 'vel' else 'accel'} {self.upward:.2f}\t "
                          f"angular {self.angular:.2f}")
                    
        except Exception as e:
            print(f"Error in main control loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup when shutting down."""
        # Stop all movement
        self.forward = self.leftward = self.upward = self.angular = 0.0
        self.publish_movement()
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    """Main function."""
    if len(sys.argv) != 4:
        print("Usage: python3 multirotor_keyboard_control.py <multirotor_type> <multirotor_num> <control_type>")
        sys.exit(1)
        
    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]
    
    controller = MultirotorKeyboardController(multirotor_type, multirotor_num, control_type)
    controller.run()

if __name__ == "__main__":
    main()
