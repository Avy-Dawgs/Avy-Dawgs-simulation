from typing import Tuple
from pymavlink import mavutil

class DroneMAVLink:
    '''
    This class represents the MAVLink connection to the drone, providing abstraction over all 
    MAVLink commands to be used. 
    '''

    def __init__(self, connection_str: str) -> None:
        '''
        Initialize instance of drone. 
        '''
        self.connection_str = connection_str
        self.mavconn: mavutil.mavfile
        self.yaw = 0

    def connect(self):
        '''
        Connect to drone.
        '''
        self.mavconn = mavutil.mavlink_connection(self.connection_str)

    def set_yaw(self, yaw):
        '''
        Lock yaw to specified orientation. (in degrees)
        '''
        self.yaw = yaw

    def read_local_position(self) -> Tuple[float, float, float]:
        '''
        Read the drone's local position.
        '''
        pass
        while True:
            pos = self.mavconn.recv_match(type="LOCAL_POSITION_NED", blocking=True)
            if pos:
                return pos.x, pos.y, pos.z

    def read_global_position(self) -> Tuple[float, float]:
        '''
        Read the drone's global position.
        '''
        while True:
            pos = self.mavconn.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            if pos:
                # scale down to real coordinates
                return float(pos.lat) * 1e-7, float(pos.lon) * 1e-7

    def wait_for_GPS_ready(self) -> None: 
        '''
        Wait for the GPS to be ready. 
        '''
        while True:
            gps = self.mavconn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if gps and gps.fix_type & mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
                break

    def wait_for_EKF_ready(self): 
        '''
        Wait for EKF to be ready.
        '''
        while True: 
            ekf = self.mavconn.recv_match(type="EKF_STATUS_REPORT", blocking=True) 
            if ekf and (ekf.flags & (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_POS_HORIZ_REL)):
                break

    def wait_for_prearm(self) -> None: 
        '''
        Wait for the drone to be prearmed.
        '''
        self.mavconn.recv_match(type="SYS_STATUS", condition = "SYS_STATUS.onboard_control_sensors_health & 0x10000000", blocking=True)

    def send_mode(self, mode: str) -> None:
        '''
        Send a new mode to the drone.
        '''
        mode_id = self.mavconn.mode_mapping()[Mode]
        self.mavconn.mav.set_mode_send(
            self.mavconn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    def send_arm_command(self) -> None: 
        '''
        Send the arm command to the drone.
        '''
        self.mavconn.mav.command_long_send(
            self.mavconn.target_system,
            self.mavconn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, # confirmation
            1, # param1 (1 to indicate arm)
            0, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def send_takeoff_command(self) -> None: 
        '''
        Send the takeoff command to the drone.
        '''
        pass 

    def send_return_to_home_command(self) -> None: 
        '''
        Send the return to home command.
        '''
        pass

    def send_local_position_target(self, target: Tuple[float, float, float]) -> None: 
        '''
        Send new local position target.

        target: x, y, z (meters)
        '''
        mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                )

        self.mavconn.mav.set_position_target_local_ned_send(
            0, 
            self.mavconn.target_system, 
            self.mavconn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            target[0], target[1], target[2],    # x, y, z
            0, 0, 0,    # vx, vy, vz
            0, 0, 0,    # ax, ay, az
            self.yaw, 0        # yaw, yaw rate
            )

    def send_global_position_target(self, target: Tuple[float, float], alt: int) -> None: 
        '''
        Send new global position target.

        target: (lattitude, longitude)
        '''
        mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                )

        # pack lat/lon into integer
        lat_i = int(target[0] * 1e7)
        lon_i = int(target[1] * 1e7)

        self.mavconn.mav.set_position_target_global_int_send(
            0,                              # time_boot_ms (ignored)
            self.mavconn.target_system,
            self.mavconn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mask,
            lat_i, lon_i, alt,              # position
            0, 0, 0,                        # velocity (ignored)
            0, 0, 0,                        # acceleration (ignored)
            0, 0                            # yaw, yaw_rate (ignored)
        )


class DroneController: 
    '''
    This class acts as the bridge from overall program logic to the MAVLink connection. This 
    class handles checklists, errors, and provides an easy interface to control the drone. 
    '''

    def __init__(self, connection_str: str) -> None: 
        '''
        Initiates a new instance. 
        '''
        self.mavlink = DroneMAVLink(connection_str)

    def takeoff(self):
        '''
        Tells the drone to takeoff.
        '''
        self.preflight_checklist()
        self.mavlink.send_arm_command() 
        self.mavlink.send_takeoff_command()

    def return_to_home(self):
        '''
        Tells the drone to return to home.
        '''
        pass

    def update_position_target(self, target: Tuple[float, float, float]):
        '''
        Updates the position target of the drone. 
        '''
        pass 

    def set_max_speed(self, max_speed):
        '''
        Sets the maximum speed of the drone. 
        '''
        pass

    def set_heading(self, heading):
        '''
        Sets the heading angle of the drone. 
        '''
        pass

    async def start_status_monitor(self, lowbat_cb, lostpos_cb, lostconn_cb):
        '''
        Monitors the drone's status asynchronously, calling callbacks when certain events
        occur.
        '''
        self.lowbat_cb = lowbat_cb 
        self.lostpos_cb = lostpos_cb 
        self.lostconn_cb = lostconn_cb

        # EVENTS: 
        # - low battery 
        # - position lost 
        # - connection broken
        pass

    def preflight_checklist(self):
        '''
        Goes through the preflight checklist.
        '''
        # EKF ready at the very least
        pass 
