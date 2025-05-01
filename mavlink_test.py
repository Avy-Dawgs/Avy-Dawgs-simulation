'''
Mavlink control test for Ardupilot copter.
'''
from typing import Tuple
import geopy 
from geopy.distance import geodesic
from pymavlink import mavutil
from time import sleep
import subprocess

# parameters
# HOME_LAT = 40.76996628730284 
# HOME_LON = -111.84630508969808
ALT = 10


def main():
    '''
    Entry point.
    '''

    # start mavlink connectoin as a TCP listener
    master: mavutil.mavtcpin = mavutil.mavlink_connection("tcpin:localhost:14550")

    print("TCP Listener initiated")

    ### run the Ardupilot SITL simulation as a subprocess ##
    # comment out if running SITL separately
    # ** this must happen after TCP listener initiated **
    # runs MAVproxy ground station, which forwards connection to this script via port 14550
    subprocess.Popen(["sim_vehicle.py", "-v", "ArduCopter", "-C", "tcp:localhost:5760", "--out", "tcp:localhost:14550", "--console", "--map"])

    master.wait_heartbeat()
    print("Heartbeat received")
    print("Target system: ", master.target_system, " Target component: ", master.target_component)

    # PrearmWait(master)

    # set origin and home
    # master.mav.set_gps_global_origin_send(1, LAT, LON, ALT)
    # master.mav.set_home_position_send(1, LAT, LON, ALT, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)
    # print("Sent origin and home position")

    # wait for EKF ready
    EKF_wait(master)
    print("EKF ready") 

    # read initial GPS coordinates
    lat, lon = GPS_read(master)

    # get list of coordinates to follow
    coor_list = gen_coordinate_list(lat, lon, 10, 100, 10)

    # set mode to guided
    print("Setting mode guided")
    send_mode(master, "GUIDED")

    master.wait_heartbeat()

    # wait for prearm
    prearm_wait(master)
    print("PreArm Complete, ready for takeoff")

    # send arm command
    print("Arming motors")
    send_arm_command(master)

    # wait for motors to be armed
    master.motors_armed_wait()
    print("Motors armed")

    # send takeoff command
    print("Sending takeoff command")
    send_takeoff_command(master, 10)

    # TODO test if this is needed 
    # removing may cause drone to land immediately due to sending a desination too soon
    sleep(5)

    # loop through coordinates
    for coor in coor_list:
        # send next destination 
        target_lat, target_lon = coor[0], coor[1]
        send_position_target(master, target_lat, target_lon, ALT)

        # wait for dest reached
        while True:
            new_lat, new_lon = GPS_read(master)

            # calculate errors based on GPS readings
            lat_err = abs(new_lat - target_lat)
            lon_err = abs(new_lon - target_lon)

            print(f"Lat error: {lat_err}     Lon error: {lon_err}")

            # check for within error threshold 
            # TODO find a better way to check location
            if lat_err <= 5e-7 and lon_err <= 5e-7:
                break

    # loop forever
    while True:
        pass


def GPS_read(MavConn: mavutil.mavfile) -> Tuple[float, float]:
    '''
    Read GPS coordinates.
    '''
    while True:
        gps = MavConn.recv_match(type="GPS_RAW_INT", blocking=True)
        if gps:
            # scale down to real coordinates
            return float(gps.lat) * 1e-7, float(gps.lon) * 1e-7


def GPS_wait(MavConn: mavutil.mavfile) -> None:
    '''
    Wait for GPS to be ready.
    '''
    while True:
        gps = MavConn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if gps and gps.fix_type & mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
            break


def EKF_wait(MavConn: mavutil.mavfile) -> None:
    '''
    Wait for EKF to be ready.
    '''
    while True: 
        ekf = MavConn.recv_match(type="EKF_STATUS_REPORT", blocking=True) 
        if ekf and (ekf.flags & (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_POS_HORIZ_REL)):
            break


def send_mode(MavConn: mavutil.mavfile, Mode: str) -> None:
    '''
    Send a new mode.
    '''
    mode_id = MavConn.mode_mapping()[Mode]
    MavConn.mav.set_mode_send(
        MavConn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )


def prearm_wait(MavConn: mavutil.mavfile) -> None:
    '''
    Wait for prearm.
    '''
    MavConn.recv_match(type="SYS_STATUS", condition = "SYS_STATUS.onboard_control_sensors_health & 0x10000000", blocking=True)


def send_arm_command(MavConn: mavutil.mavfile) -> None:
    '''
    Send arm command.
    '''
    MavConn.mav.command_long_send(
        MavConn.target_system,
        MavConn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, # confirmation
        1, # param1 (1 to indicate arm)
        0, # param2 (all other params meaningless)
        0, # param3
        0, # param4
        0, # param5
        0, # param6
        0) # param7


def send_position_target(MavConn: mavutil.mavfile, Lat: float, Lon: float, Alt: int) -> None:
    '''
    Send new position target.
    '''
    # pack lat/lon into integer
    lat_i = int(Lat * 1e7)
    lon_i = int(Lon * 1e7)

    type_mask = 0b0000111111111000

    MavConn.mav.set_position_target_global_int_send(
        0,                              # time_boot_ms (ignored)
        MavConn.target_system,
        MavConn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_i, lon_i, Alt,              # position
        0, 0, 0,                        # velocity (ignored)
        0, 0, 0,                        # acceleration (ignored)
        0, 0                            # yaw, yaw_rate (ignored)
    )


def send_takeoff_command(MavConn: mavutil.mavfile, Alt: int) -> None:
    '''
    Send takeoff command.
    '''
    # send takeoff command 
    MavConn.mav.command_long_send(
            MavConn.target_system,
            MavConn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            # float('nan'),
            0,
            0, # lat 
            0, # lon 
            Alt) # alt


def calc_new_coordinates(Lat1: float, Lon1: float, Dist, Bearing):
    '''
    Calculates destination coordinate given previous coordinate, 
    distance and bearing.
    '''
    origin = geopy.Point(Lat1, Lon1) 
    dest = geodesic(meters=Dist).destination(origin, Bearing)

    return dest[0], dest[1]


def gen_coordinate_list(StartLat: float, StartLon: float, NumPasses: int, Length, Width) -> list[Tuple]:
    '''
    Generate a list of coordinates.
    '''
    retval = []

    prev_coors = (StartLat, StartLon)

    
    for _ in range(int(NumPasses / 2)):
        # north
        next_lat, next_lon = calc_new_coordinates(prev_coors[0], prev_coors[1], Length, 90)
        retval.append((next_lat, next_lon))
        prev_coors = (next_lat, next_lon)

        # east
        next_lat, next_lon = calc_new_coordinates(prev_coors[0], prev_coors[1], Width, 0)
        retval.append((next_lat, next_lon))
        prev_coors = (next_lat, next_lon)

        # south
        next_lat, next_lon = calc_new_coordinates(prev_coors[0], prev_coors[1], Length, 270)
        retval.append((next_lat, next_lon))
        prev_coors = (next_lat, next_lon)

        # east
        next_lat, next_lon = calc_new_coordinates(prev_coors[0], prev_coors[1], Width, 0)
        retval.append((next_lat, next_lon))
        prev_coors = (next_lat, next_lon)

        prev_coors = (next_lat, next_lon)

    return retval


if __name__ == "__main__":
    main()
