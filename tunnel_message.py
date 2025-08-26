
'''
Send a tunnel command once per second.
'''

from pymavlink import mavutil 
from time import sleep


def main(): 
    drone: mavutil.mavtcp = mavutil.mavlink_connection("tcp:localhost:5763", source_system=1, source_component=199)
    print("TCP Listener initiated")

    drone.wait_heartbeat()

    print("Heartbeat received")
    print("Target system: ", drone.target_system, " Target component: ", drone.target_component)

    i = 0
    while True: 
        if i == 256: 
            i = 0

        payload = [i] * 128

        drone.mav.tunnel_send(
                255,  # target sys
                0,  # target comp
                64, # payload type 
                0,  # payload length
                payload # payload
            )

            
        i += 1
        sleep(1)


if __name__ == "__main__": 
    main()
