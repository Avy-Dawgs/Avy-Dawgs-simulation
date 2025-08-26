from pymavlink import mavutil
import time

start_time = time.time()

master = mavutil.mavlink_connection("tcp:localhost:5762", source_system=1, source_component=199)

master.wait_heartbeat() 

count = 0.0
up_count = True
while True: 
    if up_count: 
        if count >= 1:
            up_count = False
            count -= 0.25
        else:
            count += 0.25
    else: 
        if count <= 0: 
            up_count = True
            count += 0.25
        else: 
            count -= 0.25

    master.mav.named_value_float_send(
            int((time.time() - start_time) * 1e3), 
            b"message", 
            count
            )
    print("sent")

    time.sleep(1)

