from pymavlink.generator import mavcrc
from pymavlink import mavutil

import hashlib
from datetime import datetime as dt
import struct

# Set the IP and PORT of the UAV for the UDP connection
# For MAVProxy to relay to the UAV in simulations, use port 5760
IP = "127.0.0.1"
PORT = 5760

# Create the connection to MAVProxy
master = mavutil.mavlink_connection(f"udpin:{IP}:{PORT}")

print("Waiting for heartbeat")

# Wait for a heartbeat before generating the message or sending commands
master.wait_heartbeat()

print("Connected to UAV")

while True:
    message = master.recv_match(blocking=True)
    if(message.get_type() == "TIMESYNC"):
        print("===TIMESYNC===")
        d = message.to_dict()
        print(d)
        if d["tc1"] == 0:
            print("Syncing")
            print("Syncing time: ", dt.fromtimestamp(int(d["ts1"])*1e-9).strftime('%Y-%m-%d %H:%M:%S'))
        else:
            print("Responding time: ", dt.fromtimestamp(int(d["tc1"])*1e-9).strftime('%Y-%m-%d %H:%M:%S'))
            print("Syncing time: ", dt.fromtimestamp(int(d["ts1"])*1e-9).strftime('%Y-%m-%d %H:%M:%S'))
        
