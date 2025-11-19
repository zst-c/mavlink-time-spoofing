from pymavlink.generator import mavcrc
from pymavlink import mavutil

import hashlib
import time
import struct

from datetime import datetime as dt

import gnss_lib_py as glp

# Set the IP and PORT of the UAV for the UDP connection
# For MAVProxy to relay to the UAV in simulations, use port 5765
IP = "0.0.0.0"
PORT = 5760

# If relaying with a second GCS, ensure telemtry forwarding is enabled in that GCS as well!
# Create the connection to MAVProxy
master = mavutil.mavlink_connection(f"udpin:{IP}:{PORT}")

print(f"Waiting for heartbeat on {IP}:{PORT}")

# Wait for a heartbeat before generating the message or sending commands
master.wait_heartbeat()

print("Got heartbeat")

key = input("Enter the secret key: ").encode('ascii')

# Set the offset in seconds for the timestamp in the packet, from current time
# I.e. if this is 86400, offset the timestamp by one day
MODE = int(input("Enter whether you want to be in offset mode [0] or chosen mode [1]:\n"))

# ================================================================================ #

# Search for the message class in pymavlink.dialects.v20.common.py (or other message definition files), then fill out the fields below:

# Get MSG_ID from <class>.id
# 232 == GPS_INPUT
# (which we use to put MAV_CMD_DO_SET_MODE inside of)
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
MSG_ID = 232

# Specify the message's SEQ number
SEQ = 200

# Specify how the structure should be packed, using the string in <class>.unpacker
STRUCT_PACK = "<QIiifffffffffHHBBBH"

# Specify the payload arguments as a list (check the order of these from <class>.ordered_fieldnames)
# ["time_usec", "time_week_ms", "lat", "lon", "alt", "hdop", "vdop", "vn", "ve", "vd", "speed_accuracy", "horiz_accuracy", "vert_accuracy", "ignore_flags", "time_week", "gps_id", "fix_type", "satellites_visible", "yaw"]

POS = [514492880, -26083820]
ALT = 1

OFFSET = int(input(f"Enter a GPS {'offset' if MODE == 0 else 'time'} in SECONDS"))

unixNow = (int(time.time()) if MODE == 0 else 0) + OFFSET

gpsWeek, gpsTimeInWeek = glp.unix_millis_to_tow(unixNow * 1E3)

data = {
          # (uint64_t) Timestamp (MICROSECONDS since boot or Unix epoch)
          'time_usec': int(glp.tow_to_unix_millis(gpsWeek, gpsTimeInWeek) * 1E3),
          # (uint8_t) ID of the GPS for multiple GPS inputs
          'gps_id': 0,
          # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
          'ignore_flags': 0,
          # (uint32_t) GPS time (MILLISECONDS from start of GPS week)
          'time_week_ms': int(gpsTimeInWeek * 1E3),
          # (uint16_t) GPS week number
          'time_week': int(gpsWeek),                    
          # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
          'fix_type': 3,
          # (int32_t) Latitude (WGS84), in degrees * 1E7
          'lat': round(POS[0]),
          # (int32_t) Longitude (WGS84), in degrees * 1E7
          'lon': round(POS[1]),
          # (float) Altitude (AMSL, not WGS84), in m (positive for up)
          'alt': round(ALT),
          # (float) GPS HDOP horizontal dilution of position in m
          'hdop': 1,
          # (float) GPS VDOP vertical dilution of position in m
          'vdop': 1,
          # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
          'vn': 0,
          # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
          've': 0,
          # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
          'vd': 0,
          # (float) GPS speed accuracy in m/s
          'speed_accuracy': 0,
          # (float) GPS horizontal accuracy in m
          'horiz_accuracy': 0,
          # (float) GPS vertical accuracy in m
          'vert_accuracy': 0,
          # (uint8_t) Number of satellites visible.
          'satellites_visible': 13
        }

# https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.h
PAYLOAD = [data["time_usec"],
           data["time_week_ms"],
           data["lat"],
          data["lon"],
          data["alt"],
          data["hdop"],
          data["vdop"],
          data["vn"],
          data["ve"],
          data["vd"],
          data["speed_accuracy"],
          data["horiz_accuracy"],
          data["vert_accuracy"],
          data["ignore_flags"],
          data["time_week"],
          data["gps_id"],
          data["fix_type"],
          data["satellites_visible"],
          data["yaw"]]

# Get the CRC_EXTRA byte from <class>.crc_extra
# See: https://mavlink.io/en/guide/serialization.html#crc_extra
CRC_EXTRA = 151

# ================================================================================ #

# SYS, CMP & LINK IDs
# Specify that this comes from the GCS
SYSID = 255
CMPID = 230
LINKID = 1

# INC & CMP Flags
INCF = 1
CMPF = 0

MAGIC = 0xFD

# This is the MAVLink offset from the beginning of UNIX time
# see: https://mavlink.io/en/guide/message_signing.html#timestamp
MAVOFFSET = 1420070400

# ================================================================================ #

# Given the secret key as a bytearray, serialise the message defined at the top
# of the program
def serialiseMessage(key):

    # Pack the payload according to the packing arguments defined at the top of the program
    payload = struct.Struct(STRUCT_PACK).pack(*PAYLOAD)

    # Truncate the payload's 0-bytes
    length = len(payload)

    while (length > 1 and payload[length - 1] == 0):
        length-=1

    payload = payload[:length]

    magic = bytearray([MAGIC])
    link = bytearray([LINKID])

    # Serialise the body into bytes
    body = bytearray(
        [
        len(payload), # Payload Length
        INCF, # INC flag
        CMPF, # CMP flag
        SEQ, # SEQ
        SYSID, # SYS ID
        CMPID, # CMP ID
        ]
        + list(MSG_ID.to_bytes(3, "little")) # MSG ID
        + list(payload) # Payload
    )

    # Generate the message's CRC bytes
    crc = mavcrc.x25crc_slow(body + bytearray([CRC_EXTRA])).crc.to_bytes(2, "little")

    # Hash the message with SHA256, with the key appended (to generate the signature)
    m = hashlib.sha256()
    m.update(key)
    m.update(magic)
    m.update(body)
    m.update(crc)
    m.update(link)

    # Time in offset mode
    if(MODE == 0):
        TIME = time.time() + OFFSET

    # Time in chosen mode
    if(MODE == 1):
    # Generate the timestamp as an offset from current time
        TIME = dt.fromtimestamp(OFFSET)

    timestamp = bytearray(int((TIME - MAVOFFSET)*1e5).to_bytes(6, "little"))

    m.update(timestamp)

    # Truncate the hash to only 6 bytes in length
    hash = bytearray([x for x in m.digest()[:6]])

    # Return the fully serialised packet
    return (magic + body + crc + link + timestamp + hash)

# ================================================================================ #

# Ask the user for the secret key, and convert that to bytes in the same way MAVProxy does. See:
# https://github.com/ArduPilot/MAVProxy/blob/ 594d661ffe81a4b8b362963e0604cbcea46cc16a/MAVProxy/modules/mavproxy_signing.py#L39
k = hashlib.sha256()
k.update(key)
key = k.digest()

# Generate the serialised packet
packet = serialiseMessage(key)

print("Sending packet to switch to LAND mode")

# Print the packet for the user
for b in packet:
    print(f"{b:02X}", end=" ")
print("\n")

buf = list(packet)
timeInt = int.from_bytes(bytearray(buf[44:50]), byteorder="little", signed=False) # ten micro sec units since jan 2015

timeInt *= 1e-5 # convert into seconds
timeInt += MAVOFFSET # ten microsecond units since start
print("Time in signature:", dt.fromtimestamp(timeInt).strftime('%Y-%m-%d %H:%M:%S'))

# Send the packet to the UAV
# https://github.com/ArduPilot/pymavlink/blob/ f7dd7261f01181fb9ff824a9bd8906e3ec0a4beb/mavutil.py#L1105
master.write(packet)

print("Sent packet")