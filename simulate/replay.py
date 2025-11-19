from pymavlink import mavutil

# Set the IP and PORT of the UAV for the UDP connection
# For MAVProxy to relay to the UAV in simulations, use port 5760
IP = "0.0.0.0"
PORT = 5760

# ================================================================================ #

# Create the connection to MAVProxy
master = mavutil.mavlink_connection(f"udpin:{IP}:{PORT}")

# Wait for a heartbeat before generating the message or sending commands
master.wait_heartbeat()

# Ask the user for the given message, in the format received from Wireshark when using "copy as hex stream"
# I.e. of the style: fd160100e30101c100000000000000000000000000000000000000000000 7f03514800b363a6a47c1d3e1b40d7896e
packet = input("Enter the packet, as bytes: ")

# Strip the stream and convert it to hex
packet = bytearray.fromhex(packet.strip())

# Print the packet for the user
for b in packet:
    print(f"{b:02X}", end=" ")
print("\n")

# Send the packet to the UAV
# https://github.com/ArduPilot/pymavlink/blob/ f7dd7261f01181fb9ff824a9bd8906e3ec0a4beb/mavutil.py#L1105
master.write(packet)

print("Sent packet")