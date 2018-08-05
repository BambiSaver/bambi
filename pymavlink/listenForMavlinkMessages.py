#!/usr/bin/python
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass