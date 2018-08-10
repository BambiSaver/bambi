#!/usr/bin/python
# Import mavutil
import sys
from time import sleep
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

master.wait_heartbeat()

print "Heartbeat received, sending message now"
#print mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE

print "TARGET (", master.target_system, ", ", 240, ")"

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
#    master.target_component,
    240,
#    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    2720,
    0,
    1, 0, 0, 10, 0, 0, 0);



# Check ACK
ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    print ack_msg

    # Check if command in the same in `arm_disarm`
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print "wrong acknoledged"
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break



sys.exit(0);

sleep(5);

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    print ack_msg

    # Check if command in the same in `arm_disarm`
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print "wrong acknoledged"
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break