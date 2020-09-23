##########DEPENDENCIES#############

from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import math
import argparse
from pymavlink import mavutil


vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)
#########FUNCTIONS#################

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable!=True:
		print("Arm icin gerekli sartlar saglaniyor.")
		time.sleep(1)
	print("Drone arm edilebilir.")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("GUIDED moduna geciliyor.")
		time.sleep(1)
	print("Drone artik GUIDED modunda.")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Drone arm olmasi bekleniyor.")
		time.sleep(1)
	print("Drone arm oldu.")

	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Takeoff gerceklesti.")

	return None

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto(targetLocation):
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED": 
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.025:
            print("Reached target")
            break;
        time.sleep(2)

def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def set_servo(serno,pwm)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
        0,
        serno,      # param 1
        pwm,        # param 2
        0,          # param 3
        0,          # param 4
        0, 0, 0)    # param 5,6,7
    vehicle.send_mavlink(msg)

def land():
	print ("Drone inis yapiliyor.")
	vehicle.mode = VehicleMode("LAND")
	time.sleep(5)
	vehicle.armed= False

