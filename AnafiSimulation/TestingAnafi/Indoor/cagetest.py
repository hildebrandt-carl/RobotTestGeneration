# -*- coding: UTF-8 -*-

import time
import olympe
import re
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, GPSFixStateChanged
import signal
import sys
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
import time

print('---------------------------------------')
print('---------------------------------------')
print('---------------------------------------')
print('---------------------------------------')
print("here")
print('---------------------------------------')
print('---------------------------------------')
print('---------------------------------------')
print('---------------------------------------')

PHYSICAL_IP = "192.168.42.1"
SIMULATED_IP = "10.202.0.1"
DRONE_IP = PHYSICAL_IP

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        drone(Landing()).wait()
        drone.disconnection()
        sf.close()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if DRONE_IP == SIMULATED_IP:
    savename = "./simulation_output_nogps.txt"
else:
    savename = "./outdoor_output_nogps.txt"

# Open the save file
sf = open(savename,"w")

with olympe.Drone(DRONE_IP) as drone:
    drone.connection()

    # Wait for GPS fix
    # drone(GPSFixStateChanged(_policy = 'wait'))
    # print("GPS position before take-off :", drone.get_state(HomeChanged))

    drone(TakeOff()).wait()

    # Start a flying action asynchronously
    flyingAction = drone(
        FlyingStateChanged(state="hovering", _timeout=5)
		>> moveBy(0, 0, -0.5, 0)
		>> moveBy(0, -1, 0, 0)
		>> moveBy(1, 0, 0, 0)
		>> moveBy(0, 1, 0, 0)
		>> moveBy(-1, 0, 0, 0)
		>> moveBy(0, -1, 0, 0)
		>> moveBy(1, 0, 0, 0)
		>> moveBy(0, 1, 0, 0)
		>> moveBy(-1, 0, 0, 0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    )

    while 1:
        print("GPS position after take-off : ", drone.get_state(PositionChanged))
        sf.write("Latitude: " + str(drone.get_state(PositionChanged)['latitude']) + "\n")
        sf.write("Longitude: " + str(drone.get_state(PositionChanged)['longitude']) + "\n")
        sf.write("Altitude: " + str(drone.get_state(PositionChanged)['altitude']) + "\n")
        sf.write("-------------------------------------------\n")
        time.sleep(0.25)

    # Wait for the end of the flying action
    if not flyingAction.wait().success():
        raise RuntimeError("Cannot complete the flying action")

    # Leaving the with statement scope: implicit drone.disconnection()
print("Finished!")