
from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged

# Connection
drone = olympe.Drone("10.202.0.1")
drone.connection()

# Wait for GPS fix
drone(GPSFixStateChanged(_policy = 'wait'))
print("GPS position before take-off :", drone.get_state(HomeChanged))
    

# Take-off and print position
drone(TakeOff()).wait()

count = 0
while count < 10000:
    print("GPS position after take-off : ", drone.get_state(PositionChanged))
    count += 1

drone.disconnection()