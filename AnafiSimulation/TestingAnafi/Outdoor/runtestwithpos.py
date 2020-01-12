# -*- coding: UTF-8 -*-

import time
import olympe
import re
import argparse
import signal
import sys

from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, GPSFixStateChanged
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        drone(Landing()).wait()
        drone.disconnection()
        sf.close()
        sys.exit(0)

# Parse the input arguments
parser = argparse.ArgumentParser()
parser.add_argument('-n', '--test_number',
                    type=int,
                    required=True,
                    help='Select the test you want to run')
parser.add_argument('-t', '--test_type',
                    type=str,
                    required=True,
                    help='Select score type used (simulation), (outdoor)')
args = parser.parse_args()

# Get the test number
test_number = args.test_number

# Declare the drone IP
PHYSICAL_IP = "192.168.42.1"
SIMULATED_IP = "10.202.0.1"
DRONE_IP = None
if args.test_type == "simulation":
    DRONE_IP = SIMULATED_IP
elif args.test_type == "outdoor":
    DRONE_IP = PHYSICAL_IP
else:
    print("Drone type not known")
    exit()

# If you get signal.SIGINT send that to our singal_handler function
signal.signal(signal.SIGINT, signal_handler)

# Get the test name
testname = "./test/maps/map" +str(test_number)+ "/test.txt"
if DRONE_IP == SIMULATED_IP:
    savename = "./test/maps/map" +str(test_number)+ "/simulation_output.txt"
else:
    savename = "./test/maps/map" +str(test_number)+ "/outdoor_output.txt"

# Read the test file to get the positions
with open(testname, "r") as f:
    filedata = f.readlines()

# Open the save file
sf = open(savename,"w")

gp = []

# For each line in the data
for line in filedata:
    # Get the first character
    initial_char = line[0]
    # If the first character is a G save the location
    if initial_char == "G":
        # Get the goal locations
        result = re.search('[(](.*)[)]', line)
        goal_string = result.group(0)

        # Remove the first character '(' an last character ')' from the strong
        goal_string = goal_string[1:-1]

        # Get the goal positions
        goals = goal_string.split(',')

        # Add the goals to the final goal array (Y is inverted in test file)
        gp.append([ float(goals[0]), 
                    -1*float(goals[1]),
                    float(goals[2])])

# Need to convert the goals to be relative to each other
prevx = 0
prevy = 0
prevz = 0
newx = 0
newy = 0
newz = 0
rp = []
for i in range(0, len(gp)):
    print("Goal " + str(i) + ": " + str(gp[i]))
    newx = gp[i][0] - prevx
    newy = gp[i][1] - prevy
    newz = gp[i][2] - prevz
    relative_pos = [newx, newy, newz]
    print("Relative Position: " + str(relative_pos))
    rp.append(relative_pos)
    prevx = gp[i][0]
    prevy = gp[i][1]
    prevz = gp[i][2]
    print("")


with olympe.Drone(DRONE_IP) as drone:
    drone.connection()

    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy = 'wait'))
    print("GPS position before take-off :", drone.get_state(HomeChanged))

    drone(TakeOff()).wait()

    # Start a flying action asynchronously
    flyingAction = drone(
        FlyingStateChanged(state="hovering", _timeout=5)
        >> moveBy(rp[0][0], rp[0][1], -1*rp[0][2], 0)
        >> moveBy(rp[1][0], rp[1][1], -1*rp[1][2], 0)
        >> moveBy(rp[2][0], rp[2][1], -1*rp[2][2], 0)
        >> moveBy(rp[3][0], rp[3][1], -1*rp[3][2], 0)
        >> moveBy(rp[4][0], rp[4][1], -1*rp[4][2], 0)
        >> moveBy(rp[5][0], rp[5][1], -1*rp[5][2], 0)
        >> moveBy(rp[6][0], rp[6][1], -1*rp[6][2], 0)
        >> moveBy(rp[7][0], rp[7][1], -1*rp[7][2], 0)
        >> moveBy(rp[8][0], rp[8][1], -1*rp[8][2], 0)
        >> moveBy(rp[9][0], rp[9][1], -1*rp[9][2], 0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    )

    print(-1*rp[9][2])

    # Get the current time
    start_time = time.time()

    # Print out the drone details
    while 1:
        print("GPS position after take-off : ", drone.get_state(PositionChanged))
        sf.write("Latitude: " + str(drone.get_state(PositionChanged)['latitude']) + "\n")
        sf.write("Longitude: " + str(drone.get_state(PositionChanged)['longitude']) + "\n")
        sf.write("Altitude: " + str(drone.get_state(PositionChanged)['altitude']) + "\n")
        sf.write("Time: " + str(time.time() - start_time) + "\n")
        sf.write("-------------------------------------------\n")
        time.sleep(0.25)

    # Wait for the end of the flying action
    if not flyingAction.wait().success():
        raise RuntimeError("Cannot complete the flying action")

    # Leaving the with statement scope: implicit drone.disconnection()
print("Finished!")