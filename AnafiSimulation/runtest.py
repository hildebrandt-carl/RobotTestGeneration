# -*- coding: UTF-8 -*-

import time
import olympe
import re
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, GPSFixStateChanged

filename = "test.txt"

# Read the test file to get the positions
with open(filename, "r") as f:
    filedata = f.readlines()

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

        # Remove the first character '(' an last charcter ')' from the strong
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


with olympe.Drone("10.202.0.1") as drone:
    drone.connection()

    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy = 'wait'))
    print("GPS position before take-off :", drone.get_state(HomeChanged))

    # Start a flying action asynchronously
    flyingAction = drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
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
        >> Landing()
    )

    # Wait for the end of the flying action
    if not flyingAction.wait().success():
        raise RuntimeError("Cannot complete the flying action")

    # Leaving the with statement scope: implicit drone.disconnection()
print("Finished!")