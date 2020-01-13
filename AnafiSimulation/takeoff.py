# -*- coding: UTF-8 -*-

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff

# Simulation
# drone = olympe.Drone("10.202.0.1")

# Real life
drone = olympe.Drone("192.168.42.1")

drone.connection()
drone(TakeOff()).wait()
drone.disconnection()
