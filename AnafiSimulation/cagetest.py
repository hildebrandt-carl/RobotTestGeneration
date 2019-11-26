# -*- coding: UTF-8 -*-

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
import signal
import sys

PHYSICAL_IP = "192.168.42.1"
SIMULATED_IP = "10.202.0.1"

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        drone(Landing()).wait()
        drone.disconnection()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

drone = olympe.Drone(PHYSICAL_IP)
drone.connection()
drone(TakeOff()).wait()
drone(moveBy(0,  0, -1, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  2, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(Landing()).wait()
drone.disconnection()