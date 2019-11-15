
from __future__ import print_function, absolute_import
from random import random
from time import sleep

import threading
import pygame
import sys
import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile


class DroneController:

    def __init__(self, drone=None, speed=65, refresh_move=0.2):
        """"""
        try:
            self._quit_pressed = None

            # Launch pygame controller
            pygame.init()
            self.joy = pygame.joystick.Joystick(0)  

        # No controller connected
        except pygame.error as e:
            print(e)
            print("\n(There is no joystick connected to the system)")
            sys.exit(0)


    def start(self):
        # Initilize the joystick
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))

        # Start the flying loop
        self._mainloop()


    def stop(self):
        self._quit_pressed = True



    def _get_joy_values(self):
        pygame.event.pump()

        #Read input from the two joysticks and take only the ones we need
        out_joys = [self.joy.get_axis(i) for i in [0,1,3,4]]

        return out_joys


    def _is_takeoff_pressed(self):
        return self.joy.get_button(3) == 1


    def _is_landed_pressed(self):
        return self.joy.get_button(0) == 1


    def _check_quit_pressed(self):
        self._quit_pressed = self.joy.get_button(8) == 1


    def _mainloop(self):
        """"""
        while not self._quit_pressed:
            sleep(0.2)
            joy_values = self._get_joy_values()
            
            if self._is_takeoff_pressed():
                print("Pressed takeoff button!")
            elif self._is_landed_pressed():
                print("Pressed landing button!")
            else:
                print(joy_values)
            
            self._check_quit_pressed()

        print("\n============")
        print("Pressed QUIT button (X)")
        print("============\n")






if __name__ == "__main__":
    try:
        x = DroneController(None)
        x.start()
    except KeyboardInterrupt:
        x.stop()

    print("Teleoperating stopped\n")
    sys.exit(0)