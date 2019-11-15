First start the firmware
$ sudo systemctl start firmwared.service

If you want to check if the firmware is working:
$ fdc ping

To launch the simulator:
$ sphinx /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/sphinxfiles/drones/anafi4k.drone::stolen_interface=enp:eth0:192.168.42.1/24

You can reset the wifi name afterwards using:
$ ifconfig dev6 down  
$ ip link set dev6 name enp0  
$ ifconfig enp0 up 

We have to source the environment
$ source /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/code/parrot-groundsdk/./products/olympe/linux/env/shell





controlling without GPS you can send a message moveby which requests a move in m relative to the drone

That returns a 
ardrone3.PilotingEvent.moveByEnd(dX=-9.90627670288086, dY=0.034654825925827026, dZ=-0.00042492151260375977, dPsi=3.0688436368713494e-41, error=MoveByEnd_Error.ok)
message. I want to get that at all times 



Moveby takes in m
Moveto takes in long lat



Installing olympe





# Connecting to the parrot

## Via phone connection

Open the central arm on the anafi controller
connect your phone to the controller via a USB cable
Start the free flight 6
Click fly

It should connect automatically to the drone


### Changing the wifi password for anafi
In freeflight 6 open settings->network->password
Then enter a new wifi password

Current Password: autosoftlab

### Forgot wifi password
Hold the drones power button for 10 second to reset and reinitiate the password