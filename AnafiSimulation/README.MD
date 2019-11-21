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




# Deleting Olympe
Delete the file code/Parrot-SDK
It should be in your home directory



# Reading X,Y,Z position from Drone
You can do this using datalog feature.

Run the sphinx simulator in datalog mode:
$ sphinx <path_to_drone>/anafi4k.drone::stolen_interface=enp0:eth0:192.168.42.1/24 --datalog

You can then access the data using:
$ tlm-data-logger inet:127.0.0.1:9060 | grep worldPosition


<!-- TODO: [DOES NOT SEEM TO WORK YET] -->
To speed up the datalogging run (note: rate is in ms): datalog-rate=<rate>
$ sphinx <path_to_drone>/anafi4k.drone::stolen_interface=enp0:eth0:192.168.42.1/24 --datalog --datalog-rate=10





# Connect to wifi on Main Machine

First start by turning the parrot drone on. The parrot drone will create a wifi hotspot which you can then connect to.

To connect we can use the ethernet wifi cards we have bought.

We need to assign static IP to the ether port. To do this run:

$ ifconfig
>>> It should be an enp1p* connection

Now assign the IP address:

$ sudo ifconfig enp1p1 192.168.1.2

Then launch the internet browser and connect to "192.168.1.252"











# Changing simulation state
Get the current state
$ echo '{"jsonrpc": "2.0", "method": "GetAllParameters", "params": {}, "id": 1}' | curl -d @- http://localhost:8383 | python -m json.tool

Turn the GPS off
$ echo '{"jsonrpc": "2.0", "method": "SetParam", "params": {"machine":"anafi4k", "object":"gps/gps", "parameter":"out_of_order", "value":"true"}, "id": 1}' | curl -d @- http://localhost:8383 | python -m json.tool

I am not sure how to make the compass not function properly