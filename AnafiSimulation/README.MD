# Anafi

## Installing Anafi

To install Anafi firmware and its simulator you need to run the following:
```
$ echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
$ sudo apt-get update
$ sudo apt-get install parrot-sphinx
```

Make sure you put your computer's username into when prompted. Next we are going to make a udev rule to make sure you wifi card names aren't changed after each run. To do that you can run:
```
sudo nano /etc/udev/rules.d/76-netnames.rules
>>> SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:1b:21:a7:54:0c", NAME="enp2f0"
>>> SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:1b:21:a7:54:0d", NAME="enp2f1"
>>> SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:1b:21:a7:54:0e", NAME="enp2f2"
>>> SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:1b:21:a7:54:0f", NAME="enp2f3"
>>> SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="e0:d5:5e:25:d4:6a", NAME="enp1f0"
```

Now we have to launch the drone firmware. First start the firmware
```
$ sudo systemctl start firmwared.service
```

If you want to check if the firmware is working:
```
$ fdc ping
```

Make sure to add your mac address for each of the cards. You can then launch the sphinx simulator using:
```
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=enp2f0:eth0:192.168.42.1/24 /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/sphinxfiles/worlds/empty.world
```

Make sure to add your ethernets name instead of `enp2f0`.


Next we need to install Olympe so that we can run the python code to fly the drone. To do that you can run
```
$ sudo apt-get install repo -y
$ cd $HOME
$ mkdir -p code/parrot-groundsdk
$ cd code/parrot-groundsdk
$ pwd
$ repo init -u https://github.com/Parrot-Developers/groundsdk-manifest.git
$ repo sync
```

Install the parrot dependencies using:
```
$ cd ~/code/parrot-groundsdk
$ ./products/olympe/linux/env/postinst
```

Finally we need to build the repo. To do that you can run:
```
$ cd ~/code/parrot-groundsdk
$ ./build.sh -p olympe-linux -A all final -j
```

To run Olympe code you can type:
```
$ source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
```

Then run the python code you want to run. For instance
```
python3 joycontoller.py
```


## Running Simulation

Once everything is setup all you need to do to run the drone is to type:
Terminal 1:
```
$ sudo systemctl start firmwared.service
$ sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=enp2f0:eth0:192.168.42.1/24 /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/sphinxfiles/worlds/empty.world
```

Terminal 2:
```
$ cd ~/Desktop/RobotTestGeneration/AnafiSimulation
$ source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
$ python3 joycontoller.py
```



## Other Notes:

### Resetting Network Cards Name

There are times when the network cards name will be permentally changed. To reset it you can do the following:

First start the firmware
```
$ sudo systemctl start firmwared.service
```

If you want to check if the firmware is working:
```
$ fdc ping
```

To launch the simulator:
```
$ sphinx /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/sphinxfiles/drones/anafi4k.drone::stolen_interface=enp:eth0:192.168.42.1/24
```

You can reset the wifi name afterwards using:
```
$ ifconfig dev6 down  
$ ip link set dev6 name enp0  
$ ifconfig enp0 up 
```

### Sourcing the Olympe Environment

We have to source the environment in order to use the packages:
```
$ source /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/code/parrot-groundsdk/./products/olympe/linux/env/shell
```

### Moving the quadrotor using Olympe

Controlling without GPS you can send a message `moveby` which requests a move in meters relative to the drone. This returns the following message:
```
ardrone3.PilotingEvent.moveByEnd(dX=-9.90627670288086, dY=0.034654825925827026, dZ=-0.00042492151260375977, dPsi=3.0688436368713494e-41, error=MoveByEnd_Error.ok)
message. I want to get that at all times 
```

You can use the following commands:
* `Moveby`: Give a command in meters to change the position by that amount in terms of x, y, and z
* `Moveto`: Give a coordinate in terms of long lat and alt


### Deleting Olympe

Delete the file `code/Parrot-SDK`. It should be in your home directory


### Reading X,Y,Z position from Drone

You can do this using a datalog feature.

Run the sphinx simulator in datalog mode:
```
$ sphinx <path_to_drone>/anafi4k.drone::stolen_interface=enp0:eth0:192.168.42.1/24 --datalog
```

You can then access the data using:
```
$ tlm-data-logger inet:127.0.0.1:9060 | grep worldPosition
```

**This does not seem to work** -- To speed up the data logging run (note: rate is in ms): datalog-rate=<rate>
```
$ sphinx <path_to_drone>/anafi4k.drone::stolen_interface=enp0:eth0:192.168.42.1/24 --datalog --datalog-rate=10
```






## Flying in the Real World

### Via phone connection

To fly the Anafi drone using your phone use the following:

* Open the central arm on the Anafi controller
* Connect your phone to the controller via a USB cable
* Start the free flight 6
* Click fly

It should connect automatically to the drone


### Changing the wifi password for Anafi

To change the wifi password do the following

* In freeflight 6 open `settings->network->password`
* Then enter a new wifi password

**Current Password:** autosoftlab


### Resetting the wifi password for anafi

Hold the drones power button for 10 second to reset and reinitiate the password


### Connect to wifi on Main Machine

First start by turning the parrot drone on. The parrot drone will create a wifi hotspot which you can then connect to.

To connect we can use the ethernet wifi cards we have bought.

We need to assign static IP to the ether port. To do this run:
```
$ ifconfig
>>> It should be an enp1p* connection
```

Now assign the IP address:
```
$ sudo ifconfig enp1p1 192.168.1.2
```

Then launch the internet browser and connect to "192.168.1.252"











# Changing simulation state
Get the current state
$ echo '{"jsonrpc": "2.0", "method": "GetAllParameters", "params": {}, "id": 1}' | curl -d @- http://localhost:8383 | python -m json.tool

Turn the GPS off
$ echo '{"jsonrpc": "2.0", "method": "SetParam", "params": {"machine":"anafi4k", "object":"gps/gps", "parameter":"out_of_order", "value":"true"}, "id": 1}' | curl -d @- http://localhost:8383 | python -m json.tool

I am not sure how to make the compass not function properly


