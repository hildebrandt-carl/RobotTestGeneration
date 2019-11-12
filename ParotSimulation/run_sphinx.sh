#!/bin/zsh

run_number=1
MAC="e0:d5:5e:25:d4:6a"
mainfolder='PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_kinematic_waypoint'

# Create a temporary directory to work out of
mkdir run_sphinx$run_number
cd run_sphinx$run_number

# Get current directory
current_dir="$PWD"

# Copy the mavlink updater file
cp ../../TestGen/MavlinkUpdater.py MavlinkUpdater.py

for map in {1..231}
do

    # Get the working folder
    workingfolder="../../TestGen/Results/$mainfolder/maps/map$map"

    # Find what the network interface name is
    current_name=`ifconfig | grep $MAC | awk '{print $1}'`

    # Check the network interface exists
    if [ "$current_name" = "" ]; 
    then
        echo "Network MAC - $MAC does not exist"
    fi

    # Check if network name changes are required
    if [ "$current_name" = "enp0" ]; 
    then
        echo "No changes required"
    else
        echo "Changing network name"
        sudo ifconfig dev6 down  
        sleep 5
        sudo ip link set dev6 name enp0  
        sudo ifconfig enp0 up
        sleep 10
    fi

    # Launch the firmware
    echo "Starting firmware"
    sudo systemctl start firmwared.service
    # Allow the firmware to launch
    sleep 10

    # Launch the simulator
    echo "Launching simulator"
    sphinx /home/autosoftlab/Desktop/RobotTestGeneration/ParotSimulation/sphinxfiles/worlds/empty.world /home/autosoftlab/Desktop/RobotTestGeneration/ParotSimulation/sphinxfiles/drones/bebop2_local.drone::with_front_cam=true &
    # Get the sphinx ID
    sphinx_ID=$!
    # Allow the simulator to start
    sleep 15

    # Source the ros files
    echo "Launching ROS driver"
    source ../ros_ws/devel/setup.zsh
    # Launch the ROS script
    roslaunch bebop_utils automatic_control.launch ip:="10.202.0.1" save_location:="$current_dir" &
    # Get the sphinx ID
    roslaunch_ID=$!
    # Let ROS launch
    sleep 15

    # Copy the waypoint file
    echo "Getting Mavlink file and converting it"
    cp $workingfolder/flightplan_raw.mavlink flightplan_raw.mavlink
    # Convert the flightplan_raw file to work in the simulation
    python3 MavlinkUpdater.py --latitude 48.8789 --longitude 2.36778 --altitude 0.0
    # Upload the waypoint file
    curl -T flightplan.mavlink ftp://10.202.0.1:61/
    sleep 2

    # Start the waypoint file
    echo "Starting autonomous flight"
    rostopic pub --once bebop/autoflight/start std_msgs/String '{data: flightplan.mavlink}'

    # Wait 90 seconds to complete the test
    sleep 180

    # Stop the simulator and ros code
    kill -INT $roslaunch_ID
    kill -INT $sphinx_ID

    # Kill all other gazebo processors running
    killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 sphinx
    gazebo_id=`ps aux | grep parrot-sphinx | grep root | grep Ss | awk '{print $2}'`
    sudo kill -9 $gazebo_id

    # Save the performance and flightplan
    cp flightplan.mavlink $workingfolder/flightplan_bebop2.mavlink
    cp performance.txt $workingfolder/performance_bebop2.txt

    # Delete the waypoint file
    rm flightplan.mavlink
    rm flightplan_raw.mavlink
    rm performance.txt

    # Let everything die
    sleep 5

done

cd ..
rm -r run_sphinx1

echo Completed Script