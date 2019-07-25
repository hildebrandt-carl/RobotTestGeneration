#!/bin/zsh

# Change the ROS_MASTER_URI to allow multiple version of ROS to run at the same time
ROS_MASTER_URI=http://localhost:11317

# Source the ros workspace
source ROS_WS/devel/setup.zsh

# Creat a counter to count how many tests we have done
counter=8

# Create a temporary unity folder
cp -r ./Unity/Build ./Build25007

# Go into the directory
cd ./Build25007

# Get current directory
current_dir="$PWD"

# Change the port number inside the new build
sed -i -e 's/(25001)/(25007)/g' ./config.txt

# Run 3 tests
while [ $counter -le 9 ]
do

	# Get the current test
	cp ../TestGen/Results/BEBOP_Waypoint3/maps/map$counter/test.txt test.txt

	# Run the simulator
	./WorldEngine.x86_64 &

	# Get the PID so that I can kill it later
	unity_PID=$!

	# Wait 30 seconds for unity to start
	sleep 30

	# Launch the ros file
	roslaunch flightcontroller fly.launch port:="25007" test_location:="$current_dir" save_location:="$current_dir" &

	# Get the PID so that I can kill it later
	roslaunch_PID=$!

	# Each test is given 30 seconds
	sleep 60

	# Kill the code
	kill -INT $unity_PID
	kill -INT $roslaunch_PID

	# Remove the temporary test
	rm test.txt
	mv performance.txt ../TestGen/Results/BEBOP_Waypoint3/maps/map$counter/performance.txt

	# Allow 30 seconds for linux to clean up
	sleep 30

	# Increment the counter
	((counter++))

# End while loop
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build25007/

echo Completed Script
