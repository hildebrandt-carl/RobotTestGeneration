#!/bin/zsh

# Source the ros workspace
source ROS_WS/devel/setup.zsh

# Creat a counter to count how many tests we have done
counter=1

# Run 3 tests
while [ $counter -le 134 ]
do

	# Get the current test
	cp TestRunning/Tests/test$counter.txt test.txt

	# Run the simulator
	./Unity/Build/WorldEngine.x86_64 &

	# Get the PID so that I can kill it later
	unity_PID=$!

	# Wait 30 seconds for unity to start
	sleep 30

	# Launch the ros file
	roslaunch flightcontroller fly.launch &

	# Get the PID so that I can kill it later
	roslaunch_PID=$!

	# Each test is given 30 seconds
	sleep 30

	# Kill the code
	kill -INT $unity_PID
	kill -INT $roslaunch_PID

	# Remove the temporary test
	rm test.txt
	mv performance.txt TestRunning/Results/performance$counter.txt

	# Allow 30 seconds for linux to clean up
	sleep 30

	# Increment the counter
	((counter++))

# End while loop
done

echo Completed Script
