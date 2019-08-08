#!/bin/zsh

# Change the ROS_MASTER_URI to allow multiple version of ROS to run at the same time
ROS_MASTER_URI=http://localhost:11313

# Source the ros workspace
source ROS_WS/devel/setup.zsh

# Create a temporary unity folder
cp -r ./Unity/Build ./Build25003

# Go into the directory
cd ./Build25003

# Get current directory
current_dir="$PWD"

# Change the port number inside the new build
sed -i -e 's/(25001)/(25003)/g' ./config.txt

for seedcounter in 10
do
	for depthcounter in 10
	do
		for beamcounter in 5 100 10
		do
			# Get the total number of tests to run 
			mapcounter=1
			totaltests=$(ls ../TestGen/Results/MIT_seed$seedcounter\_depth$depthcounter\_nodes1000_res2_beamwidth$beamcounter\_baseline0/maps | wc -l)

			echo "--------------------------------------------------------"
			echo "Processing: TestGen/Results/MIT_seed$seedcounter\_depth$depthcounter\_nodes1000_res2_beamwidth$beamcounter\_baseline0"
			echo "Total tests found: $totaltests"
			echo "--------------------------------------------------------"

			while [ $mapcounter -le $totaltests ]
			do

				echo "Processing: MIT_seed$seedcounter\_depth$depthcounter\_nodes1000_res2_beamwidth$beamcounter\_baseline0/maps/map$mapcounter"
				echo " "

				# Get the current test
				cp ../TestGen/Results/MIT_seed$seedcounter\_depth$depthcounter\_nodes1000_res2_beamwidth$beamcounter\_baseline0/maps/map$mapcounter/test.txt test.txt

				# Run the simulator
				./WorldEngine.x86_64 &

				# Get the PID so that I can kill it later
				unity_PID=$!

				# Wait 30 seconds for unity to start
				sleep 30

				# Launch the ros file
				roslaunch flightcontroller fly.launch port:="25003" test_location:="$current_dir" save_location:="$current_dir" &

				# Get the PID so that I can kill it later
				roslaunch_PID=$!

				# Each test is given 30 seconds
				sleep 75

				# Kill the code
				kill -INT $unity_PID
				kill -INT $roslaunch_PID

				# Remove the temporary test
				rm test.txt
				mv performance.txt ../TestGen/Results/MIT_seed$seedcounter\_depth$depthcounter\_nodes1000_res2_beamwidth$beamcounter\_baseline0/maps/map$mapcounter/performance.txt

				# Allow 30 seconds for gezbo to clean up
				sleep 30

				# Increment the mapcounter
				((mapcounter++))

			# End mapcounter
			done
		# End beamcounter
		done
	# End depthcounter
	done
# End seedcounter
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build25003/

echo Completed Script
